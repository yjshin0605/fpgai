`timescale 1ns / 1ps
// ============================================================================
// traj_rec : traj_frame 이 내보내는 프레임별 (p, m, c) 를 레코드(N_FR 프레임) 단위로 모아
//            궤적 특징 레지스터 13개를 계산한다.  (freq_features.py 와 비트 단위로 같음)
//
//   활성 프레임 : 앞단이 보낸 무효 표시가 0 이고, 프레임 번호가 skip_frames 이상인 프레임
//                 (앞단 정규화가 프레임마다 세기를 맞추므로, 크기 비교로는 판정할 수 없다.
//                  앞단은 정규화 전 RMS 와 peak hold 로 판정하므로 그 결과를 그대로 쓴다.)
//   PASS  (프레임마다 6클럭)  : n_active, bw_sum, 피크 칸 히스토그램, 이웃 프레임 비교
//                              (n_pairs, n_zero, n_small, n_jump, mono), 3프레임 비교
//                              (curv_sum, n_triples). p 와 활성 여부를 레지스터 배열에 복사
//   SCANH (칸마다 3클럭)      : 히스토그램에서 hmax(최대값), nocc(2 이상인 칸 수). 읽은 칸은 0으로
//   REP   (비교마다 1클럭)    : 지연 L = REP_MIN..REP_MAX 마다
//                              "k 와 k-L 이 둘 다 활성이고 |p[k]-p[k-L]| <= 1" 인 수를 세어
//                              가장 큰 값(rep_max)과 그때의 L(rep_lag, 같으면 작은 L)
//
// 처리 시간: 127*6 + 256*3 + 약 5,900 ≈ 7,500클럭. 다음 레코드(127프레임)는 3만 클럭 이상에
// 걸쳐 들어오고, 기록 메모리가 은행 2개라 계산 중에도 다음 레코드를 받을 수 있다.
// ============================================================================
module traj_rec #(
    parameter MAG_W        = 16,
    parameter N_FR         = 127,       // 레코드당 프레임 수
    parameter SMALL_STEP   = 4,
    parameter REP_MIN      = 2,
    parameter REP_MAX      = 63
)(
    input  wire             clk,
    input  wire             rstn,

    // traj_frame 출력을 그대로 연결
    input  wire             fr_valid,           // 프레임 값이 들어옴 (1클럭 펄스)
    input  wire [7:0]       fr_p,               // 피크 칸 번호 (fftshift 순서)
    input  wire [MAG_W-1:0] fr_m,               // 피크 크기
    input  wire [8:0]       fr_c,               // 순간 대역폭 칸 수
    input  wire             fr_invalid,         // 앞단의 무효 표시 (1이면 잡음뿐인 프레임)
    input  wire [7:0]       fr_frame,           // 프레임 번호 0~126
    input  wire [7:0]       skip_frames,        // 이 번호보다 작은 프레임은 계산에서 뺀다
                                                // 0이면 전부 사용. DC 수렴 구간을 뺄 때만 24 등으로

    output reg              regs_valid,         // 아래 13개가 새로 나옴 (1클럭 펄스)
    output reg  [31:0]      n_active, n_pairs, n_zero, n_small, n_jump, mono,
    output reg  [31:0]      curv_sum, n_triples, rep_max, rep_lag, hmax, nocc, bw_sum,
    output wire             busy
);

    localparam RW = 1 + 9 + MAG_W + 8;          // 기록 한 칸 = {act, c, m, p}

    // ==================================================================
    // 1. 프레임 기록 (쓰기 쪽)
    // ==================================================================
    reg [RW-1:0]    rec_mem [0:511];            // 주소 = {은행, 프레임 번호}
    reg             wbank;
    reg [7:0]       wcnt;
    // 활성 판정: 무효 표시가 0 이고, 프레임 번호가 skip_frames 이상
    wire w_act = (!fr_invalid) && (fr_frame >= skip_frames);

    reg             start;
    reg             s_bank;

    always @(posedge clk) begin
        if (fr_valid)
            rec_mem[{wbank, wcnt}] <= {w_act, fr_c, fr_m, fr_p};
    end

    always @(posedge clk) begin
        if (!rstn) begin
            wbank <= 1'b0;  wcnt <= 8'd0;
            start <= 1'b0;  s_bank <= 1'b0;
        end else begin
            start <= 1'b0;
            if (fr_valid) begin
                if (wcnt == N_FR - 1) begin
                    start  <= 1'b1;
                    s_bank <= wbank;
                    wbank  <= ~wbank;
                    wcnt   <= 8'd0;
                end else begin
                    wcnt <= wcnt + 8'd1;
                end
            end
        end
    end

    // ==================================================================
    // 2. 메모리 읽기 포트, 히스토그램, p·활성 레지스터 배열
    // ==================================================================
    reg  [8:0]      raddr;
    reg  [RW-1:0]   rdata;
    always @(posedge clk) rdata <= rec_mem[raddr];

    reg  [7:0]      hist [0:255];
    reg  [7:0]      h_raddr, h_waddr;
    reg  [7:0]      h_rdata, h_wdata;
    reg             h_we;
    integer q;
    initial for (q = 0; q < 256; q = q + 1) hist[q] = 8'd0;
    always @(posedge clk) begin
        if (h_we) hist[h_waddr] <= h_wdata;
        h_rdata <= hist[h_raddr];
    end

    reg  [7:0]      pa [0:255];                 // 프레임별 피크 칸 (반복성 계산용)
    reg             aa [0:255];                 // 프레임별 활성 여부

    // ==================================================================
    // 3. 계산 FSM
    // ==================================================================
    localparam S_IDLE = 3'd0, S_PASS = 3'd1, S_SCANH = 3'd2, S_REP = 3'd3, S_DONE = 3'd4;
    reg [2:0]       state;
    reg [2:0]       step;
    reg [8:0]       idx;                        // PASS: 프레임, SCANH: 칸
    reg             c_bank;

    reg [7:0]       cur_p;
    reg [8:0]       cur_c;
    reg             cur_act;
    reg [7:0]       p1, p2;                     // 한·두 프레임 전 피크 칸
    reg             a1, a2;                     // 한·두 프레임 전 활성

    reg [7:0]       r_active, r_pairs, r_zero, r_small, r_jump, r_triples;
    reg signed [9:0] r_mono;                    // (+이동 수) - (-이동 수)
    reg [17:0]      r_curv;
    reg [15:0]      r_bw;
    reg [7:0]       r_hmax;
    reg [8:0]       r_nocc;

    reg [7:0]       lag;                        // REP: 지금 지연 L
    reg [7:0]       rk;                         // REP: 지금 프레임 k
    reg [7:0]       rcnt, best, best_lag;

    // PASS 조합 신호 (rdata 도착 시점)
    wire [7:0]       rd_p   = rdata[7:0];
    wire [MAG_W-1:0] rd_m   = rdata[MAG_W+7:8];
    wire [8:0]       rd_c   = rdata[MAG_W+16:MAG_W+8];
    wire             rd_act = rdata[RW-1];             // 저장해 둔 활성 비트

    // 이웃 프레임 비교 (step 4 에서 사용)
    wire signed [9:0]  d    = $signed({2'b00, cur_p}) - $signed({2'b00, p1});
    wire        [9:0]  ad   = d[9] ? -d : d;
    wire               pair = cur_act && a1 && (idx >= 9'd1);
    wire               tri3  = cur_act && a1 && a2 && (idx >= 9'd2);
    wire signed [10:0] dd   = $signed({3'b000, cur_p}) - $signed({2'b00, p1, 1'b0}) + $signed({3'b000, p2});
    wire        [10:0] add  = dd[10] ? -dd : dd;
    wire               is_small = (ad >= 10'd1) && (ad <= SMALL_STEP);

    // 반복성 비교 (1클럭 1회)
    wire [7:0]       pk    = pa[rk];
    wire [7:0]       pkl   = pa[rk - lag];
    wire             both  = aa[rk] && aa[rk - lag];
    wire             close = (pk >= pkl) ? (pk - pkl <= 8'd1) : (pkl - pk <= 8'd1);
    wire [7:0]       rcnt_n = rcnt + ((both && close) ? 8'd1 : 8'd0);

    assign busy = (state != S_IDLE);

    always @(posedge clk) begin
        if (!rstn) begin
            state <= S_IDLE;  step <= 3'd0;  idx <= 9'd0;  raddr <= 9'd0;
            h_raddr <= 8'd0;  h_waddr <= 8'd0;  h_wdata <= 8'd0;  h_we <= 1'b0;
            regs_valid <= 1'b0;
            n_active <= 0; n_pairs <= 0; n_zero <= 0; n_small <= 0; n_jump <= 0; mono <= 0;
            curv_sum <= 0; n_triples <= 0; rep_max <= 0; rep_lag <= 0; hmax <= 0; nocc <= 0; bw_sum <= 0;
        end else begin
            h_we       <= 1'b0;
            regs_valid <= 1'b0;

            case (state)
            // ---------------------------------------------------------- 대기
            S_IDLE: begin
                if (start) begin
                    c_bank <= s_bank;
                    r_active <= 0; r_pairs <= 0; r_zero <= 0; r_small <= 0; r_jump <= 0;
                    r_triples <= 0; r_mono <= 0; r_curv <= 0; r_bw <= 0;
                    p1 <= 0; p2 <= 0; a1 <= 1'b0; a2 <= 1'b0;
                    idx  <= 9'd0;
                    step <= 3'd0;
                    state <= S_PASS;
                end
            end

            // ---------------------------------------------------------- 프레임마다 6클럭
            S_PASS: begin
                case (step)
                3'd0: begin raddr <= {c_bank, idx[7:0]}; step <= 3'd1; end
                3'd1: begin step <= 3'd2; end
                3'd2: begin                                         // 기록 도착
                    cur_p   <= rd_p;
                    cur_c   <= rd_c;
                    cur_act <= rd_act;
                    pa[idx[7:0]] <= rd_p;
                    aa[idx[7:0]] <= rd_act;
                    h_raddr <= rd_p;
                    step <= 3'd3;
                end
                3'd3: begin step <= 3'd4; end
                3'd4: begin                                         // 히스토그램 값 도착
                    if (cur_act) begin
                        h_we     <= 1'b1;
                        h_waddr  <= cur_p;
                        h_wdata  <= h_rdata + 8'd1;
                        r_active <= r_active + 8'd1;
                        r_bw     <= r_bw + {7'd0, cur_c};
                    end
                    if (pair) begin
                        r_pairs <= r_pairs + 8'd1;
                        if (ad == 10'd0)       r_zero  <= r_zero + 8'd1;
                        if (is_small) begin
                            r_small <= r_small + 8'd1;
                            r_mono  <= d[9] ? r_mono - 10'sd1 : r_mono + 10'sd1;
                        end
                        if (ad > SMALL_STEP)   r_jump  <= r_jump + 8'd1;
                    end
                    if (tri3) begin
                        r_triples <= r_triples + 8'd1;
                        r_curv    <= r_curv + {7'd0, add};
                    end
                    p2 <= p1;  a2 <= a1;
                    p1 <= cur_p;  a1 <= cur_act;
                    step <= 3'd5;
                end
                3'd5: begin
                    if (idx + 9'd1 >= N_FR) begin
                        idx   <= 9'd0;
                        r_hmax <= 8'd0;
                        r_nocc <= 9'd0;
                        step  <= 3'd0;
                        state <= S_SCANH;
                    end else begin
                        idx  <= idx + 9'd1;
                        step <= 3'd0;
                    end
                end
                default: step <= 3'd0;
                endcase
            end

            // ---------------------------------------------------------- 칸마다 3클럭
            S_SCANH: begin
                case (step)
                3'd0: begin h_raddr <= idx[7:0]; step <= 3'd1; end
                3'd1: begin step <= 3'd2; end
                3'd2: begin
                    if (h_rdata > r_hmax)  r_hmax <= h_rdata;
                    if (h_rdata >= 8'd2)   r_nocc <= r_nocc + 9'd1;
                    h_we    <= 1'b1;                                // 다음 레코드를 위해 0으로
                    h_waddr <= idx[7:0];
                    h_wdata <= 8'd0;
                    if (idx == 9'd255) begin
                        lag      <= REP_MIN;
                        rk       <= REP_MIN;
                        rcnt     <= 8'd0;
                        best     <= 8'd0;
                        best_lag <= 8'd0;
                        state    <= S_REP;
                    end else begin
                        idx  <= idx + 9'd1;
                        step <= 3'd0;
                    end
                end
                default: step <= 3'd0;
                endcase
            end

            // ---------------------------------------------------------- 비교마다 1클럭
            S_REP: begin
                if (rk == N_FR - 1) begin                           // 이 지연의 마지막 비교
                    if (rcnt_n > best) begin
                        best     <= rcnt_n;
                        best_lag <= lag;
                    end
                    if (lag == REP_MAX) begin
                        state <= S_DONE;
                    end else begin
                        lag  <= lag + 8'd1;
                        rk   <= lag + 8'd1;
                        rcnt <= 8'd0;
                    end
                end else begin
                    rcnt <= rcnt_n;
                    rk   <= rk + 8'd1;
                end
            end

            // ---------------------------------------------------------- 결과
            S_DONE: begin
                n_active   <= {24'd0, r_active};
                n_pairs    <= {24'd0, r_pairs};
                n_zero     <= {24'd0, r_zero};
                n_small    <= {24'd0, r_small};
                n_jump     <= {24'd0, r_jump};
                mono       <= r_mono[9] ? {22'd0, -r_mono} : {22'd0, r_mono};
                curv_sum   <= {14'd0, r_curv};
                n_triples  <= {24'd0, r_triples};
                rep_max    <= {24'd0, best};
                rep_lag    <= {24'd0, best_lag};
                hmax       <= {24'd0, r_hmax};
                nocc       <= {23'd0, r_nocc};
                bw_sum     <= {16'd0, r_bw};
                regs_valid <= 1'b1;
                state      <= S_IDLE;
            end

            default: state <= S_IDLE;
            endcase
        end
    end

endmodule
