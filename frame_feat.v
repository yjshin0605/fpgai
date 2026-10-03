`timescale 1ns / 1ps
// ============================================================================
// frame_feat : col_feat 가 내보내는 줄 특징을 레코드(255줄) 단위로 모아 특징을 계산한다.
//
// 1) 줄 기록  : 줄마다 {퍼짐, p, E} 를 기록 메모리에 쓰고 E 최대·최소를 갱신한다.
//               기록 메모리는 2개 은행(핑퐁)이라, 계산 중에도 다음 레코드를 받을 수 있다.
// 2) PASS     : 레코드가 끝나면(col_rec_last) 기록을 처음부터 다시 읽으며 켜진 줄만 골라
//               켜진 줄 수, E 합, 퍼짐 합, 16줄 전과의 피크 칸 비교, 피크 칸 히스토그램을 만든다.
//               켜짐 판정: 2*E > (emax + emin)   (나눗셈·반올림 없이 기준값 (emax+emin)/2 와 비교)
// 3) SCAN     : 히스토그램 0..255 칸을 누적하며 10%·90% 위치를 찾고, 읽은 칸은 0으로 지운다.
//               p10 = 누적*10 >= n_on   인 첫 칸,  p90 = 누적*10 >= n_on*9 인 첫 칸
// 4) DONE     : 결과를 출력 레지스터에 옮기고 feat_valid 를 1클럭 올린다.
//
// 처리 시간: 줄마다 6클럭 + 칸마다 3클럭 = 255*6 + 256*3 ≈ 2,300클럭.
// 한 레코드는 실제로 수만 클럭에 걸쳐 들어오므로, 다음 레코드가 끝나기 전에 계산이 끝난다.
// (레코드 하나가 2,300클럭보다 빨리 끝나면 그 레코드는 계산되지 않는다. busy 로 확인 가능)
// ============================================================================
module frame_feat #(
    parameter E_W = 40,                 // col_feat 의 E, 퍼짐 비트 수
    parameter LAG = 16                  // 정체 비교 간격 [줄]. 50% 겹침에서 16줄 = 약 82 µs
)(
    input  wire             clk,
    input  wire             rstn,               // 0 이면 초기화

    // col_feat 출력을 그대로 연결
    input  wire             col_valid,          // 줄 특징이 들어옴 (1클럭 펄스)
    input  wire [E_W-1:0]   col_e,              // 줄 에너지 E
    input  wire [7:0]       col_p,              // 피크 칸 번호 (정렬됨)
    input  wire [E_W-1:0]   col_spread,         // 퍼짐 에너지
    input  wire             col_rec_last,       // 레코드 마지막 줄

    output reg              feat_valid,         // 아래 결과가 새로 나옴 (1클럭 펄스)
    output reg  [E_W-1:0]   f_emax,
    output reg  [E_W-1:0]   f_emin,
    output reg  [8:0]       f_ncols,            // 받은 줄 수
    output reg  [8:0]       f_non,              // 켜진 줄 수
    output reg  [E_W+7:0]   f_esum,             // 켜진 줄 E 합
    output reg  [E_W+7:0]   f_ssum,             // 켜진 줄 퍼짐 합
    output reg  [8:0]       f_npair,            // 켜진 줄 중 LAG 줄 전도 켜진 줄 수
    output reg  [8:0]       f_nstay,            // 그중 피크 칸이 같은 줄 수
    output reg  [7:0]       f_p10,              // 히스토그램 10% 위치
    output reg  [7:0]       f_p90,              // 히스토그램 90% 위치
    output wire             busy                // 계산 중
);

    localparam RW = 2 * E_W + 8;        // 기록 한 칸 = {퍼짐, p, E}

    // ==================================================================
    // 1. 줄 기록 (쓰기 쪽)
    // ==================================================================
    reg [RW-1:0]  rec_mem [0:511];      // 주소 = {은행 1비트, 줄 번호 8비트}
    reg           wbank;
    reg [7:0]     wcnt;
    reg [E_W-1:0] emax_w, emin_w;

    wire [E_W-1:0] emax_n = (wcnt == 8'd0 || col_e > emax_w) ? col_e : emax_w;
    wire [E_W-1:0] emin_n = (wcnt == 8'd0 || col_e < emin_w) ? col_e : emin_w;

    reg           start;                // 계산 시작 요청 (1클럭)
    reg           s_bank;
    reg [8:0]     s_ncols;
    reg [E_W-1:0] s_emax, s_emin;

    always @(posedge clk) begin
        if (col_valid)
            rec_mem[{wbank, wcnt}] <= {col_spread, col_p, col_e};
    end

    always @(posedge clk) begin
        if (!rstn) begin
            wbank   <= 1'b0;
            wcnt    <= 8'd0;
            emax_w  <= {E_W{1'b0}};
            emin_w  <= {E_W{1'b0}};
            start   <= 1'b0;
            s_bank  <= 1'b0;
            s_ncols <= 9'd0;
            s_emax  <= {E_W{1'b0}};
            s_emin  <= {E_W{1'b0}};
        end else begin
            start <= 1'b0;
            if (col_valid) begin
                emax_w <= emax_n;
                emin_w <= emin_n;
                wcnt   <= wcnt + 8'd1;
                if (col_rec_last) begin
                    start   <= 1'b1;
                    s_bank  <= wbank;
                    s_ncols <= {1'b0, wcnt} + 9'd1;
                    s_emax  <= emax_n;
                    s_emin  <= emin_n;
                    wbank   <= ~wbank;
                    wcnt    <= 8'd0;
                end
            end
        end
    end

    // ==================================================================
    // 2. 메모리 읽기 포트와 히스토그램
    // ==================================================================
    reg  [8:0]    rec_raddr;
    reg  [RW-1:0] rec_rdata;
    always @(posedge clk)
        rec_rdata <= rec_mem[rec_raddr];

    reg  [8:0]    hist [0:255];
    reg  [7:0]    hist_raddr;
    reg  [8:0]    hist_rdata;
    reg           hist_we;
    reg  [7:0]    hist_waddr;
    reg  [8:0]    hist_wdata;

    integer k;
    initial begin                        // BRAM 초기값 0 (합성 시 초기화 값으로 들어감)
        for (k = 0; k < 256; k = k + 1) hist[k] = 9'd0;
    end

    always @(posedge clk) begin
        if (hist_we)
            hist[hist_waddr] <= hist_wdata;
        hist_rdata <= hist[hist_raddr];
    end

    // ==================================================================
    // 3. 계산 FSM
    // ==================================================================
    localparam S_IDLE = 2'd0, S_PASS = 2'd1, S_SCAN = 2'd2, S_DONE = 2'd3;
    reg [1:0]       state;
    reg [2:0]       step;
    reg [8:0]       idx;                // PASS: 줄 번호, SCAN: 칸 번호

    reg             c_bank;
    reg [8:0]       c_ncols;
    reg [E_W-1:0]   c_emax, c_emin;
    reg [E_W:0]     c_thr2;             // emax + emin

    reg [E_W-1:0]   cur_e, cur_sp;
    reg [7:0]       cur_p;
    reg             cur_on;

    reg [8:0]       a_non, a_npair, a_nstay;
    reg [E_W+7:0]   a_esum, a_ssum;
    reg [8*LAG-1:0] lag_p;              // 최근 LAG 줄의 피크 칸 (맨 위가 가장 오래됨)
    reg [LAG-1:0]   lag_on;             // 최근 LAG 줄의 켜짐 여부

    reg [8:0]       cum;
    reg             f10, f90;
    reg [7:0]       r_p10, r_p90;

    wire [E_W-1:0]  rd_e  = rec_rdata[E_W-1:0];
    wire [7:0]      rd_p  = rec_rdata[E_W+7:E_W];
    wire [E_W-1:0]  rd_sp = rec_rdata[2*E_W+7:E_W+8];
    wire            rd_on = ({1'b0, rd_e} << 1) > {1'b0, c_thr2};

    wire [8:0]      cum_n   = cum + hist_rdata;
    wire [12:0]     cum10   = ({4'd0, cum_n} << 3) + ({4'd0, cum_n} << 1);
    wire [12:0]     non1    = {4'd0, a_non};
    wire [12:0]     non9    = ({4'd0, a_non} << 3) + {4'd0, a_non};

    wire [7:0]      p_old   = lag_p[8*LAG-1 -: 8];
    wire            on_old  = lag_on[LAG-1];

    assign busy = (state != S_IDLE);

    always @(posedge clk) begin
        if (!rstn) begin
            state      <= S_IDLE;
            step       <= 3'd0;
            idx        <= 9'd0;
            rec_raddr  <= 9'd0;
            hist_raddr <= 8'd0;
            hist_we    <= 1'b0;
            hist_waddr <= 8'd0;
            hist_wdata <= 9'd0;
            feat_valid <= 1'b0;
            f_emax <= {E_W{1'b0}};  f_emin <= {E_W{1'b0}};
            f_ncols <= 9'd0;  f_non <= 9'd0;  f_npair <= 9'd0;  f_nstay <= 9'd0;
            f_esum <= {(E_W+8){1'b0}};  f_ssum <= {(E_W+8){1'b0}};
            f_p10 <= 8'd0;  f_p90 <= 8'd0;
        end else begin
            hist_we    <= 1'b0;
            feat_valid <= 1'b0;

            case (state)
            // ---------------------------------------------------------- 대기
            S_IDLE: begin
                if (start) begin
                    c_bank  <= s_bank;
                    c_ncols <= s_ncols;
                    c_emax  <= s_emax;
                    c_emin  <= s_emin;
                    c_thr2  <= {1'b0, s_emax} + {1'b0, s_emin};
                    a_non   <= 9'd0;  a_npair <= 9'd0;  a_nstay <= 9'd0;
                    a_esum  <= {(E_W+8){1'b0}};  a_ssum <= {(E_W+8){1'b0}};
                    lag_p   <= {(8*LAG){1'b0}};
                    lag_on  <= {LAG{1'b0}};
                    idx     <= 9'd0;
                    step    <= 3'd0;
                    state   <= S_PASS;
                end
            end

            // ---------------------------------------------------------- 줄마다 6클럭
            S_PASS: begin
                case (step)
                3'd0: begin rec_raddr <= {c_bank, idx[7:0]}; step <= 3'd1; end
                3'd1: begin step <= 3'd2; end                      // 기록 메모리 읽는 중
                3'd2: begin                                        // 기록 도착
                    cur_e      <= rd_e;
                    cur_p      <= rd_p;
                    cur_sp     <= rd_sp;
                    cur_on     <= rd_on;
                    hist_raddr <= rd_p;
                    step       <= 3'd3;
                end
                3'd3: begin step <= 3'd4; end                      // 히스토그램 읽는 중
                3'd4: begin                                        // 히스토그램 값 도착
                    if (cur_on) begin
                        hist_we    <= 1'b1;
                        hist_waddr <= cur_p;
                        hist_wdata <= hist_rdata + 9'd1;
                        a_non      <= a_non + 9'd1;
                        a_esum     <= a_esum + {8'd0, cur_e};
                        a_ssum     <= a_ssum + {8'd0, cur_sp};
                        if (on_old) begin
                            a_npair <= a_npair + 9'd1;
                            if (cur_p == p_old)
                                a_nstay <= a_nstay + 9'd1;
                        end
                    end
                    lag_p  <= {lag_p[8*LAG-9:0], cur_p};
                    lag_on <= {lag_on[LAG-2:0], cur_on};
                    step   <= 3'd5;
                end
                3'd5: begin                                        // 히스토그램 쓰기 반영
                    if (idx + 9'd1 >= c_ncols) begin
                        idx   <= 9'd0;
                        cum   <= 9'd0;
                        f10   <= 1'b0;
                        f90   <= 1'b0;
                        r_p10 <= 8'd0;
                        r_p90 <= 8'd0;
                        step  <= 3'd0;
                        state <= S_SCAN;
                    end else begin
                        idx  <= idx + 9'd1;
                        step <= 3'd0;
                    end
                end
                default: step <= 3'd0;
                endcase
            end

            // ---------------------------------------------------------- 칸마다 3클럭
            S_SCAN: begin
                case (step)
                3'd0: begin hist_raddr <= idx[7:0]; step <= 3'd1; end
                3'd1: begin step <= 3'd2; end
                3'd2: begin
                    cum <= cum_n;
                    if (!f10 && cum10 >= non1) begin f10 <= 1'b1; r_p10 <= idx[7:0]; end
                    if (!f90 && cum10 >= non9) begin f90 <= 1'b1; r_p90 <= idx[7:0]; end
                    hist_we    <= 1'b1;                            // 읽은 칸은 0으로 (다음 레코드 준비)
                    hist_waddr <= idx[7:0];
                    hist_wdata <= 9'd0;
                    if (idx == 9'd255) begin
                        state <= S_DONE;
                    end else begin
                        idx  <= idx + 9'd1;
                        step <= 3'd0;
                    end
                end
                default: step <= 3'd0;
                endcase
            end

            // ---------------------------------------------------------- 결과 내보내기
            S_DONE: begin
                f_emax     <= c_emax;
                f_emin     <= c_emin;
                f_ncols    <= c_ncols;
                f_non      <= a_non;
                f_esum     <= a_esum;
                f_ssum     <= a_ssum;
                f_npair    <= a_npair;
                f_nstay    <= a_nstay;
                f_p10      <= r_p10;
                f_p90      <= r_p90;
                feat_valid <= 1'b1;
                state      <= S_IDLE;
            end
            endcase
        end
    end

endmodule
