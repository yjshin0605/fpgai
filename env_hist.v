`timescale 1ns / 1ps
// ============================================================================
// env_hist : 전처리 포락선(17비트)으로 32칸 히스토그램을 만들고 분포 레지스터 4개를 계산한다.
//            (hist_features.py 와 비트 단위로 같음)
//
//   칸 번호  b = min(env >> HIST_SHIFT, 31)
//   n_modes  : h[i] >= OCC_THR 이고 h[i] > h[i-1], h[i] >= h[i+1] 인 칸 수 (양끝 밖은 0)
//   occ      : h[i] >= OCC_THR 인 칸 수
//   mean_q8  : S1 >> (LOG2_N - 8)                    S1 = 칸 번호의 합
//   var_q8   : (N*S2 - S1*S1) >> (2*LOG2_N - 8)     S2 = 칸 번호 제곱의 합, N = 샘플 수
//
// 샘플이 들어올 때마다 칸 카운터 +1, S1 += b, S2 += b*b (b <= 31 이라 작은 곱셈).
// N_REC 번째 샘플에서 결과를 스냅샷으로 옮기고 카운터를 0으로 → 다음 레코드를 바로 받을 수 있다.
// 곱셈은 마지막에 2번 (N*S2, S1*S1).
// ============================================================================
module env_hist #(
    parameter ENV_W      = 17,
    parameter N_REC      = 16384,
    parameter LOG2_N     = 14,
    parameter HIST_SHIFT = 10,
    parameter OCC_THR    = 256
)(
    input  wire             clk,
    input  wire             rstn,

    input  wire [ENV_W-1:0] e_tdata,            // 포락선 한 샘플
    input  wire             e_tvalid,           // e_tdata 가 유효함
    output wire             e_tready,           // 항상 1

    output reg              regs_valid,         // 아래 4개가 새로 나옴 (1클럭 펄스)
    output reg  [31:0]      n_modes, occ, mean_q8, var_q8
);

    assign e_tready = 1'b1;

    localparam CW = LOG2_N + 1;                 // 칸 카운터 비트 (최대 N_REC)

    // ------------------------------------------------------------------
    // 1. 샘플 받기
    // ------------------------------------------------------------------
    wire [ENV_W-HIST_SHIFT-1:0] b_raw = e_tdata >> HIST_SHIFT;
    wire [4:0]  b    = (b_raw > 31) ? 5'd31 : b_raw[4:0];
    wire [9:0]  bsq  = b * b;

    reg  [CW-1:0]   h  [0:31];                  // 지금 레코드 카운터
    reg  [CW-1:0]   hs [0:31];                  // 끝난 레코드 스냅샷 (검증 때 hist.mem 과 비교)
    reg  [CW-1:0]   n;
    reg  [CW+4:0]   s1;                         // 최대 16384*31
    reg  [CW+9:0]   s2;                         // 최대 16384*961

    reg             done;                       // 스냅샷 완료 → 계산 시작
    reg  [CW-1:0]   n_s;
    reg  [CW+4:0]   s1_s;
    reg  [CW+9:0]   s2_s;

    integer i;
    always @(posedge clk) begin
        if (!rstn) begin
            for (i = 0; i < 32; i = i + 1) h[i] <= {CW{1'b0}};
            n <= 0;  s1 <= 0;  s2 <= 0;  done <= 1'b0;
        end else begin
            done <= 1'b0;
            if (e_tvalid) begin
                if (n == N_REC - 1) begin                       // 레코드 마지막 샘플
                    for (i = 0; i < 32; i = i + 1) begin
                        hs[i] <= h[i] + ((b == i) ? 1'b1 : 1'b0);
                        h[i]  <= {CW{1'b0}};
                    end
                    n_s  <= n + 1'b1;
                    s1_s <= s1 + b;
                    s2_s <= s2 + bsq;
                    n  <= 0;  s1 <= 0;  s2 <= 0;
                    done <= 1'b1;
                end else begin
                    h[b] <= h[b] + 1'b1;
                    n  <= n + 1'b1;
                    s1 <= s1 + b;
                    s2 <= s2 + bsq;
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // 2. 스냅샷에서 분포 특징 계산 (몇 클럭)
    // ------------------------------------------------------------------
    reg [5:0] c_modes, c_occ;
    integer j;
    always @(*) begin
        c_modes = 6'd0;
        c_occ   = 6'd0;
        for (j = 0; j < 32; j = j + 1) begin
            if (hs[j] >= OCC_THR) begin
                c_occ = c_occ + 6'd1;
                if (hs[j] > ((j == 0)  ? {CW{1'b0}} : hs[j-1]) &&
                    hs[j] >= ((j == 31) ? {CW{1'b0}} : hs[j+1]))
                    c_modes = c_modes + 6'd1;
            end
        end
    end

    reg  [1:0]          st;
    reg  [2*CW+10:0]    prod_ns2, prod_s1s1;    // 곱셈 결과 (파이프라인 1단)

    always @(posedge clk) begin
        if (!rstn) begin
            st <= 2'd0;  regs_valid <= 1'b0;
            n_modes <= 0;  occ <= 0;  mean_q8 <= 0;  var_q8 <= 0;
        end else begin
            regs_valid <= 1'b0;
            case (st)
            2'd0: if (done) st <= 2'd1;
            2'd1: begin
                prod_ns2  <= n_s  * s2_s;
                prod_s1s1 <= s1_s * s1_s;
                st <= 2'd2;
            end
            2'd2: begin
                n_modes    <= {26'd0, c_modes};
                occ        <= {26'd0, c_occ};
                mean_q8    <= s1_s >> (LOG2_N - 8);
                var_q8     <= (prod_ns2 - prod_s1s1) >> (2 * LOG2_N - 8);
                regs_valid <= 1'b1;
                st <= 2'd0;
            end
            default: st <= 2'd0;
            endcase
        end
    end

endmodule
