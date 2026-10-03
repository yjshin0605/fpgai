`timescale 1ns / 1ps
// ============================================================================
// env_detect : I/Q 샘플에서 포락선(그 순간 신호가 얼마나 센지)을 뽑는다.
//              env_hist 의 입력을 만드는 모듈이며, FFT 갈래와는 무관하다.
//
//   env = max(|I|,|Q|) + 3/8 × min(|I|,|Q|)
//
// 제곱근 대신 쓰는 근사식이다. 참값 sqrt(I²+Q²) 과 최대 약 4% 차이가 나지만,
// 뒤에서 하는 일이 "세기를 32칸으로 나눠 세기" 라서 이 정도 오차는 칸을 바꾸지 않는다.
// 곱셈이 없어 DSP 를 쓰지 않는다.
//
// 3/8 은 (min >> 2) + (min >> 3) 로 계산한다. 각각 따로 버린다.
// 골든 pre_x.mem / pre_env.mem 144개 중 20개로 확인한 결과 이 방식이 일치했다
// (env_check.py). FRAC_MODE = 1 은 (3*min)>>3 방식으로, 지금은 쓰지 않는다.
//
// 입력을 어디서 받나
//   DC 제거 직후(정규화 전) 값을 받는다. 앞단 정규화는 프레임마다 다른 이득을 곱하므로,
//   그 뒤에서 포락선을 뽑으면 켜짐·꺼짐 차이가 사라져 분포 특징이 무의미해진다.
//
// 배율 맞추기 (norm_sh)
//   히스토그램은 칸 번호를 (포락선 >> 10) 으로 정하므로, 포락선의 절대 크기가 칸을 좌우한다.
//   약한 신호(예: dc_remove 후 최대 122)는 그대로 넣으면 16,384샘플이 전부 0번 칸에 몰린다.
//   그래서 레코드마다 한 번 배율을 맞춘다. 레코드 안에서는 같은 배율이므로
//   켜짐·꺼짐 차이와 굴곡은 그대로 보존된다.
//
//   norm_sh = 14 - (레코드 최대 절댓값의 비트수 - 1)
//     최대 122   (7비트)  → norm_sh = +8  → 256배
//     최대 3,000 (12비트) → norm_sh = +3  → 8배
//     최대 17,000(15비트) → norm_sh =  0  → 1배
//     최대 90,000(17비트) → norm_sh = -2  → 1/4배
//   PS 가 DMA 설정 전에 DDR 의 레코드를 훑어 계산하고 레지스터에 쓴다 (약 0.1 ms).
//
// 입력  : dc_remove 출력 (I, Q 각각 18비트 부호 있는 정수)
// 출력  : e_tdata 17비트 부호 없는 정수. env_hist 의 e_tdata 에 바로 연결
// ============================================================================
module env_detect #(
    parameter IN_W      = 18,       // 입력 I, Q 비트 수 (dc_remove 출력)
    parameter OUT_W     = 17,       // 출력 포락선 비트 수 (골든 ENV_BITS)
    parameter FRAC_MODE = 0,        // 0 = (min>>2)+(min>>3) — 골든과 일치 확인됨
    parameter SH_W      = 5         // norm_sh 비트 수
)(
    input  wire                     clk,
    input  wire                     rstn,          // 0 이면 초기화

    input  wire signed [IN_W-1:0]   s_i,           // DC 제거된 I 샘플 (정규화 전)
    input  wire signed [IN_W-1:0]   s_q,           // DC 제거된 Q 샘플 (정규화 전)
    input  wire                     s_valid,       // s_i, s_q 가 유효하다는 표시
    output wire                     s_ready,       // 항상 1 (멈추지 않음)

    input  wire signed [SH_W-1:0]   norm_sh,       // 레코드 배율. 양수면 왼쪽, 음수면 오른쪽으로 민다
                                                   // 레코드가 시작되기 전에 고정해 둘 것

    output reg  [OUT_W-1:0]         e_tdata,       // 포락선 한 샘플
    output reg                      e_tvalid       // e_tdata 가 유효하다는 표시
);

    assign s_ready = 1'b1;

    localparam RAW_W = IN_W + 2;                   // 근사식 결과 폭 (max + 3/8 min < 1.375 × 최대)

    // ------------------------------------------------------------------
    // 1단: 절댓값 구하기
    // ------------------------------------------------------------------
    reg [IN_W-1:0] a1, b1;
    reg            v1;

    wire [IN_W-1:0] abs_i = s_i[IN_W-1] ? (~s_i + 1'b1) : s_i;
    wire [IN_W-1:0] abs_q = s_q[IN_W-1] ? (~s_q + 1'b1) : s_q;

    always @(posedge clk) begin
        if (!rstn) begin
            a1 <= {IN_W{1'b0}};  b1 <= {IN_W{1'b0}};  v1 <= 1'b0;
        end else begin
            a1 <= abs_i;
            b1 <= abs_q;
            v1 <= s_valid;
        end
    end

    // ------------------------------------------------------------------
    // 2단: 큰 쪽과 작은 쪽 나누기
    // ------------------------------------------------------------------
    reg [IN_W-1:0] mx2, mn2;
    reg            v2;

    always @(posedge clk) begin
        if (!rstn) begin
            mx2 <= {IN_W{1'b0}};  mn2 <= {IN_W{1'b0}};  v2 <= 1'b0;
        end else begin
            mx2 <= (a1 >= b1) ? a1 : b1;
            mn2 <= (a1 >= b1) ? b1 : a1;
            v2  <= v1;
        end
    end

    // ------------------------------------------------------------------
    // 3단: env = max + 3/8 × min
    // ------------------------------------------------------------------
    reg [RAW_W-1:0] raw3;
    reg             v3;

    wire [RAW_W-1:0] frac = (FRAC_MODE == 0)
                          ? ({2'b00, mn2} >> 2) + ({2'b00, mn2} >> 3)   // 각각 버림
                          : (({2'b00, mn2} * 3) >> 3);                  // 한 번에 버림

    always @(posedge clk) begin
        if (!rstn) begin
            raw3 <= {RAW_W{1'b0}};  v3 <= 1'b0;
        end else begin
            raw3 <= {2'b00, mx2} + frac;
            v3   <= v2;
        end
    end

    // ------------------------------------------------------------------
    // 4단: 배율 맞추고 OUT_W 비트로 포화
    // ------------------------------------------------------------------
    localparam SHIFT_W = RAW_W + (1 << (SH_W-1));   // 왼쪽으로 밀 수 있는 최대까지 확보
    wire [SHIFT_W-1:0] wide = {{(SHIFT_W-RAW_W){1'b0}}, raw3};
    wire [SHIFT_W-1:0] shifted = norm_sh[SH_W-1] ? (wide >> (-norm_sh)) : (wide << norm_sh);
    wire               ovf     = |shifted[SHIFT_W-1:OUT_W];

    always @(posedge clk) begin
        if (!rstn) begin
            e_tdata <= {OUT_W{1'b0}};  e_tvalid <= 1'b0;
        end else begin
            e_tdata  <= ovf ? {OUT_W{1'b1}} : shifted[OUT_W-1:0];
            e_tvalid <= v3;
        end
    end

endmodule
