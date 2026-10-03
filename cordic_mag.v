`timescale 1ns / 1ps
// ============================================================================
// cordic_mag : FFT 출력의 크기를 CORDIC vectoring 방식으로 구한다.
//              (reference_python/magnitude.py 와 비트 단위로 같음)
//
//   x0 = |Re| , y0 = Im
//   16번 반복: y >= 0 이면  x += y>>>i ,  y -= x>>>i
//              y <  0 이면  x -= y>>>i ,  y += x>>>i
//              (>>> 는 산술 shift = Python 의 >> 와 같은 버림)
//   출력 = x >>> 1 을 16비트 부호 없는 수로 포화
//
// 곱셈이 없어서 DSP 를 쓰지 않는다. 16단 파이프라인이라 한 클럭에 한 칸씩 처리하고,
// 입력을 넣은 뒤 17클럭 후에 결과가 나온다 (중간에 tvalid 가 쉬어도 짝이 유지됨).
//
// 입력  : s_re, s_im  (FFT 출력의 실수부·허수부, 각각 25비트 부호 있는 정수)
//          fft_wrap 의 m_axis_tdata 에서 [24:0] 과 [56:32] 를 잘라 넣으면 된다.
// 출력  : m_tdata = 크기 18비트, traj_frame 의 s_tdata 로 바로 연결
//
// 비트 폭 근거
//   입력 최대 2^24. CORDIC 이득 1.6468 과 sqrt(2) 를 곱하면 약 3,900만 → 26비트.
//   내부를 28비트로 두어 여유 2비트를 둔다.
//   출력은 OUT_SHIFT 만큼 내려 18비트에 담는다. 실측 FFT 최대 1,138,443 기준
//   크기 최대는 약 188만이고, OUT_SHIFT = 4 면 약 117,000 으로 18비트에 들어간다.
// ============================================================================
module cordic_mag #(
    parameter IN_W     = 25,        // 입력 실수부·허수부 비트 수 (FFT 출력)
    parameter ITERS    = 16,        // CORDIC 반복 횟수
    parameter OUT_SHIFT = 4,        // 출력 전 오른쪽 shift (실측 최대에 맞춰 조정 가능)
    parameter OUT_W    = 18,        // 출력 비트 수 (부호 없음)
    parameter INT_W    = 28         // 내부 계산 비트 수 (부호 있음)
)(
    input  wire             clk,
    input  wire             rstn,           // 0 이면 초기화

    input  wire signed [IN_W-1:0] s_re,     // FFT 결과의 실수부
    input  wire signed [IN_W-1:0] s_im,     // FFT 결과의 허수부
    input  wire             s_tvalid,       // s_re, s_im 이 유효함
    output wire             s_tready,       // 항상 1

    output wire [OUT_W-1:0] m_tdata,        // 크기
    output wire             m_tvalid        // m_tdata 가 유효함
);

    assign s_tready = 1'b1;

    // 0단: x = |Re| , y = Im
    wire signed [INT_W-1:0] re_ext = {{(INT_W-IN_W){s_re[IN_W-1]}}, s_re};
    wire signed [INT_W-1:0] im_ext = {{(INT_W-IN_W){s_im[IN_W-1]}}, s_im};
    wire signed [INT_W-1:0] x0 = re_ext[INT_W-1] ? -re_ext : re_ext;
    wire signed [INT_W-1:0] y0 = im_ext;

    reg signed [INT_W-1:0] x [0:ITERS];
    reg signed [INT_W-1:0] y [0:ITERS];
    reg                    v [0:ITERS];

    integer i;
    always @(posedge clk) begin
        if (!rstn) begin
            for (i = 0; i <= ITERS; i = i + 1) begin
                x[i] <= {INT_W{1'b0}};
                y[i] <= {INT_W{1'b0}};
                v[i] <= 1'b0;
            end
        end else begin
            x[0] <= x0;
            y[0] <= y0;
            v[0] <= s_tvalid;
            for (i = 0; i < ITERS; i = i + 1) begin
                // i번째 반복: shift 량도 i
                if (y[i] >= 0) begin
                    x[i+1] <= x[i] + (y[i] >>> i);
                    y[i+1] <= y[i] - (x[i] >>> i);
                end else begin
                    x[i+1] <= x[i] - (y[i] >>> i);
                    y[i+1] <= y[i] + (x[i] >>> i);
                end
                v[i+1] <= v[i];
            end
        end
    end

    // 출력: x >>> OUT_SHIFT 를 0 ~ 2^OUT_W-1 로 포화
    wire signed [INT_W-1:0] xf    = x[ITERS] >>> OUT_SHIFT;
    wire signed [INT_W-1:0] max_v = {{(INT_W-OUT_W){1'b0}}, {OUT_W{1'b1}}};   // OUT_W=18 이면 262,143
    wire                    ovf   = (xf > max_v);

    assign m_tdata  = xf[INT_W-1]             ? {OUT_W{1'b0}} :   // 음수면 0 (실제로는 생기지 않음)
                      ovf                     ? {OUT_W{1'b1}} :   // 넘치면 최대값
                                                xf[OUT_W-1:0];
    assign m_tvalid = v[ITERS];

endmodule
