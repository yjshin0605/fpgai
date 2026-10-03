`timescale 1ns / 1ps
// ============================================================================
// traj_frame : 스펙트로그램 한 프레임(256칸)에서 값 3개를 구한다.  (freq_features.py 1단계)
//
//   fr_p : 피크 칸 번호   = argmax (같은 값이면 앞 칸)
//   fr_m : 피크 크기     = max
//   fr_c : 순간 대역폭 칸 수 = 이 프레임에서 크기 >= (fr_m >> BW_SHIFT) 인 칸 수
//          → traj_rec 이 활성 프레임의 fr_c 만 더해 bw_sum 을 만든다.
//            (스펙트로그램 127 x 256 전체를 저장하지 않아도 됨)
//
// 입력 약속
//   - 크기는 cordic_mag 가 내보낸 값, 칸 번호는 FFT 꼬리표(tuser[7:0])에서 받는다.
//   - FFT 출력은 natural order (0 Hz 가 0번) 이므로, 여기서 128 을 더해 fftshift 순서로 바꾼다.
//     그래야 0 Hz 를 가로지르는 LFM 의 피크가 255 에서 0 으로 튀지 않는다.
//   - 프레임 끝은 s_tlast (FFT 의 마지막 빈, k = 255) 로 판단한다.
//   - 무효 표시(tuser[8])와 프레임 번호(tuser[16:9])는 그대로 뒤로 넘긴다.
//   - tready 는 항상 1. tvalid 는 중간에 쉬어도 됨
//
// 순간 대역폭은 피크를 알아야 셀 수 있으므로, 프레임을 버퍼에 저장했다가 끝난 뒤 다시 읽는다.
// 짝수 칸·홀수 칸 버퍼를 나눠 한 클럭에 2칸씩 읽는다 → 약 130클럭 (다음 프레임 256클럭 안에 끝남).
// 버퍼는 은행 2개(핑퐁)라 다시 읽는 동안 다음 프레임을 받을 수 있다.
// ============================================================================
module traj_frame #(
    parameter MAG_W    = 18,        // cordic_mag 출력 폭과 같아야 한다 (16으로 두면 피크가 랩함)
    parameter BW_SHIFT = 2
)(
    input  wire             clk,
    input  wire             rstn,               // 0 이면 초기화

    input  wire [MAG_W-1:0] s_tdata,            // 한 칸의 크기 (CORDIC 출력)
    input  wire [7:0]       s_bin,              // 그 칸의 번호 (tuser[7:0], natural order)
    input  wire             s_invalid,          // 이 프레임이 잡음뿐이라는 표시 (tuser[8])
    input  wire [7:0]       s_frame,            // 프레임 번호 0~126 (tuser[16:9])
    input  wire             s_tlast,            // 프레임의 마지막 칸 (k = 255)
    input  wire             s_tvalid,           // s_tdata 가 유효함
    output wire             s_tready,           // 항상 1

    output reg              fr_valid,           // 아래 값이 준비됨 (1클럭 펄스)
    output reg  [7:0]       fr_p,               // 피크 칸 번호 (fftshift 순서, 128 = 0 Hz)
    output reg  [MAG_W-1:0] fr_m,               // 피크 크기
    output reg  [8:0]       fr_c,               // 순간 대역폭 칸 수 (0..256)
    output reg              fr_invalid,         // 이 프레임의 무효 표시
    output reg  [7:0]       fr_frame            // 이 프레임의 번호
);

    assign s_tready = 1'b1;

    // ------------------------------------------------------------------
    // 1. 칸 받기: 버퍼 저장 + 피크 추적
    // ------------------------------------------------------------------
    wire [7:0]      k = s_bin + 8'd128;          // fftshift 순서로 바꾼 칸 번호
    reg [7:0]       wcnt;                        // 버퍼에 쓴 개수 (주소용)
    reg             wbank;
    reg [MAG_W-1:0] buf_e [0:255];              // 짝수 칸: 주소 {은행, k[7:1]}
    reg [MAG_W-1:0] buf_o [0:255];              // 홀수 칸
    reg [MAG_W-1:0] pk_m;
    reg [7:0]       pk_p;

    wire new_pk = (wcnt == 8'd0) || (s_tdata > pk_m) ||
                  ((s_tdata == pk_m) && (k < pk_p));     // 같은 값이면 칸 번호가 작은 쪽

    always @(posedge clk) begin
        if (s_tvalid && !k[0]) buf_e[{wbank, k[7:1]}] <= s_tdata;   // 짝수 칸
        if (s_tvalid &&  k[0]) buf_o[{wbank, k[7:1]}] <= s_tdata;   // 홀수 칸
    end

    reg             sc_start;
    reg             sc_bank;
    reg [7:0]       sc_p;
    reg [MAG_W-1:0] sc_m;
    reg             sc_invalid;
    reg [7:0]       sc_frame;

    always @(posedge clk) begin
        if (!rstn) begin
            wcnt <= 8'd0;  wbank <= 1'b0;  pk_m <= {MAG_W{1'b0}};  pk_p <= 8'd0;
            sc_start <= 1'b0;  sc_bank <= 1'b0;  sc_p <= 8'd0;  sc_m <= {MAG_W{1'b0}};
            sc_invalid <= 1'b0;  sc_frame <= 8'd0;
        end else begin
            sc_start <= 1'b0;
            if (s_tvalid) begin
                if (new_pk) begin
                    pk_m <= s_tdata;
                    pk_p <= k;
                end
                wcnt <= wcnt + 8'd1;
                if (s_tlast) begin                      // 프레임의 마지막 칸
                    sc_start   <= 1'b1;
                    sc_bank    <= wbank;
                    sc_m       <= new_pk ? s_tdata : pk_m;
                    sc_p       <= new_pk ? k : pk_p;
                    sc_invalid <= s_invalid;
                    sc_frame   <= s_frame;
                    wbank      <= ~wbank;
                    wcnt       <= 8'd0;
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // 2. 프레임이 끝난 뒤: 크기 >= 피크 >> BW_SHIFT 인 칸 수 세기 (한 클럭 2칸)
    // ------------------------------------------------------------------
    reg             busy;
    reg [7:0]       si;                         // 읽는 주소 0..127, 128 이면 마무리
    reg             b_bank;
    reg [7:0]       b_p;
    reg [MAG_W-1:0] b_m, thr;
    reg             b_invalid;
    reg [7:0]       b_frame;
    reg [MAG_W-1:0] rd_e, rd_o;
    reg             rd_v;
    reg [8:0]       cnt;

    always @(posedge clk) begin
        rd_e <= buf_e[{b_bank, si[6:0]}];
        rd_o <= buf_o[{b_bank, si[6:0]}];
    end

    wire [1:0] add2 = (rd_v && rd_e >= thr ? 2'd1 : 2'd0) + (rd_v && rd_o >= thr ? 2'd1 : 2'd0);

    always @(posedge clk) begin
        if (!rstn) begin
            busy <= 1'b0;  si <= 8'd0;  rd_v <= 1'b0;  cnt <= 9'd0;
            b_bank <= 1'b0;  b_p <= 8'd0;  b_m <= {MAG_W{1'b0}};  thr <= {MAG_W{1'b0}};
            fr_valid <= 1'b0;  fr_p <= 8'd0;  fr_m <= {MAG_W{1'b0}};  fr_c <= 9'd0;
            fr_invalid <= 1'b0;  fr_frame <= 8'd0;  b_invalid <= 1'b0;  b_frame <= 8'd0;
        end else begin
            fr_valid <= 1'b0;
            if (sc_start) begin
                busy   <= 1'b1;
                si     <= 8'd0;
                rd_v   <= 1'b0;
                cnt    <= 9'd0;
                b_bank <= sc_bank;
                b_p       <= sc_p;
                b_m       <= sc_m;
                b_invalid <= sc_invalid;
                b_frame   <= sc_frame;
                thr       <= sc_m >> BW_SHIFT;
            end else if (busy) begin
                cnt <= cnt + {7'd0, add2};
                if (si < 8'd128) begin
                    rd_v <= 1'b1;
                    si   <= si + 8'd1;
                end else begin
                    busy     <= 1'b0;
                    rd_v     <= 1'b0;
                    fr_valid   <= 1'b1;
                    fr_p       <= b_p;
                    fr_m       <= b_m;
                    fr_c       <= cnt + {7'd0, add2};
                    fr_invalid <= b_invalid;
                    fr_frame   <= b_frame;
                end
            end
        end
    end

endmodule
