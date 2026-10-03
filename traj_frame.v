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
// 입력 약속 (CORDIC 크기 스트림)
//   - 한 프레임 = 256칸, fftshift 순서 (도착 순서 = 칸 번호 0..255)
//   - 칸 번호는 모듈 안에서 센다. 256번째 칸에서 프레임이 끝난다. tlast 는 쓰지 않는다.
//   - tready 는 항상 1. tvalid 는 중간에 쉬어도 됨
//
// 순간 대역폭은 피크를 알아야 셀 수 있으므로, 프레임을 버퍼에 저장했다가 끝난 뒤 다시 읽는다.
// 짝수 칸·홀수 칸 버퍼를 나눠 한 클럭에 2칸씩 읽는다 → 약 130클럭 (다음 프레임 256클럭 안에 끝남).
// 버퍼는 은행 2개(핑퐁)라 다시 읽는 동안 다음 프레임을 받을 수 있다.
// ============================================================================
module traj_frame #(
    parameter MAG_W    = 16,
    parameter BW_SHIFT = 2
)(
    input  wire             clk,
    input  wire             rstn,               // 0 이면 초기화

    input  wire [MAG_W-1:0] s_tdata,            // 한 칸의 크기 (CORDIC 출력)
    input  wire             s_tvalid,           // tdata 가 유효함
    output wire             s_tready,           // 항상 1

    output reg              fr_valid,           // 아래 값이 준비됨 (1클럭 펄스)
    output reg  [7:0]       fr_p,               // 피크 칸 번호
    output reg  [MAG_W-1:0] fr_m,               // 피크 크기
    output reg  [8:0]       fr_c                // 순간 대역폭 칸 수 (0..256)
);

    assign s_tready = 1'b1;

    // ------------------------------------------------------------------
    // 1. 칸 받기: 버퍼 저장 + 피크 추적
    // ------------------------------------------------------------------
    reg [7:0]       k;                          // 지금 칸 번호
    reg             wbank;
    reg [MAG_W-1:0] buf_e [0:255];              // 짝수 칸: 주소 {은행, k[7:1]}
    reg [MAG_W-1:0] buf_o [0:255];              // 홀수 칸
    reg [MAG_W-1:0] pk_m;
    reg [7:0]       pk_p;

    wire new_pk = (k == 8'd0) || (s_tdata > pk_m);

    always @(posedge clk) begin
        if (s_tvalid && !k[0]) buf_e[{wbank, k[7:1]}] <= s_tdata;
        if (s_tvalid &&  k[0]) buf_o[{wbank, k[7:1]}] <= s_tdata;
    end

    reg             sc_start;
    reg             sc_bank;
    reg [7:0]       sc_p;
    reg [MAG_W-1:0] sc_m;

    always @(posedge clk) begin
        if (!rstn) begin
            k <= 8'd0;  wbank <= 1'b0;  pk_m <= {MAG_W{1'b0}};  pk_p <= 8'd0;
            sc_start <= 1'b0;  sc_bank <= 1'b0;  sc_p <= 8'd0;  sc_m <= {MAG_W{1'b0}};
        end else begin
            sc_start <= 1'b0;
            if (s_tvalid) begin
                if (new_pk) begin
                    pk_m <= s_tdata;
                    pk_p <= k;
                end
                k <= k + 8'd1;                          // 255 다음은 자동으로 0
                if (k == 8'd255) begin
                    sc_start <= 1'b1;
                    sc_bank  <= wbank;
                    sc_m     <= new_pk ? s_tdata : pk_m;
                    sc_p     <= new_pk ? k : pk_p;
                    wbank    <= ~wbank;
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
        end else begin
            fr_valid <= 1'b0;
            if (sc_start) begin
                busy   <= 1'b1;
                si     <= 8'd0;
                rd_v   <= 1'b0;
                cnt    <= 9'd0;
                b_bank <= sc_bank;
                b_p    <= sc_p;
                b_m    <= sc_m;
                thr    <= sc_m >> BW_SHIFT;
            end else if (busy) begin
                cnt <= cnt + {7'd0, add2};
                if (si < 8'd128) begin
                    rd_v <= 1'b1;
                    si   <= si + 8'd1;
                end else begin
                    busy     <= 1'b0;
                    rd_v     <= 1'b0;
                    fr_valid <= 1'b1;
                    fr_p     <= b_p;
                    fr_m     <= b_m;
                    fr_c     <= cnt + {7'd0, add2};
                end
            end
        end
    end

endmodule
