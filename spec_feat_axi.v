`timescale 1ns / 1ps
// ============================================================================
// spec_feat_axi : spec_feat + AXI4-Lite 레지스터.  블록 디자인에 넣을 최상위 모듈.
//
//   FFT 출력 스트림   ┐
//                      ├→ spec_feat → 특징 17개 ┐
//   dc_remove 출력 스트림┘                       └→ AXI-Lite 레지스터 → PS 가 읽음
//
// 레지스터가 새로 나오면 그림자 레지스터(shadow)에 한 번에 복사하고 STATUS 의 done 을 1로
// 올린다. PS 는 done 이 1인지 보고 17개를 읽으면, 읽는 도중에 값이 바뀌는 일이 없다.
//
// 주소 지도 (바이트 주소, 전부 32비트 읽기)
//   0x00 STATUS   bit0 = done (새 결과 있음). PS 가 이 비트에 1을 쓰면 0으로 지워짐
//                 bit1 = busy (계산 중)
//   0x04 COUNT    지금까지 끝낸 레코드 수 (전원 켠 뒤 누적)
//   0x50 NORM_SH  포락선 배율 (부호 있는 5비트). PS 가 레코드 시작 전에 써 둔다
//                 = 14 - (그 레코드 최대 절댓값의 비트수 - 1)
//                 읽기도 되며, 리셋하면 0 이 된다
//   0x54 SKIP_FR  앞 몇 프레임을 궤적 계산에서 뺄지. 기본 0 (전부 사용).
//                 DC 수렴 구간을 빼고 비교해 보려면 24 등을 쓴다
//   0x08 ~ 0x4C   특징 레지스터 0 ~ 16 (sim_fixed.REGS 순서)
//                 0x08 n_active  0x0C n_pairs  0x10 n_zero   0x14 n_small  0x18 n_jump
//                 0x1C mono      0x20 curv_sum 0x24 n_triples 0x28 rep_max  0x2C rep_lag
//                 0x30 hmax      0x34 nocc     0x38 bw_sum   0x3C n_modes  0x40 occ
//                 0x44 mean_q8   0x48 var_q8
//   그 밖의 주소는 0을 돌려준다.
//
// PS 쪽 읽는 순서 (C 예시)
//   while (!(Xil_In32(BASE + 0x00) & 1)) ;          // done 기다리기
//   for (i = 0; i < 17; i++) r[i] = Xil_In32(BASE + 0x08 + 4*i);
//   Xil_Out32(BASE + 0x00, 1);                      // done 지우기
// ============================================================================
module spec_feat_axi #(
    parameter ADDR_W = 8                // AXI-Lite 주소 비트 수 (256바이트 공간)
)(
    // ---------------- 신호 처리 스트림 (PL 안에서 FFT·전처리와 연결) ----------------
    input  wire              clk,           // PL 클럭 (AXI 클럭과 같은 것을 쓰는 것을 권장)
    input  wire              rstn,          // 0 이면 신호 처리부 초기화

    input  wire [63:0]       fft_tdata,     // fft_wrap 출력. [24:0] 실수부, [56:32] 허수부
    input  wire [16:0]       fft_tuser,     // [7:0] 빈 번호, [8] 무효 표시, [16:9] 프레임 번호
    input  wire              fft_tlast,     // 프레임의 마지막 빈
    input  wire              fft_tvalid,    // fft_tdata 가 유효함
    output wire              fft_tready,    // 항상 1

    input  wire [35:0]       dc_tdata,      // dc_remove 출력 {Q[17:0], I[17:0]} (정규화 전)
    input  wire              dc_tvalid,     // dc_tdata 가 유효함
    output wire              dc_tready,     // 항상 1

    output wire              irq,           // done 과 같음. PS 인터럽트로 쓰고 싶을 때 연결

    // ---------------- AXI4-Lite (PS 가 읽는 쪽) ----------------
    input  wire              s_axi_aclk,    // AXI 클럭
    input  wire              s_axi_aresetn, // 0 이면 AXI 초기화

    input  wire [ADDR_W-1:0] s_axi_awaddr,  // 쓰기 주소
    input  wire              s_axi_awvalid, // 쓰기 주소가 유효함
    output reg               s_axi_awready, // 쓰기 주소를 받을 준비됨

    input  wire [31:0]       s_axi_wdata,   // 쓰는 값
    input  wire [3:0]        s_axi_wstrb,   // 바이트별 쓰기 허용 (이 모듈은 쓰이지 않음)
    input  wire              s_axi_wvalid,  // wdata 가 유효함
    output reg               s_axi_wready,  // 값을 받을 준비됨

    output reg  [1:0]        s_axi_bresp,   // 쓰기 응답 (항상 정상 00)
    output reg               s_axi_bvalid,  // 쓰기 응답이 유효함
    input  wire              s_axi_bready,  // PS 가 응답을 받을 준비됨

    input  wire [ADDR_W-1:0] s_axi_araddr,  // 읽기 주소
    input  wire              s_axi_arvalid, // 읽기 주소가 유효함
    output reg               s_axi_arready, // 읽기 주소를 받을 준비됨

    output reg  [31:0]       s_axi_rdata,   // 읽은 값
    output reg  [1:0]        s_axi_rresp,   // 읽기 응답 (항상 정상 00)
    output reg               s_axi_rvalid,  // rdata 가 유효함
    input  wire              s_axi_rready   // PS 가 값을 받을 준비됨
);

    // ------------------------------------------------------------------
    // 1. 특징 추출부
    // ------------------------------------------------------------------
    wire             regs_valid;
    wire [32*17-1:0] regs_flat;
    wire             feat_busy;

    // 인스턴스보다 먼저 선언해야 암묵적 1비트 선으로 잘리지 않는다
    wire       pair_err;                    // 두 갈래의 짝이 안 맞아 한쪽을 버린 적이 있음
    reg signed [4:0] norm_sh;               // 0x50 에 쓴 값. 포락선 배율
    reg [7:0]        skip_frames;           // 0x54 에 쓴 값. 앞 몇 프레임을 뺄지 (기본 0)

    spec_feat u_feat (
        .clk(clk), .rstn(rstn),
        .fft_tdata(fft_tdata), .fft_tuser(fft_tuser), .fft_tlast(fft_tlast),
        .fft_tvalid(fft_tvalid), .fft_tready(fft_tready),
        .skip_frames(skip_frames),
        .dc_tdata(dc_tdata), .dc_tvalid(dc_tvalid), .dc_tready(dc_tready),
        .norm_sh(norm_sh),
        .regs_valid(regs_valid), .regs_flat(regs_flat), .busy(feat_busy),
        .pair_err(pair_err)
    );

    // ------------------------------------------------------------------
    // 2. 그림자 레지스터 (PS 가 읽는 동안 값이 바뀌지 않게 한 번에 복사)
    //    AXI 클럭과 PL 클럭이 같다고 보고 만든다 (블록 디자인에서 같은 클럭을 연결할 것).
    // ------------------------------------------------------------------
    reg [31:0] shadow [0:16];
    reg [31:0] rec_count;
    reg        done;
    reg        ovr;                         // 읽기 전에 새 결과가 와서 버린 적이 있음

    // 바이트 0 이 활성일 때만 쓴다. wstrb 를 안 보면 WSTRB=0000 인 쓰기에도 값이 바뀐다.
    wire       wr_hit   = s_axi_awready && s_axi_awvalid && s_axi_wready && s_axi_wvalid
                          && s_axi_wstrb[0];
    wire [5:0] wr_word   = s_axi_awaddr[ADDR_W-1:2];
    wire       clr_done  = wr_hit && (wr_word == 6'd0) && s_axi_wdata[0];
    wire       clr_ovr   = wr_hit && (wr_word == 6'd0) && s_axi_wdata[2];
    wire       set_sh    = wr_hit && (wr_word == 6'd20);        // 0x50
    wire       set_skip  = wr_hit && (wr_word == 6'd21);        // 0x54

    integer k;
    always @(posedge clk) begin
        if (!rstn) begin
            for (k = 0; k < 17; k = k + 1) shadow[k] <= 32'd0;
            rec_count <= 32'd0;
            done      <= 1'b0;
            ovr       <= 1'b0;
            norm_sh   <= 5'sd0;
            skip_frames <= 8'd0;
        end else begin
            // PS 가 아직 안 읽은 결과(done=1)가 있으면 덮어쓰지 않는다.
            // 덮어쓰면 PS 가 17개를 읽는 도중 두 레코드 값이 섞인다.
            if (regs_valid) begin
                rec_count <= rec_count + 32'd1;
                if (!done) begin
                    for (k = 0; k < 17; k = k + 1) shadow[k] <= regs_flat[32*k +: 32];
                    done <= 1'b1;
                end else begin
                    ovr <= 1'b1;            // 이번 레코드는 버렸다
                end
            end else begin
                if (clr_done) done <= 1'b0;
                if (clr_ovr)  ovr  <= 1'b0;
            end
            if (set_sh)
                norm_sh <= s_axi_wdata[4:0];
            if (set_skip)
                skip_frames <= s_axi_wdata[7:0];
        end
    end

    assign irq = done;

    // ------------------------------------------------------------------
    // 3. AXI4-Lite 쓰기 (STATUS 의 done 지우기에만 쓰임)
    // ------------------------------------------------------------------
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_bvalid  <= 1'b0;
            s_axi_bresp   <= 2'b00;
        end else begin
            // 주소와 데이터가 모두 들어온 클럭에 한 번에 받는다
            if (!s_axi_awready && s_axi_awvalid && s_axi_wvalid && !s_axi_bvalid) begin
                s_axi_awready <= 1'b1;
                s_axi_wready  <= 1'b1;
                s_axi_bvalid  <= 1'b1;
                s_axi_bresp   <= 2'b00;
            end else begin
                s_axi_awready <= 1'b0;
                s_axi_wready  <= 1'b0;
                if (s_axi_bvalid && s_axi_bready)
                    s_axi_bvalid <= 1'b0;
            end
        end
    end

    // ------------------------------------------------------------------
    // 4. AXI4-Lite 읽기
    // ------------------------------------------------------------------
    wire [5:0] word = s_axi_araddr[ADDR_W-1:2];     // 4바이트 단위 번호
    reg  [31:0] rd_val;

    always @(*) begin
        if (word == 6'd0)                        rd_val = {28'd0, pair_err, ovr, feat_busy, done};
        else if (word == 6'd1)                   rd_val = rec_count;
        else if (word >= 6'd2 && word <= 6'd18)  rd_val = shadow[word - 6'd2];
        else if (word == 6'd20)                  rd_val = {{27{norm_sh[4]}}, norm_sh};
        else if (word == 6'd21)                  rd_val = {24'd0, skip_frames};
        else                                     rd_val = 32'd0;
    end

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rresp   <= 2'b00;
            s_axi_rdata   <= 32'd0;
        end else begin
            if (!s_axi_arready && s_axi_arvalid && !s_axi_rvalid) begin
                s_axi_arready <= 1'b1;
                s_axi_rdata   <= rd_val;
                s_axi_rresp   <= 2'b00;
                s_axi_rvalid  <= 1'b1;
            end else begin
                s_axi_arready <= 1'b0;
                if (s_axi_rvalid && s_axi_rready)
                    s_axi_rvalid <= 1'b0;
            end
        end
    end

endmodule
