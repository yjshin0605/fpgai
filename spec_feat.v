`timescale 1ns / 1ps
// ============================================================================
// spec_feat : 스펙트로그램·히스토그램 특징 추출 상위 모듈 (README 레지스터 17개)
//
//   FFT 출력 스트림(64b + 꼬리표) → cordic_mag → traj_frame → traj_rec → 궤적 레지스터 13개
//   dc_remove 출력 스트림 → env_detect(배율 적용) → env_hist → 분포 레지스터 4개
//
// 두 갈래가 모두 끝나면 regs_valid 를 1클럭 올린다. 레지스터 순서는 sim_fixed.REGS 와 같다.
//   0 n_active  1 n_pairs  2 n_zero  3 n_small  4 n_jump  5 mono  6 curv_sum  7 n_triples
//   8 rep_max   9 rep_lag 10 hmax   11 nocc    12 bw_sum 13 n_modes 14 occ 15 mean_q8 16 var_q8
// ============================================================================
module spec_feat (
    input  wire         clk,
    input  wire         rstn,               // 0 이면 초기화

    input  wire [63:0]  fft_tdata,          // fft_wrap 출력. [24:0] 실수부, [56:32] 허수부 (각 25비트)
    input  wire [16:0]  fft_tuser,          // [7:0] 빈 번호, [8] 무효 표시, [16:9] 프레임 번호
    input  wire         fft_tlast,          // 프레임의 마지막 빈 (k = 255)
    input  wire         fft_tvalid,         // fft_tdata 가 유효함
    output wire         fft_tready,         // 항상 1

    input  wire [35:0]  dc_tdata,           // dc_remove 출력 {Q[17:0], I[17:0]}, 레코드당 16,384샘플
    input  wire         dc_tvalid,          // dc_tdata 가 유효하다는 표시
    output wire         dc_tready,          // 항상 1
    input  wire signed [4:0] norm_sh,       // 포락선 배율. PS 가 레코드 시작 전에 정해 줌
    input  wire [7:0]   skip_frames,        // 이 번호보다 작은 프레임은 궤적 계산에서 뺀다 (기본 0)

    output reg          regs_valid,         // 17개 레지스터가 새로 나옴 (1클럭 펄스)
    output wire [32*17-1:0] regs_flat,      // 레지스터 i = regs_flat[32*i +: 32]
    output wire         busy                // 레코드 특징 계산 중
);

    // 크기 계산 (CORDIC)
    wire [17:0] mag_tdata;
    wire        mag_tvalid;

    cordic_mag #(.IN_W(25), .ITERS(16), .OUT_SHIFT(4), .OUT_W(18), .INT_W(28)) u_mag (
        .clk(clk), .rstn(rstn),
        .s_re(fft_tdata[24:0]),             // FFT 결과의 실수부
        .s_im(fft_tdata[56:32]),            // FFT 결과의 허수부
        .s_tvalid(fft_tvalid), .s_tready(fft_tready),
        .m_tdata(mag_tdata), .m_tvalid(mag_tvalid)
    );

    // 꼬리표를 CORDIC 지연(17클럭)만큼 늦춰서 크기와 짝을 맞춘다
    localparam MAG_LAT = 17;
    reg [17:0] tag_pipe [0:MAG_LAT-1];      // {프레임번호, 무효, 빈번호} + tlast
    integer t;
    always @(posedge clk) begin
        tag_pipe[0] <= {fft_tlast, fft_tuser};
        for (t = 1; t < MAG_LAT; t = t + 1) tag_pipe[t] <= tag_pipe[t-1];
    end
    wire [16:0] tag_user = tag_pipe[MAG_LAT-1][16:0];
    wire        tag_last = tag_pipe[MAG_LAT-1][17];

    wire        fr_valid;
    wire [7:0]  fr_p;
    wire [17:0] fr_m;
    wire [8:0]  fr_c;
    wire        fr_invalid;
    wire [7:0]  fr_frame;

    traj_frame #(.MAG_W(18), .BW_SHIFT(2)) u_frame (
        .clk(clk), .rstn(rstn),
        .s_tdata(mag_tdata),
        .s_bin(tag_user[7:0]),              // 빈 번호 (natural order, 모듈 안에서 +128)
        .s_invalid(tag_user[8]),            // 무효 표시
        .s_frame(tag_user[16:9]),           // 프레임 번호
        .s_tlast(tag_last),                 // 프레임 마지막 칸
        .s_tvalid(mag_tvalid), .s_tready(),
        .fr_valid(fr_valid), .fr_p(fr_p), .fr_m(fr_m), .fr_c(fr_c),
        .fr_invalid(fr_invalid), .fr_frame(fr_frame)
    );

    wire        tr_valid;
    wire [31:0] r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12;
    wire        tr_busy;

    traj_rec #(.MAG_W(18), .N_FR(127), .SMALL_STEP(4),
               .REP_MIN(2), .REP_MAX(63)) u_rec (
        .clk(clk), .rstn(rstn),
        .fr_valid(fr_valid), .fr_p(fr_p), .fr_m(fr_m), .fr_c(fr_c),
        .fr_invalid(fr_invalid), .fr_frame(fr_frame), .skip_frames(skip_frames),
        .regs_valid(tr_valid),
        .n_active(r0), .n_pairs(r1), .n_zero(r2), .n_small(r3), .n_jump(r4), .mono(r5),
        .curv_sum(r6), .n_triples(r7), .rep_max(r8), .rep_lag(r9), .hmax(r10), .nocc(r11),
        .bw_sum(r12), .busy(tr_busy)
    );

    wire        eh_valid;
    wire [31:0] r13, r14, r15, r16;

    // 포락선 검출: 정규화된 I/Q 에서 그 순간의 세기를 뽑는다
    wire [16:0] env_tdata;
    wire        env_tvalid;

    env_detect #(.IN_W(18), .OUT_W(17), .FRAC_MODE(0)) u_envd (
        .clk(clk), .rstn(rstn),
        .s_i(dc_tdata[17:0]),               // DC 제거된 I 샘플 (정규화 전)
        .s_q(dc_tdata[35:18]),              // DC 제거된 Q 샘플 (정규화 전)
        .s_valid(dc_tvalid), .s_ready(dc_tready),
        .norm_sh(norm_sh),                  // 레코드마다 PS 가 정해 주는 배율
        .e_tdata(env_tdata), .e_tvalid(env_tvalid)
    );

    env_hist #(.ENV_W(17), .N_REC(16384), .LOG2_N(14), .HIST_SHIFT(10), .OCC_THR(256)) u_env (
        .clk(clk), .rstn(rstn),
        .e_tdata(env_tdata), .e_tvalid(env_tvalid), .e_tready(),
        .regs_valid(eh_valid),
        .n_modes(r13), .occ(r14), .mean_q8(r15), .var_q8(r16)
    );

    assign busy = tr_busy;

    assign regs_flat = {r16, r15, r14, r13, r12, r11, r10, r9, r8, r7, r6, r5, r4, r3, r2, r1, r0};

    // 두 갈래가 모두 끝나면 한 번 알림
    reg tr_done, eh_done;
    always @(posedge clk) begin
        if (!rstn) begin
            tr_done <= 1'b0;  eh_done <= 1'b0;  regs_valid <= 1'b0;
        end else begin
            regs_valid <= 1'b0;
            if ((tr_done || tr_valid) && (eh_done || eh_valid)) begin
                regs_valid <= 1'b1;
                tr_done    <= 1'b0;
                eh_done    <= 1'b0;
            end else begin
                if (tr_valid) tr_done <= 1'b1;
                if (eh_valid) eh_done <= 1'b1;
            end
        end
    end

endmodule
