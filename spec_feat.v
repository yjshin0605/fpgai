`timescale 1ns / 1ps
// ============================================================================
// spec_feat : 스펙트로그램·히스토그램 특징 추출 상위 모듈 (README 레지스터 17개)
//
//   FFT 출력 스트림 → cordic_mag → traj_frame → traj_rec → 궤적 레지스터 13개
//   포락선 스트림       → env_hist              → 분포 레지스터 4개
//
// 두 갈래가 모두 끝나면 regs_valid 를 1클럭 올린다. 레지스터 순서는 sim_fixed.REGS 와 같다.
//   0 n_active  1 n_pairs  2 n_zero  3 n_small  4 n_jump  5 mono  6 curv_sum  7 n_triples
//   8 rep_max   9 rep_lag 10 hmax   11 nocc    12 bw_sum 13 n_modes 14 occ 15 mean_q8 16 var_q8
// ============================================================================
module spec_feat (
    input  wire         clk,
    input  wire         rstn,               // 0 이면 초기화

    input  wire [31:0]  fft_tdata,          // {Im[15:0], Re[15:0]}, fftshift 순서, 프레임당 256칸
    input  wire         fft_tvalid,
    output wire         fft_tready,

    input  wire [16:0]  env_tdata,          // 전처리 포락선, 레코드당 16,384샘플
    input  wire         env_tvalid,
    output wire         env_tready,

    output reg          regs_valid,         // 17개 레지스터가 새로 나옴 (1클럭 펄스)
    output wire [32*17-1:0] regs_flat,      // 레지스터 i = regs_flat[32*i +: 32]
    output wire         busy                // 레코드 특징 계산 중
);

    // 크기 계산 (CORDIC)
    wire [15:0] mag_tdata;
    wire        mag_tvalid;

    cordic_mag #(.ITERS(16), .OUT_SHIFT(1), .OUT_W(16)) u_mag (
        .clk(clk), .rstn(rstn),
        .s_tdata(fft_tdata), .s_tvalid(fft_tvalid), .s_tready(fft_tready),
        .m_tdata(mag_tdata), .m_tvalid(mag_tvalid)
    );

    wire        fr_valid;
    wire [7:0]  fr_p;
    wire [15:0] fr_m;
    wire [8:0]  fr_c;

    traj_frame #(.MAG_W(16), .BW_SHIFT(2)) u_frame (
        .clk(clk), .rstn(rstn),
        .s_tdata(mag_tdata), .s_tvalid(mag_tvalid), .s_tready(),
        .fr_valid(fr_valid), .fr_p(fr_p), .fr_m(fr_m), .fr_c(fr_c)
    );

    wire        tr_valid;
    wire [31:0] r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12;
    wire        tr_busy;

    traj_rec #(.MAG_W(16), .N_FR(127), .ACTIVE_SHIFT(3), .SMALL_STEP(4),
               .REP_MIN(2), .REP_MAX(63)) u_rec (
        .clk(clk), .rstn(rstn),
        .fr_valid(fr_valid), .fr_p(fr_p), .fr_m(fr_m), .fr_c(fr_c),
        .regs_valid(tr_valid),
        .n_active(r0), .n_pairs(r1), .n_zero(r2), .n_small(r3), .n_jump(r4), .mono(r5),
        .curv_sum(r6), .n_triples(r7), .rep_max(r8), .rep_lag(r9), .hmax(r10), .nocc(r11),
        .bw_sum(r12), .busy(tr_busy)
    );

    wire        eh_valid;
    wire [31:0] r13, r14, r15, r16;

    env_hist #(.ENV_W(17), .N_REC(16384), .LOG2_N(14), .HIST_SHIFT(10), .OCC_THR(256)) u_env (
        .clk(clk), .rstn(rstn),
        .e_tdata(env_tdata), .e_tvalid(env_tvalid), .e_tready(env_tready),
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
