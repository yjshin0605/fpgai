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
    output reg          pair_err,           // 짝이 안 맞아 한쪽 결과를 버림 (1클럭 펄스)
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

    traj_rec #(.N_FR(127), .SMALL_STEP(4),
               .REP_MIN(2), .REP_MAX(63)) u_rec (
        .clk(clk), .rstn(rstn),
        .fr_valid(fr_valid), .fr_p(fr_p), .fr_c(fr_c),          // fr_m 은 traj_rec 이 쓰지 않는다
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

    // ------------------------------------------------------------------
    // 두 갈래의 결과를 각각 붙잡아 두었다가, 둘 다 모이면 한 번 알린다.
    //
    // 완료 표시만 들고 있으면 안 된다. 궤적 계산이 끝나기 전에 다음 레코드의
    // 포락선 계산이 끝나면 env_hist 출력이 덮어써져, 레코드 A 의 궤적과
    // 레코드 B 의 분포가 섞인 결과가 한 번만 나온다. 그래서 값 자체를 복사해 둔다.
    // ------------------------------------------------------------------
    reg        tr_done, eh_done;
    reg [31:0] t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12;   // 궤적 13개 보관
    reg [31:0] e13, e14, e15, e16;                                      // 분포 4개 보관
    reg [31:0] o0, o1, o2, o3, o4, o5, o6, o7, o8, o9, o10, o11, o12;   // 내보낼 스냅샷
    reg [31:0] o13, o14, o15, o16;

    wire pair_ready = (tr_done || tr_valid) && (eh_done || eh_valid);

    // 짝이 맞는 순간 내보낼 값: 이미 보관해 둔 것(tr_done)이면 보관값, 지금 막 온 것이면 입력값
    wire use_t = tr_done;                   // 0 이면 이번 클럭의 r0~r12 가 짝의 주인
    wire use_e = eh_done;

    always @(posedge clk) begin
        if (!rstn) begin
            tr_done <= 1'b0;  eh_done <= 1'b0;  regs_valid <= 1'b0;  pair_err <= 1'b0;
        end else begin
            regs_valid <= 1'b0;
            pair_err   <= 1'b0;

            // 짝이 맞으면 그 자리에서 스냅샷을 떠서 내보낸다.
            // 스냅샷을 따로 두지 않으면, 짝을 내보내는 바로 그 클럭에 다음 레코드 값이
            // 도착했을 때 보관 레지스터가 덮여 PS 가 다른 레코드의 값을 읽게 된다.
            if (pair_ready) begin
                o0  <= use_t ? t0  : r0;   o1  <= use_t ? t1  : r1;
                o2  <= use_t ? t2  : r2;   o3  <= use_t ? t3  : r3;
                o4  <= use_t ? t4  : r4;   o5  <= use_t ? t5  : r5;
                o6  <= use_t ? t6  : r6;   o7  <= use_t ? t7  : r7;
                o8  <= use_t ? t8  : r8;   o9  <= use_t ? t9  : r9;
                o10 <= use_t ? t10 : r10;  o11 <= use_t ? t11 : r11;
                o12 <= use_t ? t12 : r12;
                o13 <= use_e ? e13 : r13;  o14 <= use_e ? e14 : r14;
                o15 <= use_e ? e15 : r15;  o16 <= use_e ? e16 : r16;
                regs_valid <= 1'b1;
                tr_done    <= 1'b0;
                eh_done    <= 1'b0;
            end

            // 갈래별 보관. 짝을 못 만난 값이 들어 있으면 덮어쓰지 않고 알린다.
            // pair_ready 인 클럭에는 보관 자리가 비므로 새 값을 받아도 된다 (스냅샷이 이미 떴다).
            // 도착한 값이 이번 짝에 쓰였으면(보관값이 없어 r 을 그대로 내보냈으면) 소비된 것이고,
            // 아니면 다음 짝을 기다리는 값이므로 대기 표시를 세워 둔다.
            // pair_ready 가 대기 표시를 무조건 지우면, 같은 클럭에 도착한 레코드가
            // 대기 목록에서 사라져 다음 짝부터 한 칸씩 밀린다.
            if (tr_valid) begin
                if (!tr_done || pair_ready) begin
                    t0 <= r0;  t1 <= r1;  t2 <= r2;  t3 <= r3;  t4 <= r4;  t5 <= r5;  t6 <= r6;
                    t7 <= r7;  t8 <= r8;  t9 <= r9;  t10 <= r10;  t11 <= r11;  t12 <= r12;
                    tr_done <= (pair_ready && !use_t) ? 1'b0 : 1'b1;
                end else begin
                    pair_err <= 1'b1;
                end
            end
            if (eh_valid) begin
                if (!eh_done || pair_ready) begin
                    e13 <= r13;  e14 <= r14;  e15 <= r15;  e16 <= r16;
                    eh_done <= (pair_ready && !use_e) ? 1'b0 : 1'b1;
                end else begin
                    pair_err <= 1'b1;
                end
            end
        end
    end

    // regs_valid 와 같은 클럭에 보이는 값 = 그 레코드의 17개 (스냅샷)
    assign regs_flat = {o16, o15, o14, o13, o12, o11, o10, o9, o8, o7, o6, o5, o4, o3, o2, o1, o0};

endmodule
