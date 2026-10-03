`timescale 1ns / 1ps
// ============================================================================
// tb_spec_feat_all : spec_feat 를 dump_golden.py 기대 출력으로 한 번에 검증한다.
//
// 준비: python make_rtl_list.py <out/golden_v1> <영문 경로 폴더>
//       → list_spec_feat.txt 한 줄 = "id fft spec pre_env peak hist regs" (절대경로)
//
// 샘플마다
//   리셋 → FFT 출력(127 x 256)과 포락선(16,384)을 동시에 넣음 (둘 다 랜덤 쉼)
//   → CORDIC 크기를 spec.mem 과, 프레임마다 피크 칸·크기를 peak.mem 과 비교
//   → regs_valid 후 레지스터 17개(regs.mem), 히스토그램 32칸(hist.mem),
//     프레임별 활성 여부(peak.mem 24번 비트) 비교
// ============================================================================
module tb_spec_feat_all;

    localparam [8*256-1:0] LIST_DEFAULT = "C:/Users/yjshi/Downloads/a/gm_vec/list_spec_feat.txt";

    localparam N_FR    = 127;
    localparam N_SPEC  = N_FR * 256;
    localparam N_REC   = 16384;
    localparam N_REGS  = 17;
    localparam GAP_PCT = 20;
    localparam TIMEOUT = 50000;

    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg         rstn = 1'b0;
    reg  [31:0] fft_tdata = 32'd0;
    reg         fft_tvalid = 1'b0;
    reg  [16:0] env_tdata = 17'd0;
    reg         env_tvalid = 1'b0;
    wire        fft_tready, env_tready;
    wire        regs_valid;
    wire [32*N_REGS-1:0] regs_flat;

    spec_feat dut (
        .clk(clk), .rstn(rstn),
        .fft_tdata(fft_tdata), .fft_tvalid(fft_tvalid), .fft_tready(fft_tready),
        .env_tdata(env_tdata), .env_tvalid(env_tvalid), .env_tready(env_tready),
        .regs_valid(regs_valid), .regs_flat(regs_flat)
    );

    reg [31:0] fft_mem  [0:N_SPEC-1];
    reg [15:0] spec_mem [0:N_SPEC-1];
    reg [16:0] env_mem  [0:N_REC-1];
    reg [24:0] peak_mem [0:N_FR-1];
    reg [31:0] hist_mem [0:31];
    reg [31:0] regs_mem [0:N_REGS-1];

    reg [8*256-1:0] list_file, f_fft, f_spec, f_env, f_peak, f_hist, f_regs;
    reg [8*64-1:0]  sid;

    integer fd_list, fd_res, fd_chk, rc, i, j, g, waited;
    integer seed_s = 11, seed_e = 23;
    integer n_fr = 0, n_mag = 0, n_err = 0;
    integer n_samples = 0, n_pass = 0, n_fail = 0;
    reg     checking = 1'b0;
    reg     file_ok;

    // ------------------------------------------------------------ 프레임마다 피크 비교
    always @(posedge clk) begin
        if (checking && rstn && dut.fr_valid) begin
            if (n_fr < N_FR && (dut.fr_p !== peak_mem[n_fr][23:16] || dut.fr_m !== peak_mem[n_fr][15:0])) begin
                n_err = n_err + 1;
                if (n_err <= 3)
                    $display("    [MISMATCH] %0s frame %0d : bin %0d/%0d  mag %0d/%0d (got/exp)",
                             sid, n_fr, dut.fr_p, peak_mem[n_fr][23:16], dut.fr_m, peak_mem[n_fr][15:0]);
            end
            n_fr = n_fr + 1;
        end
    end

    // CORDIC 크기 비교 (spec.mem)
    always @(posedge clk) begin
        if (checking && rstn && dut.mag_tvalid) begin
            if (n_mag < N_SPEC && dut.mag_tdata !== spec_mem[n_mag]) begin
                n_err = n_err + 1;
                if (n_err <= 3)
                    $display("    [MISMATCH] %0s mag %0d : got %0d exp %0d", sid, n_mag, dut.mag_tdata, spec_mem[n_mag]);
            end
            n_mag = n_mag + 1;
        end
    end

    task check_file(input [8*256-1:0] fname);
        begin
            fd_chk = $fopen(fname, "r");
            if (fd_chk == 0) file_ok = 1'b0; else $fclose(fd_chk);
        end
    endtask

    initial begin
        if (!$value$plusargs("LIST=%s", list_file)) list_file = LIST_DEFAULT;
        fd_list = $fopen(list_file, "r");
        if (fd_list == 0) begin
            $display("[FAIL] cannot open list file: %0s", list_file);
            $finish;
        end
        fd_res = $fopen("results_spec_feat.txt", "w");

        rc = $fscanf(fd_list, "%s %s %s %s %s %s %s\n", sid, f_fft, f_spec, f_env, f_peak, f_hist, f_regs);
        while (rc == 7) begin
            file_ok = 1'b1;
            check_file(f_fft); check_file(f_spec); check_file(f_env);
            check_file(f_peak); check_file(f_hist); check_file(f_regs);
            n_samples = n_samples + 1;

            if (!file_ok) begin
                n_fail = n_fail + 1;
                $display("[FAIL] %0s : .mem file not found", sid);
                if (fd_res) $fdisplay(fd_res, "FAIL %0s file_not_found", sid);
            end else begin
                $readmemh(f_fft,  fft_mem);
                $readmemh(f_spec, spec_mem);
                $readmemh(f_env,  env_mem);
                $readmemh(f_peak, peak_mem);
                $readmemh(f_hist, hist_mem);
                $readmemh(f_regs, regs_mem);

                checking = 1'b0;
                rstn <= 1'b0;  fft_tvalid <= 1'b0;  env_tvalid <= 1'b0;
                repeat (5) @(posedge clk);
                n_fr = 0;  n_mag = 0;  n_err = 0;
                checking = 1'b1;
                rstn <= 1'b1;
                @(posedge clk);

                fork
                    begin : drive_fft
                        for (i = 0; i < N_SPEC; i = i + 1) begin
                            while (({$random(seed_s)} % 100) < GAP_PCT) begin
                                fft_tvalid <= 1'b0;
                                @(posedge clk);
                            end
                            fft_tvalid <= 1'b1;
                            fft_tdata  <= fft_mem[i];
                            @(posedge clk);
                        end
                        fft_tvalid <= 1'b0;
                    end
                    begin : drive_env
                        for (j = 0; j < N_REC; j = j + 1) begin
                            while (({$random(seed_e)} % 100) < GAP_PCT) begin
                                env_tvalid <= 1'b0;
                                @(posedge clk);
                            end
                            env_tvalid <= 1'b1;
                            env_tdata  <= env_mem[j];
                            @(posedge clk);
                        end
                        env_tvalid <= 1'b0;
                    end
                join

                waited = 0;
                while (!regs_valid && waited < TIMEOUT) begin
                    @(posedge clk);
                    waited = waited + 1;
                end
                checking = 1'b0;

                if (!regs_valid) begin
                    n_err = n_err + 1;
                    $display("    [TIMEOUT] %0s : no regs_valid", sid);
                end else begin
                    if (n_fr != N_FR) begin
                        n_err = n_err + 1;
                        $display("    [MISMATCH] %0s : frames %0d/%0d", sid, n_fr, N_FR);
                    end
                    if (n_mag != N_SPEC) begin
                        n_err = n_err + 1;
                        $display("    [MISMATCH] %0s : magnitudes %0d/%0d", sid, n_mag, N_SPEC);
                    end
                    for (g = 0; g < N_REGS; g = g + 1)
                        if (regs_flat[32*g +: 32] !== regs_mem[g]) begin
                            n_err = n_err + 1;
                            $display("    [MISMATCH] %0s reg %0d : got %0d exp %0d", sid, g, regs_flat[32*g +: 32], regs_mem[g]);
                        end
                    for (g = 0; g < 32; g = g + 1)
                        if ({17'd0, dut.u_env.hs[g]} !== hist_mem[g]) begin
                            n_err = n_err + 1;
                            if (n_err <= 5)
                                $display("    [MISMATCH] %0s hist %0d : got %0d exp %0d", sid, g, dut.u_env.hs[g], hist_mem[g]);
                        end
                    for (g = 0; g < N_FR; g = g + 1)
                        if (dut.u_rec.aa[g] !== peak_mem[g][24]) begin
                            n_err = n_err + 1;
                            if (n_err <= 5)
                                $display("    [MISMATCH] %0s active frame %0d : got %0d exp %0d", sid, g, dut.u_rec.aa[g], peak_mem[g][24]);
                        end
                end

                if (n_err == 0) begin
                    n_pass = n_pass + 1;
                    $display("[PASS] %0s", sid);
                    if (fd_res) $fdisplay(fd_res, "PASS %0s", sid);
                end else begin
                    n_fail = n_fail + 1;
                    $display("[FAIL] %0s : %0d mismatches", sid, n_err);
                    if (fd_res) $fdisplay(fd_res, "FAIL %0s mismatches=%0d", sid, n_err);
                end
            end
            rc = $fscanf(fd_list, "%s %s %s %s %s %s %s\n", sid, f_fft, f_spec, f_env, f_peak, f_hist, f_regs);
        end
        $fclose(fd_list);

        $display("==================================================");
        $display(" reg order: 0 n_active 1 n_pairs 2 n_zero 3 n_small 4 n_jump 5 mono 6 curv_sum");
        $display("            7 n_triples 8 rep_max 9 rep_lag 10 hmax 11 nocc 12 bw_sum");
        $display("            13 n_modes 14 occ 15 mean_q8 16 var_q8");
        $display(" samples %0d : PASS %0d / FAIL %0d", n_samples, n_pass, n_fail);
        if (n_samples > 0 && n_fail == 0)
            $display(" [ALL PASS]");
        $display("==================================================");
        if (fd_res) begin
            $fdisplay(fd_res, "TOTAL %0d PASS %0d FAIL %0d", n_samples, n_pass, n_fail);
            $fclose(fd_res);
        end
        $finish;
    end

endmodule
