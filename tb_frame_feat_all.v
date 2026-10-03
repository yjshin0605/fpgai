`timescale 1ns / 1ps
// ============================================================================
// tb_frame_feat_all : frame_feat 를 여러 골든 샘플로 한 번에 검증한다.
//
// 준비: make_colfeat_vectors.py --all --out <폴더>
//       → <폴더>/list_frame.txt 한 줄 = "id 줄특징파일(_exp.mem) 기대결과파일(_frame.mem)"
//         _exp.mem  : col_feat 기대 출력 = frame_feat 입력 (한 줄에 한 세로줄)
//         _frame.mem: frame_feat 기대 결과 10줄 (emax, emin, ncols, non, esum, ssum,
//                     npair, nstay, p10, p90), 각 48비트
//
// 샘플마다: 리셋 → 255줄을 col_valid 펄스로 넣기 (줄 사이 0~7클럭 랜덤 간격)
//           → feat_valid 를 기다려 10개 값 비교
// ============================================================================
module tb_frame_feat_all;

    localparam [8*256-1:0] LIST_DEFAULT = "C:/Users/yjshi/Downloads/a/vectors/list_frame.txt";

    localparam E_W     = 40;
    localparam EXP_W   = 2 * E_W + 9;
    localparam N_COL   = 255;
    localparam TIMEOUT = 20000;

    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg              rstn      = 1'b0;
    reg              col_valid = 1'b0;
    reg  [E_W-1:0]   col_e     = {E_W{1'b0}};
    reg  [7:0]       col_p     = 8'd0;
    reg  [E_W-1:0]   col_spread = {E_W{1'b0}};
    reg              col_rec_last = 1'b0;

    wire             feat_valid;
    wire [E_W-1:0]   f_emax, f_emin;
    wire [8:0]       f_ncols, f_non, f_npair, f_nstay;
    wire [E_W+7:0]   f_esum, f_ssum;
    wire [7:0]       f_p10, f_p90;
    wire             busy;

    frame_feat #(.E_W(E_W)) dut (
        .clk(clk), .rstn(rstn),
        .col_valid(col_valid), .col_e(col_e), .col_p(col_p),
        .col_spread(col_spread), .col_rec_last(col_rec_last),
        .feat_valid(feat_valid),
        .f_emax(f_emax), .f_emin(f_emin), .f_ncols(f_ncols), .f_non(f_non),
        .f_esum(f_esum), .f_ssum(f_ssum), .f_npair(f_npair), .f_nstay(f_nstay),
        .f_p10(f_p10), .f_p90(f_p90), .busy(busy)
    );

    reg [EXP_W-1:0] col_mem  [0:N_COL-1];
    reg [47:0]      fexp     [0:9];
    reg [47:0]      got      [0:9];

    reg [8*256-1:0] list_file, col_file, frame_file;
    reg [8*64-1:0]  sid;

    integer fd_list, fd_res, fd_chk, rc, i, g, n_bad, waited;
    integer seed      = 7;
    integer n_samples = 0;
    integer n_pass    = 0;
    integer n_fail    = 0;
    reg     file_ok;

    initial begin
        if (!$value$plusargs("LIST=%s", list_file)) list_file = LIST_DEFAULT;
        fd_list = $fopen(list_file, "r");
        if (fd_list == 0) begin
            $display("[FAIL] cannot open list file: %0s", list_file);
            $finish;
        end
        fd_res = $fopen("results_frame.txt", "w");

        rc = $fscanf(fd_list, "%s %s %s\n", sid, col_file, frame_file);
        while (rc == 3) begin
            file_ok = 1'b1;
            fd_chk = $fopen(col_file, "r");
            if (fd_chk == 0) file_ok = 1'b0; else $fclose(fd_chk);
            fd_chk = $fopen(frame_file, "r");
            if (fd_chk == 0) file_ok = 1'b0; else $fclose(fd_chk);

            n_samples = n_samples + 1;
            if (!file_ok) begin
                n_fail = n_fail + 1;
                $display("[FAIL] %0s : .mem file not found", sid);
                if (fd_res) $fdisplay(fd_res, "FAIL %0s file_not_found", sid);
            end else begin
                $readmemh(col_file,   col_mem);
                $readmemh(frame_file, fexp);

                rstn      <= 1'b0;
                col_valid <= 1'b0;
                repeat (5) @(posedge clk);
                rstn <= 1'b1;
                @(posedge clk);

                // 255줄 넣기
                for (i = 0; i < N_COL; i = i + 1) begin
                    repeat ({$random(seed)} % 8) @(posedge clk);
                    col_valid    <= 1'b1;
                    col_e        <= col_mem[i][E_W-1:0];
                    col_p        <= col_mem[i][E_W+7:E_W];
                    col_spread   <= col_mem[i][2*E_W+7:E_W+8];
                    col_rec_last <= col_mem[i][2*E_W+8];
                    @(posedge clk);
                    col_valid    <= 1'b0;
                    col_rec_last <= 1'b0;
                end

                // 결과 기다리기
                waited = 0;
                while (!feat_valid && waited < TIMEOUT) begin
                    @(posedge clk);
                    waited = waited + 1;
                end

                if (!feat_valid) begin
                    n_fail = n_fail + 1;
                    $display("[FAIL] %0s : no feat_valid within %0d cycles", sid, TIMEOUT);
                    if (fd_res) $fdisplay(fd_res, "FAIL %0s timeout", sid);
                end else begin
                    got[0] = f_emax;   got[1] = f_emin;   got[2] = f_ncols;  got[3] = f_non;
                    got[4] = f_esum;   got[5] = f_ssum;   got[6] = f_npair;  got[7] = f_nstay;
                    got[8] = f_p10;    got[9] = f_p90;
                    n_bad = 0;
                    for (g = 0; g < 10; g = g + 1) begin
                        if (got[g] !== fexp[g]) begin
                            n_bad = n_bad + 1;
                            $display("    [MISMATCH] %0s field %0d : got %0d exp %0d", sid, g, got[g], fexp[g]);
                        end
                    end
                    if (n_bad == 0) begin
                        n_pass = n_pass + 1;
                        $display("[PASS] %0s  (%0d cycles after last column)", sid, waited);
                        if (fd_res) $fdisplay(fd_res, "PASS %0s", sid);
                    end else begin
                        n_fail = n_fail + 1;
                        $display("[FAIL] %0s : %0d fields differ", sid, n_bad);
                        if (fd_res) $fdisplay(fd_res, "FAIL %0s fields=%0d", sid, n_bad);
                    end
                end
            end
            rc = $fscanf(fd_list, "%s %s %s\n", sid, col_file, frame_file);
        end
        $fclose(fd_list);

        $display("==================================================");
        $display(" field order: 0 emax 1 emin 2 ncols 3 non 4 esum 5 ssum 6 npair 7 nstay 8 p10 9 p90");
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
