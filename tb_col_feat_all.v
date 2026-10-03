`timescale 1ns / 1ps
// ============================================================================
// tb_col_feat_all : 여러 골든 샘플을 한 번의 시뮬레이션으로 검증한다.
//
// 준비: make_colfeat_vectors.py --all --out <폴더>
//       → <폴더>/list.txt 와 샘플마다 <id>_in.mem, <id>_exp.mem 이 생긴다.
//       list.txt 한 줄 = "id 입력파일_절대경로 기대값파일_절대경로"
//
// list.txt 위치: 아래 LIST_DEFAULT, 또는 시뮬레이션 옵션 -testplusarg LIST=<경로>
// .mem 파일은 Vivado 프로젝트에 추가하지 않아도 된다 (절대경로로 직접 읽음).
//
// 샘플마다: 리셋 → 65,280칸 입력 (tvalid 를 GAP_PCT % 확률로 쉼) → 255줄 비교
// 끝나면 통과·실패 요약을 콘솔에 찍고, list.txt 옆에 results.txt 로도 저장한다.
// ============================================================================
module tb_col_feat_all;

    localparam [8*256-1:0] LIST_DEFAULT = "C:/Users/yjshi/Downloads/a/vectors/list.txt";

    localparam BIN_W   = 32;
    localparam E_W     = BIN_W + 8;
    localparam IN_W    = BIN_W + 10;
    localparam EXP_W   = 2 * E_W + 9;
    localparam N_COL   = 255;
    localparam N_BIN   = 256;
    localparam N_IN    = N_COL * N_BIN;
    localparam GAP_PCT = 20;

    reg clk = 1'b0;
    always #5 clk = ~clk;                   // 100 MHz

    reg              rstn   = 1'b0;
    reg  [BIN_W-1:0] tdata  = {BIN_W{1'b0}};
    reg              tvalid = 1'b0;
    reg              tlast  = 1'b0;
    reg  [8:0]       tuser  = 9'd0;
    wire             tready;

    wire             col_valid;
    wire [E_W-1:0]   col_e;
    wire [7:0]       col_p;
    wire [E_W-1:0]   col_spread;
    wire             col_rec_last;

    col_feat #(.BIN_W(BIN_W)) dut (
        .clk(clk), .rstn(rstn),
        .s_axis_tdata(tdata), .s_axis_tvalid(tvalid), .s_axis_tready(tready),
        .s_axis_tlast(tlast), .s_axis_tuser(tuser),
        .col_valid(col_valid), .col_e(col_e), .col_p(col_p),
        .col_spread(col_spread), .col_rec_last(col_rec_last)
    );

    reg [IN_W-1:0]  in_mem  [0:N_IN-1];
    reg [EXP_W-1:0] exp_mem [0:N_COL-1];

    reg [8*256-1:0] list_file;
    reg [8*64-1:0]  sid;
    reg [8*256-1:0] in_file, exp_file;

    integer fd_list, fd_res, fd_chk, rc, i;
    integer seed      = 1;
    integer n_col     = 0;
    integer n_err     = 0;
    integer n_samples = 0;
    integer n_pass    = 0;
    integer n_fail    = 0;
    reg     checking  = 1'b0;
    reg     file_ok;

    // ------------------------------------------------------------ 샘플 반복
    initial begin
        if (!$value$plusargs("LIST=%s", list_file)) list_file = LIST_DEFAULT;
        fd_list = $fopen(list_file, "r");
        if (fd_list == 0) begin
            $display("[FAIL] cannot open list file: %0s", list_file);
            $finish;
        end
        fd_res = $fopen("results.txt", "w");    // xsim 실행 폴더에 저장

        rc = $fscanf(fd_list, "%s %s %s\n", sid, in_file, exp_file);
        while (rc == 3) begin
            // 파일이 없으면 $readmemh 가 이전 샘플 값을 그대로 남겨서 잘못 통과할 수 있으므로 먼저 확인
            file_ok = 1'b1;
            fd_chk = $fopen(in_file, "r");
            if (fd_chk == 0) file_ok = 1'b0; else $fclose(fd_chk);
            fd_chk = $fopen(exp_file, "r");
            if (fd_chk == 0) file_ok = 1'b0; else $fclose(fd_chk);

            n_samples = n_samples + 1;
            if (!file_ok) begin
                n_fail = n_fail + 1;
                $display("[FAIL] %0s : .mem file not found", sid);
                if (fd_res) $fdisplay(fd_res, "FAIL %0s file_not_found", sid);
            end else begin
                $readmemh(in_file,  in_mem);
                $readmemh(exp_file, exp_mem);

                // 리셋으로 이전 샘플 상태를 지움
                checking = 1'b0;
                rstn   <= 1'b0;
                tvalid <= 1'b0;
                tlast  <= 1'b0;
                repeat (5) @(posedge clk);
                n_col = 0;
                n_err = 0;
                checking = 1'b1;
                rstn <= 1'b1;
                @(posedge clk);

                for (i = 0; i < N_IN; i = i + 1) begin
                    while (({$random(seed)} % 100) < GAP_PCT) begin
                        tvalid <= 1'b0;
                        @(posedge clk);
                    end
                    tvalid <= 1'b1;
                    tdata  <= in_mem[i][BIN_W-1:0];
                    tuser  <= {in_mem[i][BIN_W+9], in_mem[i][BIN_W+7:BIN_W]};
                    tlast  <= in_mem[i][BIN_W+8];
                    @(posedge clk);
                end
                tvalid <= 1'b0;
                tlast  <= 1'b0;
                repeat (30) @(posedge clk);
                checking = 1'b0;

                if (n_col != N_COL) n_err = n_err + 1;
                if (n_err == 0) begin
                    n_pass = n_pass + 1;
                    $display("[PASS] %0s", sid);
                    if (fd_res) $fdisplay(fd_res, "PASS %0s", sid);
                end else begin
                    n_fail = n_fail + 1;
                    $display("[FAIL] %0s : %0d mismatches, %0d/%0d columns", sid, n_err, n_col, N_COL);
                    if (fd_res) $fdisplay(fd_res, "FAIL %0s mismatches=%0d columns=%0d", sid, n_err, n_col);
                end
            end
            rc = $fscanf(fd_list, "%s %s %s\n", sid, in_file, exp_file);
        end
        $fclose(fd_list);

        $display("==================================================");
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

    // ------------------------------------------------------------ 출력 비교
    always @(posedge clk) begin
        if (checking && rstn && col_valid) begin
            if (n_col >= N_COL) begin
                n_err = n_err + 1;
            end else if (col_e        !== exp_mem[n_col][E_W-1:0]      ||
                         col_p        !== exp_mem[n_col][E_W+7:E_W]    ||
                         col_spread   !== exp_mem[n_col][2*E_W+7:E_W+8] ||
                         col_rec_last !== exp_mem[n_col][2*E_W+8]) begin
                n_err = n_err + 1;
                if (n_err <= 3)
                    $display("    [MISMATCH] %0s col %0d : E %0d/%0d  p %0d/%0d  spread %0d/%0d (got/exp)",
                             sid, n_col,
                             col_e,      exp_mem[n_col][E_W-1:0],
                             col_p,      exp_mem[n_col][E_W+7:E_W],
                             col_spread, exp_mem[n_col][2*E_W+7:E_W+8]);
            end
            n_col = n_col + 1;
        end
    end

endmodule
