`timescale 1ns / 1ps
// ============================================================================
// tb_spec_feat_axi : spec_feat_axi 를 골든 샘플로 검증한다.
//                    특징값을 모듈 안에서 직접 보는 게 아니라, PS 가 하는 것과 똑같이
//                    AXI4-Lite 읽기로 가져와서 regs.mem 과 비교한다.
//
// 준비: make_list_from_share.py 로 만든 list_spec_feat.txt
//       한 줄 = "id fft spec pre_env peak hist regs"
//
// 샘플마다
//   리셋 → FFT 출력·포락선 넣기 → STATUS(0x00) 의 done 이 1이 될 때까지 읽기
//   → 0x08 ~ 0x48 에서 특징 17개 읽어 비교 → COUNT(0x04) 확인 → done 지우기(0x00 에 1 쓰기)
//
// N_TEST 를 줄이면 앞쪽 몇 개만 빠르게 돌릴 수 있다 (기본 144개 전부).
// ============================================================================
module tb_spec_feat_axi;

    localparam [8*256-1:0] LIST_DEFAULT = "C:/Users/yjshi/Downloads/a/gm_vec/list_spec_feat.txt";

    localparam N_FR    = 127;
    localparam N_SPEC  = N_FR * 256;
    localparam N_REC   = 16384;
    localparam N_REGS  = 17;
    localparam N_TEST  = 144;           // 검증할 샘플 수
    localparam GAP_PCT = 20;
    localparam TIMEOUT = 200000;

    reg clk = 1'b0;
    always #5 clk = ~clk;               // 100 MHz, AXI 도 같은 클럭 사용

    reg         rstn = 1'b0;
    reg  [31:0] fft_tdata = 32'd0;
    reg         fft_tvalid = 1'b0;
    reg  [16:0] env_tdata = 17'd0;
    reg         env_tvalid = 1'b0;
    wire        fft_tready, env_tready, irq;

    // AXI4-Lite
    reg  [7:0]  awaddr = 8'd0;   reg awvalid = 1'b0;   wire awready;
    reg  [31:0] wdata  = 32'd0;  reg wvalid  = 1'b0;   wire wready;
    wire [1:0]  bresp;           wire bvalid;          reg  bready = 1'b0;
    reg  [7:0]  araddr = 8'd0;   reg arvalid = 1'b0;   wire arready;
    wire [31:0] rdata;           wire [1:0] rresp;     wire rvalid;   reg rready = 1'b0;

    spec_feat_axi #(.ADDR_W(8)) dut (
        .clk(clk), .rstn(rstn),
        .fft_tdata(fft_tdata), .fft_tvalid(fft_tvalid), .fft_tready(fft_tready),
        .env_tdata(env_tdata), .env_tvalid(env_tvalid), .env_tready(env_tready),
        .irq(irq),
        .s_axi_aclk(clk), .s_axi_aresetn(rstn),
        .s_axi_awaddr(awaddr), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
        .s_axi_wdata(wdata), .s_axi_wstrb(4'hF), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
        .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
        .s_axi_araddr(araddr), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
        .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(rready)
    );

    reg [31:0] fft_mem  [0:N_SPEC-1];
    reg [16:0] env_mem  [0:N_REC-1];
    reg [31:0] regs_mem [0:N_REGS-1];

    reg [8*256-1:0] list_file, f_fft, f_spec, f_env, f_peak, f_hist, f_regs;
    reg [8*64-1:0]  sid;
    reg [31:0]      rd_word;

    integer fd_list, fd_chk, rc, i, j, g, waited;
    integer seed_s = 31, seed_e = 47;
    integer n_err = 0, n_samples = 0, n_pass = 0, n_fail = 0;
    reg     file_ok;

    // ------------------------------------------------------------ AXI 읽기 한 번
    task axi_read(input [7:0] addr);
        begin
            @(posedge clk);
            araddr  <= addr;
            arvalid <= 1'b1;
            rready  <= 1'b1;
            @(posedge clk);
            while (!arready) @(posedge clk);
            arvalid <= 1'b0;
            while (!rvalid) @(posedge clk);
            rd_word = rdata;
            @(posedge clk);
            rready <= 1'b0;
        end
    endtask

    // ------------------------------------------------------------ AXI 쓰기 한 번
    task axi_write(input [7:0] addr, input [31:0] data);
        begin
            @(posedge clk);
            awaddr  <= addr;   awvalid <= 1'b1;
            wdata   <= data;   wvalid  <= 1'b1;
            bready  <= 1'b1;
            @(posedge clk);
            while (!(awready && wready)) @(posedge clk);
            awvalid <= 1'b0;   wvalid <= 1'b0;
            while (!bvalid) @(posedge clk);
            @(posedge clk);
            bready <= 1'b0;
        end
    endtask

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

        rc = $fscanf(fd_list, "%s %s %s %s %s %s %s\n", sid, f_fft, f_spec, f_env, f_peak, f_hist, f_regs);
        while (rc == 7 && n_samples < N_TEST) begin
            file_ok = 1'b1;
            check_file(f_fft); check_file(f_env); check_file(f_regs);
            n_samples = n_samples + 1;

            if (!file_ok) begin
                n_fail = n_fail + 1;
                $display("[FAIL] %0s : .mem file not found", sid);
            end else begin
                $readmemh(f_fft,  fft_mem);
                $readmemh(f_env,  env_mem);
                $readmemh(f_regs, regs_mem);
                n_err = 0;

                rstn <= 1'b0;  fft_tvalid <= 1'b0;  env_tvalid <= 1'b0;
                repeat (5) @(posedge clk);
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

                // PS 가 하는 것과 같은 순서: done 기다리기 → 17개 읽기 → COUNT 확인 → done 지우기
                waited  = 0;
                rd_word = 32'd0;
                while (!rd_word[0] && waited < TIMEOUT) begin
                    axi_read(8'h00);
                    waited = waited + 1;
                end
                if (!rd_word[0]) begin
                    n_err = n_err + 1;
                    $display("    [TIMEOUT] %0s : STATUS.done never set", sid);
                end else begin
                    for (g = 0; g < N_REGS; g = g + 1) begin
                        axi_read(8'h08 + g * 4);
                        if (rd_word !== regs_mem[g]) begin
                            n_err = n_err + 1;
                            $display("    [MISMATCH] %0s reg %0d (0x%02X) : got %0d exp %0d",
                                     sid, g, 8'h08 + g * 4, rd_word, regs_mem[g]);
                        end
                    end
                    axi_read(8'h04);                            // 샘플마다 리셋하므로 1 이어야 함
                    if (rd_word !== 32'd1) begin
                        n_err = n_err + 1;
                        $display("    [MISMATCH] %0s COUNT : got %0d exp 1", sid, rd_word);
                    end
                    axi_write(8'h00, 32'd1);                    // done 지우기
                    axi_read(8'h00);
                    if (rd_word[0] !== 1'b0) begin
                        n_err = n_err + 1;
                        $display("    [MISMATCH] %0s : STATUS.done not cleared", sid);
                    end
                end

                if (n_err == 0) begin
                    n_pass = n_pass + 1;
                    $display("[PASS] %0s", sid);
                end else begin
                    n_fail = n_fail + 1;
                    $display("[FAIL] %0s : %0d mismatches", sid, n_err);
                end
            end
            rc = $fscanf(fd_list, "%s %s %s %s %s %s %s\n", sid, f_fft, f_spec, f_env, f_peak, f_hist, f_regs);
        end
        $fclose(fd_list);

        $display("==================================================");
        $display(" AXI read check : 0x00 STATUS, 0x04 COUNT, 0x08~0x48 regs 0~16");
        $display(" samples %0d : PASS %0d / FAIL %0d", n_samples, n_pass, n_fail);
        if (n_samples > 0 && n_fail == 0)
            $display(" [ALL PASS]");
        $display("==================================================");
        $finish;
    end

endmodule
