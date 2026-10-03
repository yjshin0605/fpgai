`timescale 1ns / 1ps
// ============================================================================
// tb_spec_feat_smoke : 새 인터페이스로 바뀐 spec_feat_axi 의 동작 확인용.
//
// 골든 모델이 새 파이프라인(25비트 FFT, natural order, 무효 표시, 정규화 전 포락선)에
// 맞게 수정되기 전까지 쓰는 임시 검증이다. 값의 정확성이 아니라 아래를 본다.
//
//   1. 프레임 127개를 받으면 regs_valid 가 뜨는가
//   2. 레지스터 17개에 X(미확정) 가 없는가
//   3. 무효 표시를 세운 프레임이 n_active 에서 빠지는가
//   4. skip_frames 레지스터가 동작하는가
//   5. 포락선 배율(norm_sh)을 바꾸면 분포 특징이 따라 바뀌는가
//   6. 두 레코드를 연달아 넣어도 값이 섞이지 않는가
//
// 입력 신호는 테스트벤치가 만든다. CW 펄스를 흉내 낸 것으로,
// 켜진 구간은 한 칸에 큰 값, 꺼진 구간은 작은 잡음을 넣는다.
// ============================================================================
module tb_spec_feat_smoke;

    localparam N_FR   = 127;
    localparam N_BIN  = 256;
    localparam N_REC  = 16384;
    localparam N_REGS = 17;

    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg         rstn = 1'b0;
    reg  [63:0] fft_tdata = 64'd0;
    reg  [16:0] fft_tuser = 17'd0;
    reg         fft_tlast = 1'b0;
    reg         fft_tvalid = 1'b0;
    reg  [35:0] dc_tdata = 36'd0;
    reg         dc_tvalid = 1'b0;
    wire        fft_tready, dc_tready, irq;
    wire        regs_valid;
    wire [32*N_REGS-1:0] regs_flat;

    reg  [7:0]  awaddr = 8'd0;   reg awvalid = 1'b0;   wire awready;
    reg  [31:0] wdata  = 32'd0;  reg wvalid  = 1'b0;   wire wready;
    reg  [3:0]  wstrb_r = 4'hF;
    wire [1:0]  bresp;           wire bvalid;          reg  bready = 1'b0;
    reg  [7:0]  araddr = 8'd0;   reg arvalid = 1'b0;   wire arready;
    wire [31:0] rdata;           wire [1:0] rresp;     wire rvalid;   reg rready = 1'b0;

    spec_feat_axi #(.ADDR_W(8)) dut (
        .clk(clk), .rstn(rstn),
        .fft_tdata(fft_tdata), .fft_tuser(fft_tuser), .fft_tlast(fft_tlast),
        .fft_tvalid(fft_tvalid), .fft_tready(fft_tready),
        .dc_tdata(dc_tdata), .dc_tvalid(dc_tvalid), .dc_tready(dc_tready),
        .irq(irq),
        .s_axi_aclk(clk), .s_axi_aresetn(rstn),
        .s_axi_awaddr(awaddr), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
        .s_axi_wdata(wdata), .s_axi_wstrb(wstrb_r), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
        .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
        .s_axi_araddr(araddr), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
        .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(rready)
    );

    integer f, b, j, g, waited, n_err;
    integer seed = 3;
    reg [31:0] rd_word;
    reg [31:0] reg_val [0:N_REGS-1];

    // ------------------------------------------------------------ AXI 읽기/쓰기
    task axi_read(input [7:0] addr);
        begin
            @(posedge clk);
            araddr <= addr;  arvalid <= 1'b1;  rready <= 1'b1;
            @(posedge clk);
            while (!arready) @(posedge clk);
            arvalid <= 1'b0;
            while (!rvalid) @(posedge clk);
            rd_word = rdata;
            @(posedge clk);
            rready <= 1'b0;
        end
    endtask

    task axi_write(input [7:0] addr, input [31:0] data);
        begin
            @(posedge clk);
            awaddr <= addr;  awvalid <= 1'b1;
            wdata  <= data;  wvalid  <= 1'b1;
            bready <= 1'b1;
            @(posedge clk);
            while (!(awready && wready)) @(posedge clk);
            awvalid <= 1'b0;  wvalid <= 1'b0;
            while (!bvalid) @(posedge clk);
            @(posedge clk);
            bready <= 1'b0;
        end
    endtask

    task axi_write_strb(input [7:0] addr, input [31:0] data, input [3:0] strb);
        begin
            @(posedge clk);
            awaddr <= addr;  awvalid <= 1'b1;
            wdata  <= data;  wvalid  <= 1'b1;  wstrb_r <= strb;
            bready <= 1'b1;
            @(posedge clk);
            while (!(awready && wready)) @(posedge clk);
            awvalid <= 1'b0;  wvalid <= 1'b0;  wstrb_r <= 4'hF;
            while (!bvalid) @(posedge clk);
            @(posedge clk);
            bready <= 1'b0;
        end
    endtask

    // 127프레임 전부 무효 표시로 넣는다 (n_active 0 이 나와야 함)
    task drive_record_allinvalid;
        integer fi, bi;
        begin
            fork
                begin
                    for (fi = 0; fi < N_FR; fi = fi + 1)
                        for (bi = 0; bi < N_BIN; bi = bi + 1) begin
                            fft_tvalid <= 1'b1;
                            fft_tdata  <= {7'd0, 25'd0, 7'd0, 25'd500};
                            fft_tuser  <= {fi[7:0], 1'b1, bi[7:0]};   // 무효 표시 1
                            fft_tlast  <= (bi == N_BIN-1);
                            @(posedge clk);
                        end
                    fft_tvalid <= 1'b0;  fft_tlast <= 1'b0;
                end
                begin
                    for (j = 0; j < N_REC; j = j + 1) begin
                        dc_tvalid <= 1'b1;
                        dc_tdata  <= {18'd200, 18'd200};
                        @(posedge clk);
                    end
                    dc_tvalid <= 1'b0;
                end
            join
        end
    endtask

    // ------------------------------------------------------------ 한 레코드 넣기
    // on_每: 켜진 프레임 간격 (예: 4 면 4프레임마다 하나가 켜짐)
    // mark_invalid: 1 이면 꺼진 프레임에 무효 표시를 세운다
    task drive_record(input integer on_every, input integer mark_invalid, input integer peak_bin);
        integer fi, bi;
        reg on;
        reg [24:0] v;
        begin
            fork
                begin : drive_fft
                    for (fi = 0; fi < N_FR; fi = fi + 1) begin
                        on = ((fi % on_every) == 0);
                        for (bi = 0; bi < N_BIN; bi = bi + 1) begin
                            while (({$random(seed)} % 100) < 15) begin
                                fft_tvalid <= 1'b0;
                                @(posedge clk);
                            end
                            // 켜진 프레임은 peak_bin 에 큰 값, 아니면 작은 잡음
                            if (on && bi == peak_bin) v = 25'd1000000;
                            else if (on && (bi == peak_bin-1 || bi == peak_bin+1)) v = 25'd300000;
                            else                      v = {$random(seed)} % 2000;
                            fft_tvalid <= 1'b1;
                            fft_tdata  <= {7'd0, 25'd0, 7'd0, v};      // 허수부 0, 실수부만
                            fft_tuser  <= {fi[7:0], (mark_invalid && !on), bi[7:0]};
                            fft_tlast  <= (bi == N_BIN-1);
                            @(posedge clk);
                        end
                    end
                    fft_tvalid <= 1'b0;  fft_tlast <= 1'b0;
                end
                begin : drive_dc
                    for (j = 0; j < N_REC; j = j + 1) begin
                        while (({$random(seed)} % 100) < 15) begin
                            dc_tvalid <= 1'b0;
                            @(posedge clk);
                        end
                        // 켜진 구간은 큰 값, 꺼진 구간은 작은 값 (봉우리 두 개가 나와야 함)
                        dc_tvalid <= 1'b1;
                        if ((j / 1024) % on_every == 0) dc_tdata <= {18'd3000, 18'd3000};
                        else                            dc_tdata <= {18'd200, 18'd200};
                        @(posedge clk);
                    end
                    dc_tvalid <= 1'b0;
                end
            join
        end
    endtask

    // ------------------------------------------------------------ 결과 읽기
    task read_regs;
        begin
            waited = 0;
            rd_word = 32'd0;
            while (!rd_word[0] && waited < 20000) begin
                axi_read(8'h00);
                waited = waited + 1;
            end
            if (!rd_word[0]) begin
                $display("  [FAIL] done 이 뜨지 않음");
                n_err = n_err + 1;
            end else begin
                for (g = 0; g < N_REGS; g = g + 1) begin
                    axi_read(8'h08 + g * 4);
                    reg_val[g] = rd_word;
                    if (^rd_word === 1'bx) begin
                        $display("  [FAIL] reg %0d 에 X 가 있음", g);
                        n_err = n_err + 1;
                    end
                end
                axi_write(8'h00, 32'd1);
            end
        end
    endtask

    task show;
        begin
            $display("    n_active=%0d n_pairs=%0d n_zero=%0d n_small=%0d n_jump=%0d",
                     reg_val[0], reg_val[1], reg_val[2], reg_val[3], reg_val[4]);
            $display("    hmax=%0d nocc=%0d bw_sum=%0d | n_modes=%0d occ=%0d mean=%0d var=%0d",
                     reg_val[10], reg_val[11], reg_val[12], reg_val[13], reg_val[14], reg_val[15], reg_val[16]);
        end
    endtask

    task do_reset;
        begin
            rstn <= 1'b0;  fft_tvalid <= 1'b0;  dc_tvalid <= 1'b0;
            repeat (5) @(posedge clk);
            rstn <= 1'b1;
            @(posedge clk);
        end
    endtask

    initial begin
        n_err = 0;

        // --- 1. 무효 표시 없이 (전부 활성이어야 함)
        do_reset;
        axi_write(8'h50, 32'd0);                      // 배율 0
        axi_write(8'h54, 32'd0);                      // 앞 프레임 안 뺌
        $display("[1] 무효 표시 없음, 4프레임마다 켜짐");
        drive_record(4, 0, 100);
        read_regs;  show;
        if (reg_val[0] !== 127) begin
            $display("  [FAIL] n_active 가 127 이어야 하는데 %0d", reg_val[0]);
            n_err = n_err + 1;
        end

        // --- 2. 꺼진 프레임에 무효 표시 (약 32개만 활성이어야 함)
        do_reset;
        axi_write(8'h50, 32'd0);
        axi_write(8'h54, 32'd0);
        $display("[2] 꺼진 프레임에 무효 표시");
        drive_record(4, 1, 100);
        read_regs;  show;
        if (reg_val[0] > 40 || reg_val[0] < 25) begin
            $display("  [FAIL] n_active 가 32 근처여야 하는데 %0d", reg_val[0]);
            n_err = n_err + 1;
        end

        // --- 3. skip_frames = 24 (활성이 더 줄어야 함)
        do_reset;
        axi_write(8'h50, 32'd0);
        axi_write(8'h54, 32'd24);
        $display("[3] skip_frames = 24");
        drive_record(4, 1, 100);
        read_regs;  show;
        if (reg_val[0] > 32) begin
            $display("  [FAIL] skip 을 켰는데 n_active 가 안 줄었음 (%0d)", reg_val[0]);
            n_err = n_err + 1;
        end

        // --- 4. 포락선 배율을 올리면 분포가 퍼져야 함
        do_reset;
        axi_write(8'h50, 32'd4);                      // 16배
        axi_write(8'h54, 32'd0);
        $display("[4] norm_sh = 4 (포락선 16배)");
        drive_record(4, 1, 100);
        read_regs;  show;
        if (reg_val[14] < 2) begin
            $display("  [FAIL] occ 가 2 이상이어야 하는데 %0d (분포가 한 칸에 몰림)", reg_val[14]);
            n_err = n_err + 1;
        end

        // --- 5. 레코드 두 개 연달아
        do_reset;
        axi_write(8'h50, 32'd4);
        axi_write(8'h54, 32'd0);
        $display("[5] 레코드 두 개 연달아");
        drive_record(4, 1, 100);
        read_regs;
        drive_record(8, 1, 60);
        read_regs;  show;
        axi_read(8'h04);
        if (rd_word !== 32'd2) begin
            $display("  [FAIL] COUNT 가 2 여야 하는데 %0d", rd_word);
            n_err = n_err + 1;
        end

        // --- 6. 레코드 두 개 연속, 결과가 섞이지 않는지 (갈래별 보관 확인)
        do_reset;
        axi_write(8'h50, 32'd0);
        axi_write(8'h54, 32'd0);
        $display("[6] rec A(4프레임마다) -> rec B(8프레임마다), 각각 읽기");
        drive_record(4, 1, 100);
        read_regs;
        if (reg_val[0] < 25 || reg_val[0] > 40) begin
            $display("  [FAIL] rec A n_active 32 근처여야 하는데 %0d", reg_val[0]);
            n_err = n_err + 1;
        end
        drive_record(8, 1, 60);
        read_regs;
        if (reg_val[0] < 12 || reg_val[0] > 20) begin
            $display("  [FAIL] rec B n_active 16 근처여야 하는데 %0d", reg_val[0]);
            n_err = n_err + 1;
        end

        // --- 7. WSTRB = 0 쓰기는 무시되어야 함
        $display("[7] WSTRB 확인");
        axi_write(8'h50, 32'd7);
        axi_read(8'h50);
        if (rd_word !== 32'd7) begin $display("  [FAIL] 정상 쓰기 실패 %0d", rd_word); n_err = n_err + 1; end
        axi_write_strb(8'h50, 32'd0, 4'b0000);
        axi_read(8'h50);
        if (rd_word !== 32'd7) begin
            $display("  [FAIL] WSTRB=0 인데 NORM_SH 가 바뀜 (%0d)", rd_word);
            n_err = n_err + 1;
        end
        axi_write(8'h50, 32'd0);

        // --- 8. 히스토그램 누적 도중 리셋 -> 전부 무효 레코드면 hmax 도 0 이어야 함
        //     리셋은 레코드를 다 보낸 뒤, traj_rec 이 PASS 상태에서 히스토그램을
        //     쌓고 있는 동안 건다. 전송 도중에 걸면 남은 프레임이 다음 레코드에
        //     섞여 들어가 프레임 정렬이 깨지므로 이 항목을 확인할 수 없다.
        do_reset;
        axi_write(8'h50, 32'd0);
        axi_write(8'h54, 32'd0);
        $display("[8] hist 누적 도중 리셋 -> 전부 무효 레코드");
        drive_record(4, 1, 100);                    // 레코드를 끝까지 보냄
        repeat (300) @(posedge clk);                // PASS 중간 (전체 762클럭)
        rstn <= 1'b0;
        repeat (5) @(posedge clk);
        rstn <= 1'b1;
        repeat (600) @(posedge clk);                // S_CLR 이 256칸 비울 시간
        drive_record_allinvalid;                    // 1차: 리셋 전 찌꺼기를 흘려보냄
        read_regs;
        $display("    (1차) n_active=%0d hmax=%0d", reg_val[0], reg_val[10]);
        drive_record_allinvalid;                    // 2차: 이게 깨끗해야 한다
        read_regs;  show;
        if (reg_val[0] !== 0 || reg_val[10] !== 0) begin
            $display("  [FAIL] 전부 무효인데 n_active=%0d hmax=%0d (hist 에 잔여가 남음)",
                     reg_val[0], reg_val[10]);
            n_err = n_err + 1;
        end

        // --- 9. 레지스터 되읽기
        axi_write(8'h50, 32'd4);
        axi_write(8'h54, 32'd0);
        axi_read(8'h50);
        if (rd_word !== 32'd4) begin $display("  [FAIL] NORM_SH 되읽기 %0d", rd_word); n_err = n_err + 1; end
        axi_read(8'h54);
        if (rd_word !== 32'd0) begin $display("  [FAIL] SKIP_FR 되읽기 %0d", rd_word); n_err = n_err + 1; end

        $display("==================================================");
        if (n_err == 0) $display(" [SMOKE PASS] 동작 확인 완료 (값의 정확성은 새 골든으로 검증 필요)");
        else            $display(" [SMOKE FAIL] 문제 %0d 건", n_err);
        $display("==================================================");
        $finish;
    end

endmodule
