`timescale 1ns/1ps
module tb_axi_diag;
reg clk=0; always #5 clk=~clk;
reg rstn=0; reg[63:0] fft=0; reg[16:0] tag=0;reg last=0,fv=0;
reg[35:0] dc=0;reg dv=0;
always @(posedge clk) if(rstn && dut.pair_err_p) $display("PAIR_ERROR time=%0t",$time);
reg[7:0] aw=0,ar=0;reg av=0,wv=0,arv=0;reg[31:0] wd=0;reg[3:0] ws=15;
wire awr,wr,bv,arr,rv;wire[31:0] rd;wire[1:0] br,rr;wire irq;
spec_feat_axi dut(.clk(clk),.rstn(rstn),.fft_tdata(fft),.fft_tuser(tag),.fft_tlast(last),.fft_tvalid(fv),.fft_tready(),.dc_tdata(dc),.dc_tvalid(dv),.dc_tready(),.irq(irq),.s_axi_aclk(clk),.s_axi_aresetn(rstn),.s_axi_awaddr(aw),.s_axi_awvalid(av),.s_axi_awready(awr),.s_axi_wdata(wd),.s_axi_wstrb(ws),.s_axi_wvalid(wv),.s_axi_wready(wr),.s_axi_bresp(br),.s_axi_bvalid(bv),.s_axi_bready(1'b1),.s_axi_araddr(ar),.s_axi_arvalid(arv),.s_axi_arready(arr),.s_axi_rdata(rd),.s_axi_rresp(rr),.s_axi_rvalid(rv),.s_axi_rready(1'b1));

integer wc=0,rc=0;
always @(posedge clk) begin
 if(!rstn) begin wc=0;rc=0;end
 else begin
  if(bv && wc==0) $display("EARLY_BVALID: no write handshake completed before this edge");
  if(rv && rc==0) $display("EARLY_RVALID: no read handshake completed before this edge");
  if(awr&&av&&wr&&wv) wc=wc+1;
  if(arr&&arv) rc=rc+1;
 end
end
initial begin
 repeat(5)@(negedge clk);rstn=1;
 @(negedge clk);aw=8'h50;wd=4;av=1;wv=1;ar=8'h50;arv=1;
 @(posedge clk);@(posedge clk);@(negedge clk);av=0;wv=0;arv=0;
 repeat(5)@(negedge clk);$display("AXI norm_sh=%0d",dut.norm_sh);
 force dut.regs_valid=1;force dut.regs_flat=544'd111;
 @(negedge clk);force dut.regs_valid=0;
 @(negedge clk);force dut.regs_flat=544'd222;force dut.regs_valid=1;
 @(negedge clk);force dut.regs_valid=0;
 @(negedge clk);
 if(dut.shadow[0]!==111 || dut.ovr!==1 || dut.rec_count!==2) $fatal(1,"shadow protection failed");
 $display("PASS shadow preserved first=111, incoming=222 dropped, ovr=1, count=2");
 $finish;
end
endmodule
