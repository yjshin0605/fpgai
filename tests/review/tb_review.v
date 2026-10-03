`timescale 1ns/1ps
module tb_review;
reg clk=0; always #5 clk=~clk;
reg rstn=0; reg[63:0] fft=0; reg[16:0] tag=0;reg last=0,fv=0;
reg[35:0] dc=0;reg dv=0;
reg[7:0] aw=0,ar=0;reg av=0,wv=0,arv=0;reg[31:0] wd=0;reg[3:0] ws=15;
wire awr,wr,bv,arr,rv;wire[31:0] rd;wire[1:0] br,rr;wire irq;
spec_feat_axi dut(.clk(clk),.rstn(rstn),.fft_tdata(fft),.fft_tuser(tag),.fft_tlast(last),.fft_tvalid(fv),.fft_tready(),.dc_tdata(dc),.dc_tvalid(dv),.dc_tready(),.irq(irq),.s_axi_aclk(clk),.s_axi_aresetn(rstn),.s_axi_awaddr(aw),.s_axi_awvalid(av),.s_axi_awready(awr),.s_axi_wdata(wd),.s_axi_wstrb(ws),.s_axi_wvalid(wv),.s_axi_wready(wr),.s_axi_bresp(br),.s_axi_bvalid(bv),.s_axi_bready(1'b1),.s_axi_araddr(ar),.s_axi_arvalid(arv),.s_axi_arready(arr),.s_axi_rdata(rd),.s_axi_rresp(rr),.s_axi_rvalid(rv),.s_axi_rready(1'b1));
integer phase=0,seen=0;
always @(posedge clk) if(rstn && dut.regs_valid) begin
 seen=seen+1;
 $display("RESULT phase=%0d index=%0d active=%0d mean_q8=%0d",phase,seen,dut.regs_flat[0+:32],dut.regs_flat[480+:32]);
end
task reset;begin @(negedge clk);rstn=0;fv=0;dv=0;av=0;wv=0;repeat(5)@(negedge clk);rstn=1;repeat(5)@(negedge clk);seen=0;end endtask
task write_reg(input[7:0] addr,input[31:0] data,input[3:0] strobe);begin
 @(negedge clk);aw=addr;wd=data;ws=strobe;av=1;wv=1;
 do @(posedge clk); while(!(awr&&wr));
 @(negedge clk);av=0;wv=0;repeat(4)@(negedge clk);
end endtask
task send_fft(input integer nr,input integer gaps);integer r,f,k;begin
 for(r=0;r<nr;r=r+1)for(f=0;f<127;f=f+1)for(k=0;k<256;k=k+1)begin
  @(negedge clk);fv=1;last=(k==255);tag={f[7:0],1'b0,k[7:0]};fft=(k==0)?64'd16384:64'd0;
  if(gaps && k%11==0)begin @(negedge clk);fv=0;end
 end
 @(negedge clk);fv=0;last=0;
end endtask
task send_dc(input integer nr);integer r,k;begin
 for(r=0;r<nr;r=r+1)for(k=0;k<16384;k=k+1)begin
  @(negedge clk);dv=1;dc=(r+1)*1024;
 end
 @(negedge clk);dv=0;
end endtask
initial begin
 phase=1;reset();fork send_fft(1,1);send_dc(1);join
 repeat(9000)@(negedge clk);
 if(seen!=1 || dut.shadow[0]!=127 || dut.shadow[1]!=126 || dut.shadow[2]!=126 || dut.shadow[8]!=125 || dut.shadow[9]!=2 || dut.shadow[12]!=127 || dut.shadow[15]!=256) $fatal(1,"single record mismatch");
 $display("PASS single record, natural-order peak, valid gaps, 17-stage tags");
 write_reg(8'h50,5,0);$display("WSTRB_ZERO norm_sh=%0d expected=0",dut.norm_sh);
 write_reg(8'h54,24,2);$display("WSTRB_OTHER_BYTE skip_frames=%0d expected=0",dut.skip_frames);
 phase=2;reset();fork send_fft(2,0);send_dc(2);join
 repeat(9000)@(negedge clk);
 $display("CONTINUOUS results=%0d expected=2; final_mean=%0d",seen,dut.shadow[15]);
 $finish;
end
initial begin #3000000;$fatal(1,"timeout");end
endmodule
