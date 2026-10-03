`timescale 1ns/1ps
module tb_reset;
reg clk=0;always #5 clk=~clk;
reg rstn=0,valid=0,invalid=0;reg[7:0] fr=0;
wire rv;wire[31:0] hm,na;
traj_rec #(.N_FR(4),.REP_MIN(2),.REP_MAX(2)) dut(.clk(clk),.rstn(rstn),.fr_valid(valid),.fr_p(8'd42),.fr_c(9'd1),.fr_invalid(invalid),.fr_frame(fr),.skip_frames(8'd0),.regs_valid(rv),.n_active(na),.hmax(hm));
task send4;integer i;begin
for(i=0;i<4;i=i+1)begin @(negedge clk);valid=1;fr=i;end
@(negedge clk);valid=0;end endtask
initial begin
repeat(4)@(negedge clk);rstn=1;send4();
wait(dut.hist[42]==2);@(negedge clk);rstn=0;repeat(4)@(negedge clk);rstn=1;invalid=1;send4();
wait(rv);#1;$display("RESET_AFTER_PARTIAL_PASS n_active=%0d hmax=%0d expected_hmax=0",na,hm);$finish;
end
initial begin #50000;$fatal(1,"timeout");end
endmodule
