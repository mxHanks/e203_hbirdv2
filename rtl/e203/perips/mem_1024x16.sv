// Experimental adapter: EMA/EMAW must be confirmed for the selected PVT corner.
// Compile in place of mem_1024x16, not alongside its behavioral definition.
module mem_1024x16 #(
  parameter [2:0] XU_EMA = 3'b010,
  parameter [1:0] XU_EMAW = 2'b00
)(
  input [9:0] RW0_addr,
  input RW0_en, RW0_clk, RW0_wmode,
  input [15:0] RW0_wdata,
  output [15:0] RW0_rdata
);
  wire [15:0] macro_q;
  reg valid_read;
  always @(posedge RW0_clk) valid_read <= RW0_en & ~RW0_wmode;
  RAMSP1024X16 macro (
    .Q(macro_q), .CLK(RW0_clk), .CEN(~RW0_en),
    .WEN(16'h0000), .GWEN(~RW0_wmode), .A(RW0_addr),
    .D(RW0_wdata), .EMA(XU_EMA), .EMAW(XU_EMAW), .RET1N(1'b1)
  );
  assign RW0_rdata = valid_read ? macro_q : 16'bx;
endmodule
