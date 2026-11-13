// Isolated compliant32 X candidate: one known 1024x16 1RW SRAM per row/set.
// Only words 0..63 carry X. Clear writes must cover all 0..1023 physical words.
// Read requests use the engine's weightRen/weightReadK prefetch schedule.
module TileXMacroRowBuffer (
  input wire clock, reset,
  input wire io_wen,
  input wire [9:0] io_waddr,
  input wire [15:0] io_wdata,
  input wire io_packed,
  input wire [15:0] io_wdataHi,
  input wire io_ren,
  input wire [5:0] io_raddr,
  output wire [15:0] io_rdata,
  output wire io_rvalid
);
  Sram1024x16 memory (
    .clock(clock), .reset(reset),
    .io_en(io_wen || io_ren), .io_isWrite(io_wen),
    .io_addr(io_wen ? io_waddr : {4'b0, io_raddr}),
    .io_wdata(io_wdata), .io_rdata(io_rdata), .io_rvalid(io_rvalid)
  );
`ifndef SYNTHESIS
  always @(posedge clock) if (!reset) begin
    assert (!(io_wen && io_ren)) else $fatal(1,"X single-port read/write overlap");
    assert (!(io_wen && io_packed)) else $fatal(1,"X macro candidate requires scalar16 writes");
  end
`endif
endmodule
