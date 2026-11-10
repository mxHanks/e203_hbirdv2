// Experimental peripheral-only width adapter/arbiter. CPU remains RV32.
// One global ordered request; retirement and replacement may share an edge.
module xu_dma64_shared_arb (
  input wire clk, input wire rst_n,
  input wire cpu_cmd_valid, output wire cpu_cmd_ready,
  input wire [31:0] cpu_cmd_addr, input wire cpu_cmd_read,
  input wire [31:0] cpu_cmd_wdata, input wire [3:0] cpu_cmd_wmask,
  output wire cpu_rsp_valid, input wire cpu_rsp_ready,
  output wire cpu_rsp_err, output wire [31:0] cpu_rsp_rdata,
  input wire dma_cmd_valid, output wire dma_cmd_ready,
  input wire [31:0] dma_cmd_addr, input wire dma_cmd_read, input wire dma_cmd_is32,
  input wire [63:0] dma_cmd_wdata, input wire [7:0] dma_cmd_wmask,
  output wire dma_rsp_valid, input wire dma_rsp_ready,
  output wire dma_rsp_err, output wire [63:0] dma_rsp_rdata,
  output wire mem_cmd_valid, input wire mem_cmd_ready,
  output wire [31:0] mem_cmd_addr, output wire mem_cmd_read,
  output wire [63:0] mem_cmd_wdata, output wire [7:0] mem_cmd_wmask,
  output wire mem_cmd_is32,
  input wire mem_rsp_valid, output wire mem_rsp_ready,
  input wire mem_rsp_err, input wire [63:0] mem_rsp_rdata
);
  reg inflight, owner_dma, owner_high;
  reg prefer_dma, grant_locked, locked_dma;
  wire owner_ready = owner_dma ? dma_rsp_ready : cpu_rsp_ready;
  wire retire = inflight && mem_rsp_valid && owner_ready;
  wire available = !inflight || retire;
  wire select_dma = grant_locked ? locked_dma :
                    (dma_cmd_valid && (!cpu_cmd_valid || prefer_dma));
  wire selected_valid = select_dma ? dma_cmd_valid : cpu_cmd_valid;
  wire accept = mem_cmd_valid && mem_cmd_ready;
  assign mem_cmd_valid = rst_n && available && selected_valid;
  assign mem_cmd_addr = select_dma ? dma_cmd_addr : cpu_cmd_addr;
  assign mem_cmd_read = select_dma ? dma_cmd_read : cpu_cmd_read;
  assign mem_cmd_is32 = select_dma ? dma_cmd_is32 : 1'b1;
  assign mem_cmd_wdata = select_dma ? dma_cmd_wdata :
      (cpu_cmd_addr[2] ? {cpu_cmd_wdata,32'b0} : {32'b0,cpu_cmd_wdata});
  assign mem_cmd_wmask = select_dma ? dma_cmd_wmask :
      (cpu_cmd_addr[2] ? {cpu_cmd_wmask,4'b0} : {4'b0,cpu_cmd_wmask});
  assign cpu_cmd_ready = rst_n && available && !select_dma && mem_cmd_ready;
  assign dma_cmd_ready = rst_n && available && select_dma && mem_cmd_ready;
  assign cpu_rsp_valid = rst_n && inflight && !owner_dma && mem_rsp_valid;
  assign dma_rsp_valid = rst_n && inflight && owner_dma && mem_rsp_valid;
  assign mem_rsp_ready = rst_n && inflight && owner_ready;
  assign cpu_rsp_err = mem_rsp_err;
  assign dma_rsp_err = mem_rsp_err;
  assign cpu_rsp_rdata = owner_high ? mem_rsp_rdata[63:32] : mem_rsp_rdata[31:0];
  assign dma_rsp_rdata = mem_rsp_rdata;
  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
      inflight<=0; owner_dma<=0; owner_high<=0;
      prefer_dma<=0; grant_locked<=0; locked_dma<=0;
    end else begin
      if(retire) inflight<=0;
      if(accept) begin
        inflight<=1; owner_dma<=select_dma; owner_high<=cpu_cmd_addr[2];
        prefer_dma<=!select_dma;
      end
      if(mem_cmd_valid && !mem_cmd_ready) begin
        grant_locked<=1; locked_dma<=select_dma;
      end else if(accept) grant_locked<=0;
    end
  end
endmodule
