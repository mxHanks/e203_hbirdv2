// Peripheral-only CPU/DMA arbiter for the tested external shared ICB SRAM.
// One downstream request may be outstanding. A retiring response can be
// replaced by the next request on the same edge. Neither CPU LSU/BIU logic
// nor its outstanding configuration is modified.
//
// Master 0 is the original CPU sysmem output; master 1 is the independent
// peripheral DMA. Grants are round robin at accepted-command boundaries.
// A selected command remains selected throughout downstream backpressure.
// The downstream endpoint and this arbiter must share reset. A task abort
// must continue accepting its outstanding response; resetting DMA task state
// alone must never discard the downstream owner.
module xu_dma_shared_arb (
  input clk, input rst_n,
  input cpu_cmd_valid, output cpu_cmd_ready,
  input [31:0] cpu_cmd_addr, input cpu_cmd_read,
  input [31:0] cpu_cmd_wdata, input [3:0] cpu_cmd_wmask,
  output cpu_rsp_valid, input cpu_rsp_ready,
  output cpu_rsp_err, output [31:0] cpu_rsp_rdata,
  input dma_cmd_valid, output dma_cmd_ready,
  input [31:0] dma_cmd_addr, input dma_cmd_read,
  input [31:0] dma_cmd_wdata, input [3:0] dma_cmd_wmask,
  output dma_rsp_valid, input dma_rsp_ready,
  output dma_rsp_err, output [31:0] dma_rsp_rdata,
  output mem_cmd_valid, input mem_cmd_ready,
  output [31:0] mem_cmd_addr, output mem_cmd_read,
  output [31:0] mem_cmd_wdata, output [3:0] mem_cmd_wmask,
  input mem_rsp_valid, output mem_rsp_ready,
  input mem_rsp_err, input [31:0] mem_rsp_rdata
);
  reg inflight;
  reg owner_dma;
  reg prefer_dma;
  reg grant_locked;
  reg locked_dma;
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
  assign mem_cmd_wdata = select_dma ? dma_cmd_wdata : cpu_cmd_wdata;
  assign mem_cmd_wmask = select_dma ? dma_cmd_wmask : cpu_cmd_wmask;
  assign cpu_cmd_ready = rst_n && available && !select_dma && mem_cmd_ready;
  assign dma_cmd_ready = rst_n && available && select_dma && mem_cmd_ready;
  assign cpu_rsp_valid = rst_n && inflight && !owner_dma && mem_rsp_valid;
  assign dma_rsp_valid = rst_n && inflight && owner_dma && mem_rsp_valid;
  assign mem_rsp_ready = rst_n && inflight && owner_ready;
  assign cpu_rsp_err = mem_rsp_err;
  assign dma_rsp_err = mem_rsp_err;
  assign cpu_rsp_rdata = mem_rsp_rdata;
  assign dma_rsp_rdata = mem_rsp_rdata;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      inflight <= 1'b0;
      owner_dma <= 1'b0;
      prefer_dma <= 1'b0;
      grant_locked <= 1'b0;
      locked_dma <= 1'b0;
    end else begin
      if (retire) inflight <= 1'b0;
      if (accept) begin
        inflight <= 1'b1;
        owner_dma <= select_dma;
        prefer_dma <= !select_dma;
      end
      if (mem_cmd_valid && !mem_cmd_ready) begin
        grant_locked <= 1'b1;
        locked_dma <= select_dma;
      end else if (accept) begin
        grant_locked <= 1'b0;
      end
    end
  end
endmodule
