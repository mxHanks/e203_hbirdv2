// CPU and DMA are sources of the original 32-bit SystemBus, before mem_fab.
// The platform ICB arbiter retains one response-owner entry. No SRAM side port,
// width adapter, added data queue, or CPU/LSU/BIU control is introduced here.
// Optional simulation loader is another source of this SAME SystemBus only.
module xu_systembus_ingress32 #(parameter TEST_LOADER = 0)(
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
  input load_cmd_valid, output load_cmd_ready,
  input [31:0] load_cmd_addr, input load_cmd_read,
  input [31:0] load_cmd_wdata, input [3:0] load_cmd_wmask,
  output load_rsp_valid, input load_rsp_ready,
  output load_rsp_err, output [31:0] load_rsp_rdata,
  output bus_cmd_valid, input bus_cmd_ready,
  output [31:0] bus_cmd_addr, output bus_cmd_read,
  output [31:0] bus_cmd_wdata, output [3:0] bus_cmd_wmask,
  input bus_rsp_valid, output bus_rsp_ready,
  input bus_rsp_err, input [31:0] bus_rsp_rdata
);
  localparam N = TEST_LOADER ? 3 : 2;
  localparam PW = TEST_LOADER ? 2 : 1;
  reg [1:0] next_source, held_source;
  reg grant_held;
  reg [1:0] selected;
  wire [2:0] request = {TEST_LOADER && load_cmd_valid, dma_cmd_valid, cpu_cmd_valid};
  wire [N-1:0] selected_valid, source_ready, source_rsp_valid, source_rsp_err;
  wire [N*32-1:0] source_rsp_data;
  always @* begin
    selected = 0;
    case (next_source)
      1: if(request[1]) selected=1; else if(request[2]) selected=2; else selected=0;
      2: if(request[2]) selected=2; else if(request[0]) selected=0; else selected=1;
      default: if(request[0]) selected=0; else if(request[1]) selected=1; else selected=2;
    endcase
    if(grant_held) selected=held_source;
  end
  genvar i;
  generate for(i=0;i<N;i=i+1) begin: sources
    assign selected_valid[i]=request[i] && selected==i;
  end endgenerate
  // Keep the selected payload stable if the original mem_fab stalls its input.
  // Sources themselves retain their ICB payload until their command handshake.
  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin next_source<=0;held_source<=0;grant_held<=0;end
    else begin
      if(bus_cmd_valid && !bus_cmd_ready && !grant_held) begin
        held_source<=selected;grant_held<=1;
      end
      if(bus_cmd_valid && bus_cmd_ready) begin
        grant_held<=0;
        next_source<=selected==N-1 ? 0 : selected+1'b1;
      end
    end
  end
  wire [95:0] all_addr={load_cmd_addr,dma_cmd_addr,cpu_cmd_addr};
  wire [95:0] all_wdata={load_cmd_wdata,dma_cmd_wdata,cpu_cmd_wdata};
  wire [11:0] all_wmask={load_cmd_wmask,dma_cmd_wmask,cpu_cmd_wmask};
  wire [2:0] all_read={load_cmd_read,dma_cmd_read,cpu_cmd_read};
  wire [2:0] all_rsp_ready={load_rsp_ready,dma_rsp_ready,cpu_rsp_ready};
  wire [N*32-1:0] source_addr=all_addr[N*32-1:0];
  wire [N*32-1:0] source_wdata=all_wdata[N*32-1:0];
  wire [N*4-1:0] source_wmask=all_wmask[N*4-1:0];
  wire [N-1:0] source_read=all_read[N-1:0];
  wire [N-1:0] source_rsp_ready=all_rsp_ready[N-1:0];
  assign cpu_cmd_ready=rst_n && source_ready[0] && selected==0;
  assign dma_cmd_ready=rst_n && source_ready[1] && selected==1;
  assign cpu_rsp_valid=rst_n && source_rsp_valid[0];
  assign dma_rsp_valid=rst_n && source_rsp_valid[1];
  assign cpu_rsp_err=source_rsp_err[0];assign dma_rsp_err=source_rsp_err[1];
  assign cpu_rsp_rdata=source_rsp_data[31:0];assign dma_rsp_rdata=source_rsp_data[63:32];
  generate if(TEST_LOADER) begin: loader_present
    assign load_cmd_ready=rst_n && source_ready[2] && selected==2;
    assign load_rsp_valid=rst_n && source_rsp_valid[2];
    assign load_rsp_err=source_rsp_err[2];assign load_rsp_rdata=source_rsp_data[95:64];
  end else begin: loader_absent
    assign load_cmd_ready=1'b0;assign load_rsp_valid=1'b0;
    assign load_rsp_err=1'b0;assign load_rsp_rdata=32'b0;
  end endgenerate
  sirv_gnrl_icb_arbt #(
    .AW(32),.DW(32),.USR_W(1),.ARBT_SCHEME(0),
    .FIFO_OUTS_NUM(1),.FIFO_CUT_READY(1),.ALLOW_0CYCL_RSP(0),
    .ARBT_NUM(N),.ARBT_PTR_W(PW)
  ) u_platform_icb_arb (
    .o_icb_cmd_valid(bus_cmd_valid),.o_icb_cmd_ready(bus_cmd_ready),
    .o_icb_cmd_addr(bus_cmd_addr),.o_icb_cmd_read(bus_cmd_read),
    .o_icb_cmd_wdata(bus_cmd_wdata),.o_icb_cmd_wmask(bus_cmd_wmask),
    .o_icb_cmd_burst(),.o_icb_cmd_beat(),.o_icb_cmd_lock(),.o_icb_cmd_excl(),.o_icb_cmd_size(),.o_icb_cmd_usr(),
    .o_icb_rsp_valid(bus_rsp_valid),.o_icb_rsp_ready(bus_rsp_ready),
    .o_icb_rsp_err(bus_rsp_err),.o_icb_rsp_excl_ok(1'b0),.o_icb_rsp_rdata(bus_rsp_rdata),.o_icb_rsp_usr(1'b0),
    .i_bus_icb_cmd_ready(source_ready),.i_bus_icb_cmd_valid(selected_valid & {N{rst_n}}),
    .i_bus_icb_cmd_read(source_read),.i_bus_icb_cmd_addr(source_addr),
    .i_bus_icb_cmd_wdata(source_wdata),.i_bus_icb_cmd_wmask(source_wmask),
    .i_bus_icb_cmd_burst({N*2{1'b0}}),.i_bus_icb_cmd_beat({N*2{1'b0}}),
    .i_bus_icb_cmd_lock({N{1'b0}}),.i_bus_icb_cmd_excl({N{1'b0}}),
    .i_bus_icb_cmd_size({N{2'b10}}),.i_bus_icb_cmd_usr({N{1'b0}}),
    .i_bus_icb_rsp_valid(source_rsp_valid),.i_bus_icb_rsp_ready(source_rsp_ready),
    .i_bus_icb_rsp_err(source_rsp_err),.i_bus_icb_rsp_excl_ok(),
    .i_bus_icb_rsp_rdata(source_rsp_data),.i_bus_icb_rsp_usr(),.clk(clk),.rst_n(rst_n)
  );
endmodule
