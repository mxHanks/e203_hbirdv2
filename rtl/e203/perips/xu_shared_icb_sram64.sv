// Experimental real 512KiB endpoint: 64 depth banks x four 1024x16 1RW macros.
// DMA64 accesses four lanes on ONE edge; CPU32 accesses its selected two lanes.
// Compile with a RAMSP1024X16 model; no behavioral shortcut or storage backdoor.
module xu_shared_icb_sram64 #(
  parameter integer THROTTLE_PERIOD=7,
  parameter integer THROTTLE_RESET_PHASE=2,
  parameter [2:0] XU_EMA=3'b010,
  parameter [1:0] XU_EMAW=2'b00
)(
  input wire clk, input wire rst_n,
  input wire cmd_valid, output wire cmd_ready,
  input wire [31:0] cmd_addr, input wire cmd_read,
  input wire [63:0] cmd_wdata, input wire [7:0] cmd_wmask,
  input wire cmd_is32,
  output reg rsp_valid, input wire rsp_ready,
  output reg rsp_err, output wire [63:0] rsp_rdata
);
  localparam integer PHASE_BITS=(THROTTLE_PERIOD>1)?$clog2(THROTTLE_PERIOD):1;
  reg [PHASE_BITS-1:0] phase;
  wire accept=cmd_valid && cmd_ready;
  wire size_ok=cmd_is32 ? cmd_addr[1:0]==0 : cmd_addr[2:0]==0;
  wire range_ok=cmd_addr>=32'ha0000000 &&
      cmd_addr <= (cmd_is32 ? 32'ha007fffc : 32'ha007fff8);
  wire cpu_mask_ok=cmd_read || !cmd_is32 ||
      (cmd_addr[2] ? cmd_wmask[3:0]==0 : cmd_wmask[7:4]==0);
  wire legal=size_ok && range_ok && cpu_mask_ok;
  wire [5:0] bank=cmd_addr[18:13];
  reg [5:0] read_bank;
  reg response_read, response_is32, response_high;
  wire [63:0] bank_data [0:63];
  wire [63:0] selected_data=bank_data[read_bank];
  wire [63:0] read_data=response_is32 ?
      (response_high ? {selected_data[63:32],32'b0} : {32'b0,selected_data[31:0]}) : selected_data;
  assign cmd_ready=rst_n && (!rsp_valid || rsp_ready) &&
      (THROTTLE_PERIOD==0 || phase!=0);
  assign rsp_rdata=(rsp_err || !response_read) ? 64'b0 : read_data;
  genvar b,l;
  generate for(b=0;b<64;b=b+1) begin: banks
    for(l=0;l<4;l=l+1) begin: lanes
      wire lane_selected=cmd_read ? (!cmd_is32 || (l/2)==cmd_addr[2]) :
          (|cmd_wmask[2*l+:2]);
      wire macro_enable=accept && legal && bank==b && lane_selected;
      wire [15:0] wen=~{{8{cmd_wmask[2*l+1]}},{8{cmd_wmask[2*l]}}};
      RAMSP1024X16 u_macro (
        .Q(bank_data[b][16*l+:16]), .CLK(clk), .CEN(!macro_enable),
        .GWEN(cmd_read), .WEN(wen), .A(cmd_addr[12:3]),
        .D(cmd_wdata[16*l+:16]), .EMA(XU_EMA),.EMAW(XU_EMAW),.RET1N(1'b1)
      );
    end
  end endgenerate
  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
      phase<=PHASE_BITS'(THROTTLE_RESET_PHASE);
      rsp_valid<=0; rsp_err<=0; read_bank<=0;
      response_read<=0; response_is32<=0; response_high<=0;
    end else begin
      if(THROTTLE_PERIOD>0)
        phase <= phase==THROTTLE_PERIOD-1 ? 0 : phase+1'b1;
      if(rsp_valid && rsp_ready) rsp_valid<=0;
      if(accept) begin
        rsp_valid<=1; rsp_err<=!legal;
        response_read<=cmd_read; response_is32<=cmd_is32; response_high<=cmd_addr[2];
        if(cmd_read && legal) read_bank<=bank;
      end
    end
  end
  `ifndef SYNTHESIS
  initial begin
    if(THROTTLE_PERIOD==1 || THROTTLE_PERIOD<0 || THROTTLE_RESET_PHASE<0 ||
       (THROTTLE_PERIOD>0 && THROTTLE_RESET_PHASE>=THROTTLE_PERIOD))
      $fatal(1,"invalid shared64 throttle parameters");
  end
  `endif
endmodule
