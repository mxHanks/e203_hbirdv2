`ifdef XU_MATRIX_DMA_SYSBUS32
`ifdef XU_MATRIX_X_SRAM
`ifdef XU_MATRIX_STORAGE_SRAM
`define C32_FINAL_STORAGE_SELECTED
`endif
`endif
`endif
`ifdef C32_FINAL_STORAGE_SELECTED
// Compliant32 System Bus DMA: four ordered requests within the original fabric buffers.
// The unchanged64PE core receives paired32-bit W responses through internal staging.
// CPU/LSU/BIU controls are unchanged. Bias physical commit overlaps W fetch.
module xu_matrix_dma_sysbus32 (
  input wire clk, input wire rst_n,
  input wire cfg_cmd_valid, output wire cfg_cmd_ready,
  input wire [31:0] cfg_cmd_addr, input wire cfg_cmd_read,
  input wire [31:0] cfg_cmd_wdata, input wire [3:0] cfg_cmd_wmask,
  output reg cfg_rsp_valid, input wire cfg_rsp_ready,
  output reg cfg_rsp_err, output reg [31:0] cfg_rsp_rdata,
  output wire mem_cmd_valid, input wire mem_cmd_ready,
  output wire [31:0] mem_cmd_addr, output wire mem_cmd_read,
  output wire [31:0] mem_cmd_wdata, output wire [3:0] mem_cmd_wmask,
  input wire mem_rsp_valid, output wire mem_rsp_ready,
  input wire mem_rsp_err, input wire [31:0] mem_rsp_rdata
);
  localparam IDLE=0,CLEAR=1,WB=2,RUN=3,ABORT=4;
  localparam KW=0,KB=1,KX=3,KY=4;
  reg [2:0] state;
  reg busy,done,error,clear_done,wide_rows,aborting;
  reg [31:0] error_code,x_base,w_base,b_base,y_base,run_x,run_w,run_b,run_y;
  reg [31:0] cycle_count,clear_cycles,read_count,write_count,pop_count,mac_count;
  reg [31:0] clear_write_count,cmd_stall_count,rsp_wait_count,wb_cycles,run_cycles;
  reg [13:0] w_issue,w_complete;
  reg [31:0] w_low,bias_low;
  reg [9:0] b_issue;
  reg [8:0] b_complete;
  reg bias_busy,bias_valid,bias_wait;
  reg [7:0] bias_addr;
  reg [39:0] bias_data;
  reg x_filling,x_ready,x_set,run_set,tile_running;
  reg [6:0] x_row,run_row,issued_rows;
  reg [9:0] x_issue,x_limit;
  reg y_active;
  reg [3:0] y_issue;
  // Only request metadata is buffered here; all data still traverses the
  // original 32-bit mem_fab. Its command/response queues and single target
  // outstanding configuration are unchanged. Responses retire in issue order.
  localparam REQUEST_DEPTH=4;
  reg [2:0] pending_count;
  reg [1:0] pending_read_ptr,pending_write_ptr;
  reg [2:0] request_kind [0:REQUEST_DEPTH-1];
  reg [13:0] request_index [0:REQUEST_DEPTH-1];
  wire pending=pending_count!=0;
  wire [2:0] pending_kind=pending?request_kind[pending_read_ptr]:3'b0;
  wire [13:0] pending_index=pending?request_index[pending_read_ptr]:14'b0;
  reg locked;
  reg [2:0] lock_kind,want_kind;
  reg [13:0] lock_index,want_index;
  reg [31:0] lock_addr,want_addr;
  reg [31:0] lock_data,want_data;
  reg lock_read,want_read,want;
  wire core_active,core_done,core_clear_ready,core_clear_done,core_clearing;
  wire core_out_valid,core_compute_start,core_weight_read,core_mac_step,core_fifo_wait;
  wire core_clear_w,core_clear_x,core_bias_ready,core_bias_rsp,core_bias_error;
  wire [2:0] core_out_row,core_out_column;
  wire core_clear_result;
  wire [47:0] core_out_issue;
  wire [4:0] core_out_group;
  wire [47:0] core_out[0:7];
  wire cfg_fire=cfg_cmd_valid&&cfg_cmd_ready;
  wire cfg_hit=cfg_cmd_addr[11:7]==5'd6;
  wire [6:0] cfg_off=cfg_cmd_addr[6:0];
  wire rsp_fire=mem_rsp_valid&&mem_rsp_ready;
  wire fault=!aborting&&((rsp_fire&&mem_rsp_err)||(bias_wait&&core_bias_rsp&&core_bias_error));
  wire [2:0] offer_kind=locked?lock_kind:want_kind;
  wire [13:0] offer_index=locked?lock_index:want_index;
  assign mem_cmd_valid=rst_n&&(locked||(!aborting&&!fault&&want&&(pending_count<REQUEST_DEPTH||rsp_fire)));
  assign mem_cmd_addr=locked?lock_addr:want_addr;
  assign mem_cmd_read=locked?lock_read:want_read;
  assign mem_cmd_wdata=locked?lock_data:want_data;
  assign mem_cmd_wmask=4'hf;
  assign mem_rsp_ready=rst_n&&pending;
  wire cmd_fire=mem_cmd_valid&&mem_cmd_ready;
  assign cfg_cmd_ready=rst_n&&(!cfg_rsp_valid||cfg_rsp_ready);
  function automatic fits(input [31:0] base,input [31:0] bytes);
    fits=base>=32'ha0000000&&base<=32'ha0080000-bytes&&base[1:0]==0;
  endfunction
  function automatic overlap(input [31:0] a,alen,b,blen);
    overlap=(a<b+blen)&&(b<a+alen);
  endfunction
  function automatic [3:0] rows_at(input [6:0] row,input wide);
    rows_at=wide?((row<=91)?4'd8:4'(99-row)):4'd1;
  endfunction
  wire descriptor_valid=fits(x_base,25344)&&fits(w_base,32768)&&fits(b_base,2048)&&
    fits(y_base,101376)&&!overlap(y_base,101376,x_base,25344)&&
    !overlap(y_base,101376,w_base,32768)&&!overlap(y_base,101376,b_base,2048);
  wire clear_request=cfg_fire&&cfg_hit&&!cfg_cmd_read&&cfg_off==7'h04&&
    cfg_cmd_wmask==4'hf&&cfg_cmd_wdata==1&&!busy&&!pending&&!locked&&core_clear_ready;
  wire start_request=cfg_fire&&cfg_hit&&!cfg_cmd_read&&cfg_off==7'h04&&
    cfg_cmd_wmask==4'hf&&(cfg_cmd_wdata==2||cfg_cmd_wdata==32'h802)&&
    !busy&&!pending&&!locked&&clear_done&&descriptor_valid;
  function automatic [31:0] rne(input [47:0] value);
    reg signed [47:0] q;
    begin q=$signed(value)>>>9;
      if(value[8:0]>256||(value[8:0]==256&&q[0]))q=q+48'sd1;
      rne=q[31:0]; end
  endfunction
  always @* begin
    want=0;want_kind=KW;want_index=0;want_addr=0;want_data=0;want_read=1;
    if(state==WB)begin
      if(!bias_busy&&b_issue<512)begin
        want=1;want_kind=KB;want_index={4'b0,b_issue};
        want_addr=run_b+{20'b0,b_issue,2'b0};
      end else if(w_issue<8192)begin
        want=1;want_kind=KW;want_index=w_issue;
        want_addr=run_w+{16'b0,w_issue,2'b0};
      end
    end else if(state==RUN)begin
      // The result macro Q row remains owned by this stream until column7
      // response retirement. Select each accepted scalar with the issue index;
      // the core's independent output column advances only on responses.
      if(core_out_valid&&y_issue<8)begin
        want=1;want_kind=KY;want_index={10'b0,y_issue};want_read=0;
        want_addr=run_y+(({25'b0,run_row}+{29'b0,core_out_row})<<10)+
          {22'b0,core_out_group,5'b0}+{26'b0,y_issue,2'b0};
        want_data=rne(core_out_issue);
      end else if(x_filling&&x_issue<x_limit)begin
        want=1;want_kind=KX;want_index={4'b0,x_issue};
        want_addr=run_x+({25'b0,x_row}<<8)+{20'b0,x_issue,2'b0};
      end
    end
  end
  wire load_ok=rsp_fire&&!mem_rsp_err&&!aborting&&!fault;
  wire core_pop=state==RUN&&load_ok&&pending_kind==KY;
  wire core_start=state==RUN&&!aborting&&!fault&&x_ready&&!tile_running&&!y_active&&
    !pending&&!locked&&!core_active;
  wire [3:0] start_rows=rows_at(x_row,wide_rows);
  wire [7:0] next_row={1'b0,x_row}+{4'b0,start_rows};
  LinearTileDmaCore u_core (
    .clock(clk),.reset(!rst_n),.io_start(core_start),.io_validRows(start_rows),
    .io_abort(aborting||(rsp_fire&&mem_rsp_err)),.io_xRunSet(x_set),.io_active(core_active),.io_done(core_done),
    .io_xLoadWen(load_ok&&pending_kind==KX),.io_xLoadRow(pending_index[8:6]),
    .io_xLoadSet(x_set),.io_xLoadAddr(pending_index[5:0]),
    .io_xLoadData(mem_rsp_rdata[15:0]),.io_xLoadPacked(1'b0),.io_xLoadDataHi(16'b0),
    .io_wLoadWen(load_ok&&pending_kind==KW&&pending_index[0]),.io_wLoadBank({pending_index[1],2'b0}),
    .io_wLoadAddr({pending_index[6:2],pending_index[12:7]}),
    .io_wLoadData_0(w_low[15:0]),.io_wLoadData_1(w_low[31:16]),
    .io_wLoadData_2(mem_rsp_rdata[15:0]),.io_wLoadData_3(mem_rsp_rdata[31:16]),
    .io_biasHostValid(bias_valid&&!aborting&&!fault),.io_biasHostReady(core_bias_ready),
    .io_biasHostAddr(bias_addr),.io_biasHostData(bias_data),
    .io_biasRspValid(core_bias_rsp),.io_biasRspError(core_bias_error),
    .io_clearValid(clear_request),.io_clearReady(core_clear_ready),
    .io_clearDone(core_clear_done),.io_clearing(core_clearing),
    .io_outValid(core_out_valid),.io_outPop(core_pop),.io_outGroup(core_out_group),.io_outRow(core_out_row),
    .io_outColumn(core_out_column),.io_outIssueColumn(y_issue[2:0]),.io_outIssueResult(core_out_issue),
    .io_outResult_0(core_out[0]),.io_outResult_1(core_out[1]),
    .io_outResult_2(core_out[2]),.io_outResult_3(core_out[3]),
    .io_outResult_4(core_out[4]),.io_outResult_5(core_out[5]),
    .io_outResult_6(core_out[6]),.io_outResult_7(core_out[7]),
    .io_computeStart(core_compute_start),.io_weightRead(core_weight_read),
    .io_macStep(core_mac_step),.io_fifoFullWait(core_fifo_wait),
    .io_clearWeightWrite(core_clear_w),.io_clearXWrite(core_clear_x),.io_clearResultWrite(core_clear_result)
  );
  integer queue_slot;
  always @(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
      state<=IDLE;busy<=0;done<=0;error<=0;clear_done<=0;wide_rows<=0;aborting<=0;error_code<=0;
      x_base<=32'ha0000000;w_base<=32'ha0008000;b_base<=32'ha0018000;y_base<=32'ha0020000;
      run_x<=0;run_w<=0;run_b<=0;run_y<=0;
      cycle_count<=0;clear_cycles<=0;read_count<=0;write_count<=0;pop_count<=0;mac_count<=0;
      clear_write_count<=0;cmd_stall_count<=0;rsp_wait_count<=0;wb_cycles<=0;run_cycles<=0;
      w_issue<=0;w_complete<=0;b_issue<=0;b_complete<=0;w_low<=0;bias_low<=0;
      bias_busy<=0;bias_valid<=0;bias_wait<=0;bias_addr<=0;bias_data<=0;
      x_filling<=0;x_ready<=0;x_set<=0;run_set<=0;tile_running<=0;
      x_row<=0;run_row<=0;issued_rows<=0;x_issue<=0;x_limit<=0;
      y_active<=0;y_issue<=0;
      pending_count<=0;pending_read_ptr<=0;pending_write_ptr<=0;
      for(queue_slot=0;queue_slot<REQUEST_DEPTH;queue_slot=queue_slot+1)begin
        request_kind[queue_slot]<=0;request_index[queue_slot]<=0;
      end
      locked<=0;lock_kind<=0;lock_index<=0;lock_addr<=0;lock_data<=0;lock_read<=0;
      cfg_rsp_valid<=0;cfg_rsp_err<=0;cfg_rsp_rdata<=0;
    end else begin
      if(cfg_rsp_valid&&cfg_rsp_ready)cfg_rsp_valid<=0;
      if(busy&&state!=CLEAR)cycle_count<=cycle_count+1;
      if(state==CLEAR)clear_cycles<=clear_cycles+1;
      if(state==WB)wb_cycles<=wb_cycles+1;
      if(state==RUN)run_cycles<=run_cycles+1;
      if(core_mac_step)mac_count<=mac_count+1;
      if(state==CLEAR)clear_write_count<=clear_write_count+(core_clear_w?32'd16:0)+(core_clear_x?32'd16:0)+(core_clear_result?32'd21:0);
      if(mem_cmd_valid&&!mem_cmd_ready)cmd_stall_count<=cmd_stall_count+1;
      if(pending&&!mem_rsp_valid)rsp_wait_count<=rsp_wait_count+1;
      if(mem_cmd_valid&&!mem_cmd_ready&&!locked)begin
        locked<=1;lock_kind<=want_kind;lock_index<=want_index;
        lock_addr<=want_addr;lock_data<=want_data;lock_read<=want_read;
      end
      case({cmd_fire,rsp_fire})
        2'b10:pending_count<=pending_count+1'b1;
        2'b01:pending_count<=pending_count-1'b1;
        default:begin end
      endcase
      if(rsp_fire)begin
        request_kind[pending_read_ptr]<=0;request_index[pending_read_ptr]<=0;
        pending_read_ptr<=pending_read_ptr+1'b1;
      end
      if(cmd_fire)begin
        locked<=0;
        request_kind[pending_write_ptr]<=offer_kind;request_index[pending_write_ptr]<=offer_index;
        pending_write_ptr<=pending_write_ptr+1'b1;
        if(mem_cmd_read)read_count<=read_count+1;else write_count<=write_count+1;
        if(!aborting&&!fault)case(offer_kind)
          KW:w_issue<=w_issue+1;
          KB:begin b_issue<=b_issue+1;if(offer_index[0])bias_busy<=1;end
          KX:x_issue<=x_issue+1;
          KY:begin y_issue<=y_issue+1;y_active<=1;end
        endcase
      end
      if(load_ok)case(pending_kind)
        KW:begin w_complete<=w_complete+1;if(!pending_index[0])w_low<=mem_rsp_rdata;end
        KB:begin
          if(!pending_index[0])bias_low<=mem_rsp_rdata;
          else begin bias_data<={mem_rsp_rdata[7:0],bias_low};bias_addr<=pending_index[8:1];bias_valid<=1;end
        end
        KX:if(pending_index+1=={4'b0,x_limit})begin x_filling<=0;x_ready<=1;end
        KY:if(pending_index==7)begin y_active<=0;y_issue<=0;end
      endcase
      if(bias_valid&&core_bias_ready&&!aborting&&!fault)begin bias_valid<=0;bias_wait<=1;end
      if(bias_wait&&core_bias_rsp&&!aborting&&!fault)begin
        bias_wait<=0;bias_busy<=0;b_complete<=b_complete+1;
      end
      if(state==WB&&w_complete==8192&&b_complete==256&&!pending&&!locked&&!bias_busy&&!aborting)begin
        state<=RUN;x_filling<=1;x_ready<=0;x_set<=0;x_row<=0;x_issue<=0;
        x_limit<=wide_rows?10'd512:10'd64;issued_rows<=0;tile_running<=0;
      end
      if(core_start)begin
        run_row<=x_row;run_set<=x_set;tile_running<=1;x_ready<=0;
        issued_rows<=next_row[6:0];
        if(next_row<99)begin
          x_row<=next_row[6:0];x_set<=!x_set;x_filling<=1;x_issue<=0;
          x_limit<={6'b0,rows_at(next_row[6:0],wide_rows)}<<6;
        end
      end
      if(core_done)tile_running<=0;
      if(core_pop&&core_out_column==7)pop_count<=pop_count+1;
      if(state==RUN&&issued_rows==99&&!tile_running&&!core_active&&!y_active&&
        !pending&&!locked&&!core_out_valid&&!aborting)begin state<=IDLE;busy<=0;done<=1;end
      if(state==CLEAR&&core_clear_done)begin state<=IDLE;busy<=0;clear_done<=1;end
      if(fault)begin error<=1;error_code<=(rsp_fire&&mem_rsp_err)?1:2;aborting<=1;state<=ABORT;done<=0;end
      if(state==ABORT&&!pending&&!locked)begin
        state<=IDLE;busy<=0;aborting<=0;x_filling<=0;x_ready<=0;y_active<=0;tile_running<=0;
        bias_valid<=0;bias_wait<=0;bias_busy<=0;
      end
      if(cfg_fire)begin
        cfg_rsp_valid<=1;cfg_rsp_err<=0;cfg_rsp_rdata<=0;
        if(!cfg_hit||cfg_cmd_addr[1:0]!=0)cfg_rsp_err<=1;
        else if(cfg_cmd_read)case(cfg_off)
          7'h00:cfg_rsp_rdata<=32'h444d4131;
          7'h08:cfg_rsp_rdata<={28'b0,clear_done,error,done,busy};
          7'h0c:cfg_rsp_rdata<=error_code;
          7'h10:cfg_rsp_rdata<=x_base;7'h14:cfg_rsp_rdata<=w_base;
          7'h18:cfg_rsp_rdata<=b_base;7'h1c:cfg_rsp_rdata<=y_base;
          7'h20:cfg_rsp_rdata<=cycle_count;7'h24:cfg_rsp_rdata<=clear_cycles;
          7'h28:cfg_rsp_rdata<=read_count;7'h2c:cfg_rsp_rdata<=write_count;
          7'h30:cfg_rsp_rdata<=pop_count;7'h34:cfg_rsp_rdata<=mac_count;
          7'h38:cfg_rsp_rdata<=clear_write_count;7'h3c:cfg_rsp_rdata<=cmd_stall_count;
          7'h40:cfg_rsp_rdata<=rsp_wait_count;7'h44:cfg_rsp_rdata<=wb_cycles;
          7'h48:cfg_rsp_rdata<=run_cycles;7'h4c:cfg_rsp_rdata<=32'h00082001;
          7'h50:cfg_rsp_rdata<=run_x;7'h54:cfg_rsp_rdata<=run_w;
          7'h58:cfg_rsp_rdata<=run_b;7'h5c:cfg_rsp_rdata<=run_y;
          default:cfg_rsp_err<=1;
        endcase
        else if(cfg_cmd_wmask!=4'hf)cfg_rsp_err<=1;
        else case(cfg_off)
          7'h04:begin
            if(clear_request)begin
              state<=CLEAR;busy<=1;done<=0;error<=0;error_code<=0;clear_done<=0;aborting<=0;
              clear_cycles<=0;clear_write_count<=0;x_filling<=0;x_ready<=0;tile_running<=0;
              y_active<=0;bias_data<=0;bias_addr<=0;bias_busy<=0;bias_valid<=0;bias_wait<=0;
              lock_addr<=0;lock_data<=0;lock_kind<=0;lock_index<=0;lock_read<=0;
              pending_count<=0;pending_read_ptr<=0;pending_write_ptr<=0;
              for(queue_slot=0;queue_slot<REQUEST_DEPTH;queue_slot=queue_slot+1)begin
                request_kind[queue_slot]<=0;request_index[queue_slot]<=0;
              end
              y_issue<=0;
              run_x<=0;run_w<=0;run_b<=0;run_y<=0;run_row<=0;run_set<=0;
              x_row<=0;x_set<=0;x_issue<=0;x_limit<=0;issued_rows<=0;
              w_issue<=0;w_complete<=0;b_issue<=0;b_complete<=0;w_low<=0;bias_low<=0;
            end else if(start_request)begin
              state<=WB;busy<=1;done<=0;clear_done<=0;wide_rows<=cfg_cmd_wdata[11];
              run_x<=x_base;run_w<=w_base;run_b<=b_base;run_y<=y_base;
              cycle_count<=0;read_count<=0;write_count<=0;pop_count<=0;mac_count<=0;
              cmd_stall_count<=0;rsp_wait_count<=0;wb_cycles<=0;run_cycles<=0;
              w_issue<=0;w_complete<=0;b_issue<=0;b_complete<=0;w_low<=0;bias_low<=0;bias_busy<=0;bias_valid<=0;bias_wait<=0;
              issued_rows<=0;y_active<=0;y_issue<=0;aborting<=0;
            end else cfg_rsp_err<=1;
          end
          7'h10:if(!busy)x_base<=cfg_cmd_wdata;else cfg_rsp_err<=1;
          7'h14:if(!busy)w_base<=cfg_cmd_wdata;else cfg_rsp_err<=1;
          7'h18:if(!busy)b_base<=cfg_cmd_wdata;else cfg_rsp_err<=1;
          7'h1c:if(!busy)y_base<=cfg_cmd_wdata;else cfg_rsp_err<=1;
          default:cfg_rsp_err<=1;
        endcase
      end
    end
  end
`ifndef SYNTHESIS
  always @(posedge clk)if(rst_n)begin
    assert(pending_count<=REQUEST_DEPTH)else $fatal(1,"DMA metadata overflow");
    if(core_pop)begin
      assert(core_out_valid)else $fatal(1,"Y response without held result row");
      assert(pending_index=={11'b0,core_out_column})else $fatal(1,"Y issue/retire index mismatch");
      assert(y_issue>core_out_column)else $fatal(1,"Y retirement before accepted issue");
    end
    if(cmd_fire&&offer_kind==KY&&!aborting&&!fault)begin
      assert(core_out_valid&&offer_index<8)else $fatal(1,"Y issue without held macro Q row");
    end
  end
`endif
endmodule

`else
// Compliant32 System Bus DMA: four ordered requests within the original fabric buffers.
// The unchanged64PE core receives paired32-bit W responses through internal staging.
// CPU/LSU/BIU controls are unchanged. Bias physical commit overlaps W fetch.
module xu_matrix_dma_sysbus32 (
  input wire clk, input wire rst_n,
  input wire cfg_cmd_valid, output wire cfg_cmd_ready,
  input wire [31:0] cfg_cmd_addr, input wire cfg_cmd_read,
  input wire [31:0] cfg_cmd_wdata, input wire [3:0] cfg_cmd_wmask,
  output reg cfg_rsp_valid, input wire cfg_rsp_ready,
  output reg cfg_rsp_err, output reg [31:0] cfg_rsp_rdata,
  output wire mem_cmd_valid, input wire mem_cmd_ready,
  output wire [31:0] mem_cmd_addr, output wire mem_cmd_read,
  output wire [31:0] mem_cmd_wdata, output wire [3:0] mem_cmd_wmask,
  input wire mem_rsp_valid, output wire mem_rsp_ready,
  input wire mem_rsp_err, input wire [31:0] mem_rsp_rdata
);
  localparam IDLE=0,CLEAR=1,WB=2,RUN=3,ABORT=4;
  localparam KW=0,KB=1,KX=3,KY=4;
  reg [2:0] state;
  reg busy,done,error,clear_done,wide_rows,aborting;
  reg [31:0] error_code,x_base,w_base,b_base,y_base,run_x,run_w,run_b,run_y;
  reg [31:0] cycle_count,clear_cycles,read_count,write_count,pop_count,mac_count;
  reg [31:0] clear_write_count,cmd_stall_count,rsp_wait_count,wb_cycles,run_cycles;
  reg [13:0] w_issue,w_complete;
  reg [31:0] w_low,bias_low;
  reg [9:0] b_issue;
  reg [8:0] b_complete;
  reg bias_busy,bias_valid,bias_wait;
  reg [7:0] bias_addr;
  reg [39:0] bias_data;
  reg x_filling,x_ready,x_set,run_set,tile_running;
  reg [6:0] x_row,run_row,issued_rows;
  reg [9:0] x_issue,x_limit;
  reg y_active;
  reg [255:0] y_payload;
  reg [31:0] y_address;
  reg [3:0] y_issue;
  // Only request metadata is buffered here; all data still traverses the
  // original 32-bit mem_fab. Its command/response queues and single target
  // outstanding configuration are unchanged. Responses retire in issue order.
  localparam REQUEST_DEPTH=4;
  reg [2:0] pending_count;
  reg [1:0] pending_read_ptr,pending_write_ptr;
  reg [2:0] request_kind [0:REQUEST_DEPTH-1];
  reg [13:0] request_index [0:REQUEST_DEPTH-1];
  wire pending=pending_count!=0;
  wire [2:0] pending_kind=pending?request_kind[pending_read_ptr]:3'b0;
  wire [13:0] pending_index=pending?request_index[pending_read_ptr]:14'b0;
  reg locked;
  reg [2:0] lock_kind,want_kind;
  reg [13:0] lock_index,want_index;
  reg [31:0] lock_addr,want_addr;
  reg [31:0] lock_data,want_data;
  reg lock_read,want_read,want;
  wire core_active,core_done,core_clear_ready,core_clear_done,core_clearing;
  wire core_out_valid,core_compute_start,core_weight_read,core_mac_step,core_fifo_wait;
  wire core_clear_w,core_clear_x,core_bias_ready,core_bias_rsp,core_bias_error;
  wire [2:0] core_out_row;
  wire [4:0] core_out_group;
  wire [47:0] core_out[0:7];
  wire cfg_fire=cfg_cmd_valid&&cfg_cmd_ready;
  wire cfg_hit=cfg_cmd_addr[11:7]==5'd6;
  wire [6:0] cfg_off=cfg_cmd_addr[6:0];
  wire rsp_fire=mem_rsp_valid&&mem_rsp_ready;
  wire fault=!aborting&&((rsp_fire&&mem_rsp_err)||(bias_wait&&core_bias_rsp&&core_bias_error));
  wire [2:0] offer_kind=locked?lock_kind:want_kind;
  wire [13:0] offer_index=locked?lock_index:want_index;
  assign mem_cmd_valid=rst_n&&(locked||(!aborting&&!fault&&want&&(pending_count<REQUEST_DEPTH||rsp_fire)));
  assign mem_cmd_addr=locked?lock_addr:want_addr;
  assign mem_cmd_read=locked?lock_read:want_read;
  assign mem_cmd_wdata=locked?lock_data:want_data;
  assign mem_cmd_wmask=4'hf;
  assign mem_rsp_ready=rst_n&&pending;
  wire cmd_fire=mem_cmd_valid&&mem_cmd_ready;
  assign cfg_cmd_ready=rst_n&&(!cfg_rsp_valid||cfg_rsp_ready);
  function automatic fits(input [31:0] base,input [31:0] bytes);
    fits=base>=32'ha0000000&&base<=32'ha0080000-bytes&&base[1:0]==0;
  endfunction
  function automatic overlap(input [31:0] a,alen,b,blen);
    overlap=(a<b+blen)&&(b<a+alen);
  endfunction
  function automatic [3:0] rows_at(input [6:0] row,input wide);
    rows_at=wide?((row<=91)?4'd8:4'(99-row)):4'd1;
  endfunction
  wire descriptor_valid=fits(x_base,25344)&&fits(w_base,32768)&&fits(b_base,2048)&&
    fits(y_base,101376)&&!overlap(y_base,101376,x_base,25344)&&
    !overlap(y_base,101376,w_base,32768)&&!overlap(y_base,101376,b_base,2048);
  wire clear_request=cfg_fire&&cfg_hit&&!cfg_cmd_read&&cfg_off==7'h04&&
    cfg_cmd_wmask==4'hf&&cfg_cmd_wdata==1&&!busy&&!pending&&!locked&&core_clear_ready;
  wire start_request=cfg_fire&&cfg_hit&&!cfg_cmd_read&&cfg_off==7'h04&&
    cfg_cmd_wmask==4'hf&&(cfg_cmd_wdata==2||cfg_cmd_wdata==32'h802)&&
    !busy&&!pending&&!locked&&clear_done&&descriptor_valid;
  function automatic [31:0] rne(input [47:0] value);
    reg signed [47:0] q;
    begin q=$signed(value)>>>9;
      if(value[8:0]>256||(value[8:0]==256&&q[0]))q=q+48'sd1;
      rne=q[31:0]; end
  endfunction
  always @* begin
    want=0;want_kind=KW;want_index=0;want_addr=0;want_data=0;want_read=1;
    if(state==WB)begin
      if(!bias_busy&&b_issue<512)begin
        want=1;want_kind=KB;want_index={4'b0,b_issue};
        want_addr=run_b+{20'b0,b_issue,2'b0};
      end else if(w_issue<8192)begin
        want=1;want_kind=KW;want_index=w_issue;
        want_addr=run_w+{16'b0,w_issue,2'b0};
      end
    end else if(state==RUN)begin
      if(y_active&&y_issue<8)begin
        want=1;want_kind=KY;want_index={10'b0,y_issue};want_read=0;
        want_addr=y_address+{26'b0,y_issue,2'b0};want_data=y_payload[32*y_issue+:32];
      end else if(x_filling&&x_issue<x_limit)begin
        want=1;want_kind=KX;want_index={4'b0,x_issue};
        want_addr=run_x+({25'b0,x_row}<<8)+{20'b0,x_issue,2'b0};
      end
    end
  end
  wire load_ok=rsp_fire&&!mem_rsp_err&&!aborting&&!fault;
  wire core_pop=state==RUN&&!aborting&&!fault&&!y_active&&core_out_valid;
  wire core_start=state==RUN&&!aborting&&!fault&&x_ready&&!tile_running&&!y_active&&
    !pending&&!locked&&!core_active;
  wire [3:0] start_rows=rows_at(x_row,wide_rows);
  wire [7:0] next_row={1'b0,x_row}+{4'b0,start_rows};
  LinearTileDmaCore u_core (
    .clock(clk),.reset(!rst_n),.io_start(core_start),.io_validRows(start_rows),
    .io_abort(aborting||(rsp_fire&&mem_rsp_err)),.io_xRunSet(x_set),.io_active(core_active),.io_done(core_done),
    .io_xLoadWen(load_ok&&pending_kind==KX),.io_xLoadRow(pending_index[8:6]),
    .io_xLoadSet(x_set),.io_xLoadAddr(pending_index[5:0]),
    .io_xLoadData(mem_rsp_rdata[15:0]),.io_xLoadPacked(1'b0),.io_xLoadDataHi(16'b0),
    .io_wLoadWen(load_ok&&pending_kind==KW&&pending_index[0]),.io_wLoadBank({pending_index[1],2'b0}),
    .io_wLoadAddr({pending_index[6:2],pending_index[12:7]}),
    .io_wLoadData_0(w_low[15:0]),.io_wLoadData_1(w_low[31:16]),
    .io_wLoadData_2(mem_rsp_rdata[15:0]),.io_wLoadData_3(mem_rsp_rdata[31:16]),
    .io_biasHostValid(bias_valid&&!aborting&&!fault),.io_biasHostReady(core_bias_ready),
    .io_biasHostAddr(bias_addr),.io_biasHostData(bias_data),
    .io_biasRspValid(core_bias_rsp),.io_biasRspError(core_bias_error),
    .io_clearValid(clear_request),.io_clearReady(core_clear_ready),
    .io_clearDone(core_clear_done),.io_clearing(core_clearing),
    .io_outValid(core_out_valid),.io_outPop(core_pop),.io_outGroup(core_out_group),.io_outRow(core_out_row),
    .io_outResult_0(core_out[0]),.io_outResult_1(core_out[1]),
    .io_outResult_2(core_out[2]),.io_outResult_3(core_out[3]),
    .io_outResult_4(core_out[4]),.io_outResult_5(core_out[5]),
    .io_outResult_6(core_out[6]),.io_outResult_7(core_out[7]),
    .io_computeStart(core_compute_start),.io_weightRead(core_weight_read),
    .io_macStep(core_mac_step),.io_fifoFullWait(core_fifo_wait),
    .io_clearWeightWrite(core_clear_w),.io_clearXWrite(core_clear_x)
  );
  integer lane,queue_slot;
  always @(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
      state<=IDLE;busy<=0;done<=0;error<=0;clear_done<=0;wide_rows<=0;aborting<=0;error_code<=0;
      x_base<=32'ha0000000;w_base<=32'ha0008000;b_base<=32'ha0018000;y_base<=32'ha0020000;
      run_x<=0;run_w<=0;run_b<=0;run_y<=0;
      cycle_count<=0;clear_cycles<=0;read_count<=0;write_count<=0;pop_count<=0;mac_count<=0;
      clear_write_count<=0;cmd_stall_count<=0;rsp_wait_count<=0;wb_cycles<=0;run_cycles<=0;
      w_issue<=0;w_complete<=0;b_issue<=0;b_complete<=0;w_low<=0;bias_low<=0;
      bias_busy<=0;bias_valid<=0;bias_wait<=0;bias_addr<=0;bias_data<=0;
      x_filling<=0;x_ready<=0;x_set<=0;run_set<=0;tile_running<=0;
      x_row<=0;run_row<=0;issued_rows<=0;x_issue<=0;x_limit<=0;
      y_active<=0;y_payload<=0;y_address<=0;y_issue<=0;
      pending_count<=0;pending_read_ptr<=0;pending_write_ptr<=0;
      for(queue_slot=0;queue_slot<REQUEST_DEPTH;queue_slot=queue_slot+1)begin
        request_kind[queue_slot]<=0;request_index[queue_slot]<=0;
      end
      locked<=0;lock_kind<=0;lock_index<=0;lock_addr<=0;lock_data<=0;lock_read<=0;
      cfg_rsp_valid<=0;cfg_rsp_err<=0;cfg_rsp_rdata<=0;
    end else begin
      if(cfg_rsp_valid&&cfg_rsp_ready)cfg_rsp_valid<=0;
      if(busy&&state!=CLEAR)cycle_count<=cycle_count+1;
      if(state==CLEAR)clear_cycles<=clear_cycles+1;
      if(state==WB)wb_cycles<=wb_cycles+1;
      if(state==RUN)run_cycles<=run_cycles+1;
      if(core_mac_step)mac_count<=mac_count+1;
      if(state==CLEAR)clear_write_count<=clear_write_count+(core_clear_w?32'd16:0)+(core_clear_x?32'd16:0);
      if(mem_cmd_valid&&!mem_cmd_ready)cmd_stall_count<=cmd_stall_count+1;
      if(pending&&!mem_rsp_valid)rsp_wait_count<=rsp_wait_count+1;
      if(mem_cmd_valid&&!mem_cmd_ready&&!locked)begin
        locked<=1;lock_kind<=want_kind;lock_index<=want_index;
        lock_addr<=want_addr;lock_data<=want_data;lock_read<=want_read;
      end
      case({cmd_fire,rsp_fire})
        2'b10:pending_count<=pending_count+1'b1;
        2'b01:pending_count<=pending_count-1'b1;
        default:begin end
      endcase
      if(rsp_fire)begin
        request_kind[pending_read_ptr]<=0;request_index[pending_read_ptr]<=0;
        pending_read_ptr<=pending_read_ptr+1'b1;
      end
      if(cmd_fire)begin
        locked<=0;
        request_kind[pending_write_ptr]<=offer_kind;request_index[pending_write_ptr]<=offer_index;
        pending_write_ptr<=pending_write_ptr+1'b1;
        if(mem_cmd_read)read_count<=read_count+1;else write_count<=write_count+1;
        if(!aborting&&!fault)case(offer_kind)
          KW:w_issue<=w_issue+1;
          KB:begin b_issue<=b_issue+1;if(offer_index[0])bias_busy<=1;end
          KX:x_issue<=x_issue+1;
          KY:y_issue<=y_issue+1;
        endcase
      end
      if(load_ok)case(pending_kind)
        KW:begin w_complete<=w_complete+1;if(!pending_index[0])w_low<=mem_rsp_rdata;end
        KB:begin
          if(!pending_index[0])bias_low<=mem_rsp_rdata;
          else begin bias_data<={mem_rsp_rdata[7:0],bias_low};bias_addr<=pending_index[8:1];bias_valid<=1;end
        end
        KX:if(pending_index+1=={4'b0,x_limit})begin x_filling<=0;x_ready<=1;end
        KY:if(pending_index==7)y_active<=0;
      endcase
      if(bias_valid&&core_bias_ready&&!aborting&&!fault)begin bias_valid<=0;bias_wait<=1;end
      if(bias_wait&&core_bias_rsp&&!aborting&&!fault)begin
        bias_wait<=0;bias_busy<=0;b_complete<=b_complete+1;
      end
      if(state==WB&&w_complete==8192&&b_complete==256&&!pending&&!locked&&!bias_busy&&!aborting)begin
        state<=RUN;x_filling<=1;x_ready<=0;x_set<=0;x_row<=0;x_issue<=0;
        x_limit<=wide_rows?10'd512:10'd64;issued_rows<=0;tile_running<=0;
      end
      if(core_start)begin
        run_row<=x_row;run_set<=x_set;tile_running<=1;x_ready<=0;
        issued_rows<=next_row[6:0];
        if(next_row<99)begin
          x_row<=next_row[6:0];x_set<=!x_set;x_filling<=1;x_issue<=0;
          x_limit<={6'b0,rows_at(next_row[6:0],wide_rows)}<<6;
        end
      end
      if(core_done)tile_running<=0;
      if(core_pop)begin
        for(lane=0;lane<8;lane=lane+1)y_payload[32*lane+:32]<=rne(core_out[lane]);
        y_active<=1;y_issue<=0;pop_count<=pop_count+1;
        y_address<=run_y+(({25'b0,run_row}+{29'b0,core_out_row})<<10)+{22'b0,core_out_group,5'b0};
      end
      if(state==RUN&&issued_rows==99&&!tile_running&&!core_active&&!y_active&&
        !pending&&!locked&&!core_out_valid&&!aborting)begin state<=IDLE;busy<=0;done<=1;end
      if(state==CLEAR&&core_clear_done)begin state<=IDLE;busy<=0;clear_done<=1;end
      if(fault)begin error<=1;error_code<=(rsp_fire&&mem_rsp_err)?1:2;aborting<=1;state<=ABORT;done<=0;end
      if(state==ABORT&&!pending&&!locked)begin
        state<=IDLE;busy<=0;aborting<=0;x_filling<=0;x_ready<=0;y_active<=0;tile_running<=0;
        bias_valid<=0;bias_wait<=0;bias_busy<=0;
      end
      if(cfg_fire)begin
        cfg_rsp_valid<=1;cfg_rsp_err<=0;cfg_rsp_rdata<=0;
        if(!cfg_hit||cfg_cmd_addr[1:0]!=0)cfg_rsp_err<=1;
        else if(cfg_cmd_read)case(cfg_off)
          7'h00:cfg_rsp_rdata<=32'h444d4131;
          7'h08:cfg_rsp_rdata<={28'b0,clear_done,error,done,busy};
          7'h0c:cfg_rsp_rdata<=error_code;
          7'h10:cfg_rsp_rdata<=x_base;7'h14:cfg_rsp_rdata<=w_base;
          7'h18:cfg_rsp_rdata<=b_base;7'h1c:cfg_rsp_rdata<=y_base;
          7'h20:cfg_rsp_rdata<=cycle_count;7'h24:cfg_rsp_rdata<=clear_cycles;
          7'h28:cfg_rsp_rdata<=read_count;7'h2c:cfg_rsp_rdata<=write_count;
          7'h30:cfg_rsp_rdata<=pop_count;7'h34:cfg_rsp_rdata<=mac_count;
          7'h38:cfg_rsp_rdata<=clear_write_count;7'h3c:cfg_rsp_rdata<=cmd_stall_count;
          7'h40:cfg_rsp_rdata<=rsp_wait_count;7'h44:cfg_rsp_rdata<=wb_cycles;
          7'h48:cfg_rsp_rdata<=run_cycles;7'h4c:cfg_rsp_rdata<=32'h00082001;
          7'h50:cfg_rsp_rdata<=run_x;7'h54:cfg_rsp_rdata<=run_w;
          7'h58:cfg_rsp_rdata<=run_b;7'h5c:cfg_rsp_rdata<=run_y;
          default:cfg_rsp_err<=1;
        endcase
        else if(cfg_cmd_wmask!=4'hf)cfg_rsp_err<=1;
        else case(cfg_off)
          7'h04:begin
            if(clear_request)begin
              state<=CLEAR;busy<=1;done<=0;error<=0;error_code<=0;clear_done<=0;aborting<=0;
              clear_cycles<=0;clear_write_count<=0;x_filling<=0;x_ready<=0;tile_running<=0;
              y_active<=0;y_payload<=0;bias_data<=0;bias_addr<=0;bias_busy<=0;bias_valid<=0;bias_wait<=0;
              lock_addr<=0;lock_data<=0;lock_kind<=0;lock_index<=0;lock_read<=0;
              pending_count<=0;pending_read_ptr<=0;pending_write_ptr<=0;
              for(queue_slot=0;queue_slot<REQUEST_DEPTH;queue_slot=queue_slot+1)begin
                request_kind[queue_slot]<=0;request_index[queue_slot]<=0;
              end
              y_address<=0;y_issue<=0;
              run_x<=0;run_w<=0;run_b<=0;run_y<=0;run_row<=0;run_set<=0;
              x_row<=0;x_set<=0;x_issue<=0;x_limit<=0;issued_rows<=0;
              w_issue<=0;w_complete<=0;b_issue<=0;b_complete<=0;w_low<=0;bias_low<=0;
            end else if(start_request)begin
              state<=WB;busy<=1;done<=0;clear_done<=0;wide_rows<=cfg_cmd_wdata[11];
              run_x<=x_base;run_w<=w_base;run_b<=b_base;run_y<=y_base;
              cycle_count<=0;read_count<=0;write_count<=0;pop_count<=0;mac_count<=0;
              cmd_stall_count<=0;rsp_wait_count<=0;wb_cycles<=0;run_cycles<=0;
              w_issue<=0;w_complete<=0;b_issue<=0;b_complete<=0;w_low<=0;bias_low<=0;bias_busy<=0;bias_valid<=0;bias_wait<=0;
              issued_rows<=0;y_active<=0;aborting<=0;
            end else cfg_rsp_err<=1;
          end
          7'h10:if(!busy)x_base<=cfg_cmd_wdata;else cfg_rsp_err<=1;
          7'h14:if(!busy)w_base<=cfg_cmd_wdata;else cfg_rsp_err<=1;
          7'h18:if(!busy)b_base<=cfg_cmd_wdata;else cfg_rsp_err<=1;
          7'h1c:if(!busy)y_base<=cfg_cmd_wdata;else cfg_rsp_err<=1;
          default:cfg_rsp_err<=1;
        endcase
      end
    end
  end
endmodule

`endif
`ifdef C32_FINAL_STORAGE_SELECTED
`undef C32_FINAL_STORAGE_SELECTED
`endif
