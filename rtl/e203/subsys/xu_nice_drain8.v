// S3-Y32 drain8: custom3 / funct3=7 / funct7=0x20.
// rs1: 32-byte-aligned Y32 destination in [0xa0020000, 0xa0038be0].
// rs2: [4:0] expected FIFO head group, [31:5] nonzero status-poll limit.
// Status polls count retired responses, never cycles or in-flight requests.
// Success returns err=0/rdat=0. Failure returns err=1/nonzero rdat, held
// together with valid until the CPU accepts the completion. Software MUST
// inspect rdat: the fixed E203 core does not convert NICE err into a trap.
// No rollback: a POP may already have retired if the last shared write fails.
// Any failure aborts the whole task; never blindly retry a partial drain.
`include "e203_defines.v"

`ifdef E203_HAS_NICE
module xu_nice_drain8 (
    input                         nice_clk,
    input                         nice_rst_n,
    output                        nice_active,
    output                        nice_mem_holdup,
    input                         nice_req_valid,
    output                        nice_req_ready,
    input  [`E203_XLEN-1:0]       nice_req_inst,
    input  [`E203_XLEN-1:0]       nice_req_rs1,
    input  [`E203_XLEN-1:0]       nice_req_rs2,
    output                        nice_rsp_valid,
    input                         nice_rsp_ready,
    output [`E203_XLEN-1:0]       nice_rsp_rdat,
    output                        nice_rsp_err,
    output                        nice_icb_cmd_valid,
    input                         nice_icb_cmd_ready,
    output reg [`E203_ADDR_SIZE-1:0] nice_icb_cmd_addr,
    output reg                    nice_icb_cmd_read,
    output reg [`E203_XLEN-1:0]   nice_icb_cmd_wdata,
    output [1:0]                  nice_icb_cmd_size,
    input                         nice_icb_rsp_valid,
    output                        nice_icb_rsp_ready,
    input  [`E203_XLEN-1:0]       nice_icb_rsp_rdata,
    input                         nice_icb_rsp_err
);
    localparam [3:0] IDLE=0, SEND_POLL=1, WAIT_POLL=2,
                     SEND_READ=3, WAIT_READ=4, SEND_WRITE=5, WAIT_WRITE=6,
                     SEND_POP=7, WAIT_POP=8, SEND_LAST=9, WAIT_LAST=10, RESP=11;
    // Frozen software return codes. Bus error location is available in the
    // transaction trace; every bus-error phase returns the same nonzero code.
    localparam [31:0] ERR_INSTRUCTION=1, ERR_ALIGNMENT=2, ERR_RANGE=3,
                      ERR_POLL_LIMIT=4, ERR_TIMEOUT=5, ERR_GROUP=6,
                      ERR_ROW_STATE=7, ERR_BUS=8;
    localparam [31:0] ROW_STATUS=32'h100420c4, ROW_POP=32'h100420c8;
    localparam [31:0] Y32_LANES=32'h10042180;
    localparam [31:0] Y_FIRST=32'ha0020000, Y_LAST_GROUP=32'ha0038be0;

    reg [3:0] state;
    reg [31:0] destination, completion_code;
    reg [4:0] expected_group;
    reg [26:0] polls_left;
    reg [2:0] lane;
    reg [31:0] snapshot [0:7];
    wire send_command = state==SEND_POLL || state==SEND_READ ||
                        state==SEND_WRITE || state==SEND_POP || state==SEND_LAST;
    wire outstanding = state==WAIT_POLL || state==WAIT_READ ||
                       state==WAIT_WRITE || state==WAIT_POP || state==WAIT_LAST;
    wire command_fire = nice_icb_cmd_valid && nice_icb_cmd_ready;
    wire response_fire = nice_icb_rsp_valid && nice_icb_rsp_ready;
    wire instruction_ok = nice_req_inst[6:0]==7'h7b &&
                          nice_req_inst[14:12]==3'b111 &&
                          nice_req_inst[31:25]==7'h20;
    integer i;

    assign nice_active = state!=IDLE || nice_req_valid;
    assign nice_mem_holdup = state!=IDLE;
    assign nice_req_ready = state==IDLE;
    assign nice_rsp_valid = state==RESP;
    assign nice_rsp_rdat = completion_code;
    assign nice_rsp_err = completion_code!=0;
    assign nice_icb_cmd_valid = send_command;
    assign nice_icb_cmd_size = 2'b10;
    assign nice_icb_rsp_ready = outstanding;

    // SEND states and all payload inputs remain unchanged until command_fire.
    // WAIT states accept exactly one response and cannot issue another command.
    always @* begin
        nice_icb_cmd_addr = 0;
        nice_icb_cmd_read = 1;
        nice_icb_cmd_wdata = 0;
        case (state)
            SEND_POLL: nice_icb_cmd_addr = ROW_STATUS;
            SEND_READ: nice_icb_cmd_addr = Y32_LANES + {27'b0,lane,2'b0};
            SEND_WRITE: begin
                nice_icb_cmd_addr = destination + {27'b0,lane,2'b0};
                nice_icb_cmd_read = 0;
                nice_icb_cmd_wdata = snapshot[lane];
            end
            SEND_POP: begin
                nice_icb_cmd_addr = ROW_POP;
                nice_icb_cmd_read = 0;
                nice_icb_cmd_wdata = 1;
            end
            SEND_LAST: begin
                nice_icb_cmd_addr = destination + 32'd28;
                nice_icb_cmd_read = 0;
                nice_icb_cmd_wdata = snapshot[7];
            end
            default: begin end
        endcase
    end

    always @(posedge nice_clk or negedge nice_rst_n) begin
        if (!nice_rst_n) begin
            state <= IDLE;
            destination <= 0;
            completion_code <= 0;
            expected_group <= 0;
            polls_left <= 0;
            lane <= 0;
            for (i=0; i<8; i=i+1) snapshot[i] <= 0;
        end else begin
            case (state)
                IDLE: if (nice_req_valid && nice_req_ready) begin
                    destination <= nice_req_rs1;
                    expected_group <= nice_req_rs2[4:0];
                    polls_left <= nice_req_rs2[31:5];
                    lane <= 0;
                    completion_code <= 0;
                    if (!instruction_ok) begin
                        completion_code <= ERR_INSTRUCTION;
                        state <= RESP;
                    end else if (nice_req_rs1[4:0]!=0) begin
                        completion_code <= ERR_ALIGNMENT;
                        state <= RESP;
                    end else if (nice_req_rs1<Y_FIRST || nice_req_rs1>Y_LAST_GROUP) begin
                        completion_code <= ERR_RANGE;
                        state <= RESP;
                    end else if (nice_req_rs2[31:5]==0) begin
                        completion_code <= ERR_POLL_LIMIT;
                        state <= RESP;
                    end else state <= SEND_POLL;
                end
                SEND_POLL: if (command_fire) state <= WAIT_POLL;
                WAIT_POLL: if (response_fire) begin
                    if (nice_icb_rsp_err) begin
                        completion_code <= ERR_BUS;
                        state <= RESP;
                    end else if (nice_icb_rsp_rdata[1]) begin
                        // ROW_STATUS: valid[1], active[0], group[10:6].
                        if (!nice_icb_rsp_rdata[0]) begin
                            completion_code <= ERR_ROW_STATE;
                            state <= RESP;
                        end else if (nice_icb_rsp_rdata[10:6]!=expected_group) begin
                            completion_code <= ERR_GROUP;
                            state <= RESP;
                        end else begin
                            lane <= 0;
                            state <= SEND_READ;
                        end
                    end else if (polls_left==1) begin
                        completion_code <= ERR_TIMEOUT;
                        state <= RESP;
                    end else begin
                        polls_left <= polls_left-1'b1;
                        state <= SEND_POLL;
                    end
                end
                SEND_READ: if (command_fire) state <= WAIT_READ;
                WAIT_READ: if (response_fire) begin
                    if (nice_icb_rsp_err) begin
                        completion_code <= ERR_BUS;
                        state <= RESP;
                    end else begin
                        snapshot[lane] <= nice_icb_rsp_rdata;
                        if (lane==7) begin
                            lane <= 0;
                            state <= SEND_WRITE;
                        end else begin
                            lane <= lane+1'b1;
                            state <= SEND_READ;
                        end
                    end
                end
                SEND_WRITE: if (command_fire) state <= WAIT_WRITE;
                WAIT_WRITE: if (response_fire) begin
                    if (nice_icb_rsp_err) begin
                        completion_code <= ERR_BUS;
                        state <= RESP;
                    end else if (lane==6) state <= SEND_POP;
                    else begin
                        lane <= lane+1'b1;
                        state <= SEND_WRITE;
                    end
                end
                SEND_POP: if (command_fire) state <= WAIT_POP;
                WAIT_POP: if (response_fire) begin
                    if (nice_icb_rsp_err) begin
                        completion_code <= ERR_BUS;
                        state <= RESP;
                    end else state <= SEND_LAST;
                end
                SEND_LAST: if (command_fire) state <= WAIT_LAST;
                WAIT_LAST: if (response_fire) begin
                    if (nice_icb_rsp_err) completion_code <= ERR_BUS;
                    state <= RESP;
                end
                RESP: if (nice_rsp_valid && nice_rsp_ready) state <= IDLE;
                default: begin
                    completion_code <= ERR_ROW_STATE;
                    state <= RESP;
                end
            endcase
        end
    end
endmodule
`endif
