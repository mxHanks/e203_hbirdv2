// S3-Y32 drain8: custom3 / funct3=7 / funct7=0x20 (unchanged).
// Optional direct drainN: funct7=0x23, rs2 group[4:0], count_minus1[9:5],
// nonzero per-group status-poll limit[31:10]. The full span stays within
// one Y row and rs1's row offset equals group*32. Direct capability is required.
// Each group polls, snapshots eight, POPs, then waits for eight shared-write
// responses before beginning the next group. Only the final group completes.
// Optional ready drainN: funct7=0x24 uses the same descriptor, but the limit
// counts READY_WAIT edges per group. With claim low, status[31] advertises
// live status; ready wins on the final budget edge. No ROW_STATUS reads.
// Any first error aborts the complete instruction/task; partial POPs cannot retry.
// rs1: 32-byte-aligned Y32 destination in [0xa0020000, 0xa0038be0].
// rs2: [4:0] expected FIFO head group, [31:5] nonzero status-poll limit.
// Status polls count retired responses, never cycles or in-flight requests.
// Success returns err=0/rdat=0. Failure returns err=1/nonzero rdat, held
// together with valid until the CPU accepts the completion. Software MUST
// inspect rdat: the fixed E203 core does not convert NICE err into a trap.
// Read-pop transport: lane7 uses the separate 0x1a0 alias. No rollback:
// the FIFO head has retired before any shared write; any failure aborts the task.
// Requires the matching accelerator read-pop alias; instruction ABI is unchanged.
// Any failure aborts the whole task; never blindly retry a partial drain.
`include "e203_defines.v"

`ifdef E203_HAS_NICE
module xu_nice_drain8 #(parameter DIRECT_ENABLE=0) (
    // Same hfclk domain. Common asynchronous reset assertion; NICE release
    // uses the CPU's existing synchronized rst_aon. No CPU control changes.
    input                         xu_direct_capable,
    output                        xu_direct_claim,
    input                         xu_direct_valid,
    output                        xu_direct_ready,
    input [255:0]                 xu_direct_payload,
    input [31:0]                  xu_direct_status,
    output                        xu_direct_commit,
    input                         xu_direct_commit_ready,
    input                         xu_direct_commit_err,
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
                     DIRECT_WAIT=7, DIRECT_COMMIT=8, READY_WAIT=9, RESP=11;
    // Frozen software return codes. Bus error location is available in the
    // transaction trace; every bus-error phase returns the same nonzero code.
    localparam [31:0] ERR_INSTRUCTION=1, ERR_ALIGNMENT=2, ERR_RANGE=3,
                      ERR_POLL_LIMIT=4, ERR_TIMEOUT=5, ERR_GROUP=6,
                      ERR_ROW_STATE=7, ERR_BUS=8, ERR_GROUP_RANGE=9;
    localparam [31:0] ROW_STATUS=32'h100420c4;
    localparam [31:0] Y32_LANES=32'h10042180, Y32_LANE7_POP=32'h100421a0;
    localparam [31:0] Y_FIRST=32'ha0020000, Y_LAST_GROUP=32'ha0038be0,
                      Y_END=32'ha0038c00;

    reg [3:0] state;
    reg direct_mode, ready_mode;
    reg [31:0] destination, completion_code;
    reg [4:0] expected_group;
    reg [26:0] polls_left, poll_limit;
    reg [5:0] groups_left;
    reg [2:0] lane;
    reg [31:0] snapshot [0:7];
    wire send_command = state==SEND_POLL || state==SEND_READ || state==SEND_WRITE;
    wire outstanding = state==WAIT_POLL || state==WAIT_READ || state==WAIT_WRITE;
    wire command_fire = nice_icb_cmd_valid && nice_icb_cmd_ready;
    wire response_fire = nice_icb_rsp_valid && nice_icb_rsp_ready;
    wire is_ready_n = nice_req_inst[31:25]==7'h24;
    wire is_drain_n = nice_req_inst[31:25]==7'h23 || is_ready_n;
    wire [5:0] request_count = is_drain_n ? {1'b0,nice_req_rs2[9:5]}+6'd1 : 6'd1;
    wire [26:0] request_limit = is_drain_n ? {5'b0,nice_req_rs2[31:10]} : nice_req_rs2[31:5];
    wire [32:0] request_end = {1'b0,nice_req_rs1}+{22'b0,request_count,5'b0};
    wire instruction_ok = nice_req_inst[6:0]==7'h7b &&
                          nice_req_inst[14:12]==3'b111 &&
                          (nice_req_inst[31:25]==7'h20 ||
                           (nice_req_inst[31:25]==7'h23 && DIRECT_ENABLE && xu_direct_capable) ||
                           (is_ready_n && DIRECT_ENABLE && xu_direct_capable &&
                            xu_direct_status[31]));
    integer i;

    // For old 0x20/0x23, only retired ROW_STATUS responses consume polls_left.
    // New 0x24 uses READY_WAIT edges. Direct capture has no cycle timeout.
    assign xu_direct_claim = direct_mode && (state==DIRECT_WAIT || state==DIRECT_COMMIT);
    assign xu_direct_ready = direct_mode && state==DIRECT_WAIT;
    assign xu_direct_commit = direct_mode && state==DIRECT_COMMIT;
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
            SEND_READ: nice_icb_cmd_addr = lane==7 ? Y32_LANE7_POP :
                                                    Y32_LANES + {27'b0,lane,2'b0};
            SEND_WRITE: begin
                nice_icb_cmd_addr = destination + {27'b0,lane,2'b0};
                nice_icb_cmd_read = 0;
                nice_icb_cmd_wdata = snapshot[lane];
            end
            default: begin end
        endcase
    end

    always @(posedge nice_clk or negedge nice_rst_n) begin
        if (!nice_rst_n) begin
            state <= IDLE;
            direct_mode <= 0;
            ready_mode <= 0;
            destination <= 0;
            completion_code <= 0;
            expected_group <= 0;
            polls_left <= 0;
            poll_limit <= 0;
            groups_left <= 1;
            lane <= 0;
            for (i=0; i<8; i=i+1) snapshot[i] <= 0;
        end else begin
            case (state)
                IDLE: if (nice_req_valid && nice_req_ready) begin
                    direct_mode <= 0;
                    ready_mode <= is_ready_n;
                    if (DIRECT_ENABLE && xu_direct_capable) direct_mode <= 1;
                    destination <= nice_req_rs1;
                    expected_group <= nice_req_rs2[4:0];
                    polls_left <= request_limit;
                    poll_limit <= request_limit;
                    groups_left <= request_count;
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
                    end else if (is_drain_n &&
                        ({1'b0,nice_req_rs2[4:0]}+request_count>6'd32 ||
                         nice_req_rs1[9:0]!={nice_req_rs2[4:0],5'b0} ||
                         request_end>{1'b0,Y_END})) begin
                        completion_code <= ERR_GROUP_RANGE;
                        state <= RESP;
                    end else if (request_limit==0) begin
                        completion_code <= ERR_POLL_LIMIT;
                        state <= RESP;
                    end else state <= is_ready_n ? READY_WAIT : SEND_POLL;
                end
                // Claim is deliberately low: the wrapper publishes live row
                // metadata here. No bus command or POP occurs while waiting.
                // Only these edges consume the ready budget; ready/error checks
                // take priority over timeout on its final edge.
                READY_WAIT: begin
                    if (!xu_direct_capable || !xu_direct_status[31]) begin
                        completion_code <= ERR_INSTRUCTION;
                        state <= RESP;
                    end else if (xu_direct_status[1]) begin
                        if (!xu_direct_status[0]) begin
                            completion_code <= ERR_ROW_STATE;
                            state <= RESP;
                        end else if (xu_direct_status[10:6]!=expected_group) begin
                            completion_code <= ERR_GROUP;
                            state <= RESP;
                        end else begin
                            lane <= 0;
                            state <= DIRECT_WAIT;
                        end
                    end else if (polls_left==1) begin
                        completion_code <= ERR_TIMEOUT;
                        state <= RESP;
                    end else polls_left <= polls_left-1'b1;
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
                            state <= direct_mode ? DIRECT_WAIT : SEND_READ;
                        end
                    end else if (polls_left==1) begin
                        completion_code <= ERR_TIMEOUT;
                        state <= RESP;
                    end else begin
                        polls_left <= polls_left-1'b1;
                        state <= SEND_POLL;
                    end
                end
                DIRECT_WAIT: if (xu_direct_valid && xu_direct_ready) begin
                    if (!xu_direct_status[0] || !xu_direct_status[1]) begin
                        completion_code <= ERR_ROW_STATE;
                        state <= RESP;
                    end else if (xu_direct_status[10:6]!=expected_group) begin
                        completion_code <= ERR_GROUP;
                        state <= RESP;
                    end else begin
                        for (i=0; i<8; i=i+1) snapshot[i] <= xu_direct_payload[32*i +: 32];
                        lane <= 0;
                        state <= DIRECT_COMMIT;
                    end
                end
                // Commit is offered one edge AFTER all 8 snapshot registers
                // captured. No POP at the direct bundle acceptance edge.
                DIRECT_COMMIT: if (xu_direct_commit && xu_direct_commit_ready) begin
                    if (xu_direct_commit_err) begin
                        completion_code <= ERR_ROW_STATE;
                        state <= RESP;
                    end else state <= SEND_WRITE;
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
                    end else if (lane==7) begin
                        if (groups_left==1) state <= RESP;
                        else begin
                            // Claim has stayed low throughout all eight writes,
                            // so the unchanged direct wrapper has retired D_DONE.
                            groups_left <= groups_left-1'b1;
                            expected_group <= expected_group+1'b1;
                            destination <= destination+32;
                            polls_left <= poll_limit;
                            lane <= 0;
                            state <= ready_mode ? READY_WAIT : SEND_POLL;
                        end
                    end
                    else begin
                        lane <= lane+1'b1;
                        state <= SEND_WRITE;
                    end
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
