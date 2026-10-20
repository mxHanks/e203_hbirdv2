/**
 * @author mxhanks
 * 
 * 与 ICB 总线连接
 * 先测试：能否使得 CPU 读取到固定内存地址编码
 * 测试成功，与加速器做对接
 * 
 * 使用 E203 ICB 总线结构里面预留的 o15 输出通道
 */

module accel_icb_slave (
    input  wire        clk,
    input  wire        rst_n,

    // 接收隧道，加速器接收 CPU 信息
    input  wire        cmd_valid,   // CPU 提出有效请求
    output wire        cmd_ready,   // 加速器可以接受请求
    input  wire [31:0] cmd_addr,
    input  wire        cmd_read,
    input  wire [31:0] cmd_wdata,
    input  wire [3:0]  cmd_wmask,

    // 响应隧道，加速器向 CPU 回复信息
    output reg         rsp_valid,   // 加速器提出有效请求
    input  wire        rsp_ready,   // CPU 可以接收请求
    output wire        rsp_err,
    output reg  [31:0] rsp_rdata
);

    localparam [31:0] ACCEL_ID = 32'hACCE2030;

    // Register offsets
    localparam [11:0] REG_ID          = 12'h000;
    localparam [11:0] REG_SCRATCH      = 12'h004;
    localparam [11:0] REG_CTRL         = 12'h008;
    localparam [11:0] REG_STATUS       = 12'h00C;
    localparam [11:0] REG_GROUP        = 12'h010;
    localparam [11:0] REG_X_ADDR       = 12'h014;
    localparam [11:0] REG_X_DATA       = 12'h018;
    localparam [11:0] REG_W_BANK       = 12'h01C;
    localparam [11:0] REG_W_ADDR       = 12'h020;
    localparam [11:0] REG_W_DATA       = 12'h024;
    localparam [11:0] REG_BIAS_INDEX   = 12'h028;
    localparam [11:0] REG_BIAS_LO      = 12'h02C;
    localparam [11:0] REG_BIAS_HI      = 12'h030;
    localparam [11:0] REG_RESULT_INDEX = 12'h034;
    localparam [11:0] REG_RESULT_LO    = 12'h038;
    localparam [11:0] REG_RESULT_HI    = 12'h03C;

    reg [31:0] scratch;

    reg [4:0]  group_reg;

    reg [5:0]  x_addr_reg;
    reg [15:0] x_data_reg;
    reg        x_load_wen;

    reg [2:0]  w_bank_reg;
    reg [10:0] w_addr_reg;
    reg [15:0] w_data_reg;
    reg        w_load_wen;

    reg [2:0]  bias_index_reg;
    reg [39:0] bias_regs [0:7];

    reg [2:0] result_index_reg;

    reg core_start;
    reg busy;
    reg done_sticky;

    wire core_done;

    wire [47:0] core_result_0;
    wire [47:0] core_result_1;
    wire [47:0] core_result_2;
    wire [47:0] core_result_3;
    wire [47:0] core_result_4;
    wire [47:0] core_result_5;
    wire [47:0] core_result_6;
    wire [47:0] core_result_7;

    integer i;

    assign rsp_err  = 1'b0;
    assign cmd_ready = !rsp_valid || rsp_ready;

    // Chisel-generated calculation core.
    // E203 reset is active-low; Chisel reset is active-high.
    LinearComputeCore u_linear_compute_core (
        .clock          (clk),
        .reset          (!rst_n),
        .io_start       (core_start),
        .io_group       (group_reg),

        .io_xLoadWen    (x_load_wen),
        .io_xLoadAddr   (x_addr_reg),
        .io_xLoadData   (x_data_reg),

        .io_wLoadWen    (w_load_wen),
        .io_wLoadBank   (w_bank_reg),
        .io_wLoadAddr   (w_addr_reg),
        .io_wLoadData   (w_data_reg),

        .io_bias_0      (bias_regs[0]),
        .io_bias_1      (bias_regs[1]),
        .io_bias_2      (bias_regs[2]),
        .io_bias_3      (bias_regs[3]),
        .io_bias_4      (bias_regs[4]),
        .io_bias_5      (bias_regs[5]),
        .io_bias_6      (bias_regs[6]),
        .io_bias_7      (bias_regs[7]),

        .io_done        (core_done),
        .io_result_0    (core_result_0),
        .io_result_1    (core_result_1),
        .io_result_2    (core_result_2),
        .io_result_3    (core_result_3),
        .io_result_4    (core_result_4),
        .io_result_5    (core_result_5),
        .io_result_6    (core_result_6),
        .io_result_7    (core_result_7)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rsp_valid       <= 1'b0;
            rsp_rdata       <= 32'b0;
            scratch         <= 32'b0;

            group_reg       <= 5'b0;

            x_addr_reg      <= 6'b0;
            x_data_reg      <= 16'b0;
            x_load_wen      <= 1'b0;

            w_bank_reg      <= 3'b0;
            w_addr_reg      <= 11'b0;
            w_data_reg      <= 16'b0;
            w_load_wen      <= 1'b0;

            bias_index_reg  <= 3'b0;
            result_index_reg <= 3'b0;

            core_start      <= 1'b0;
            busy            <= 1'b0;
            done_sticky     <= 1'b0;

            for (i = 0; i < 8; i = i + 1)
                bias_regs[i] <= 40'b0;

        end else begin
            // These are one-cycle write/start strobes.
            core_start <= 1'b0;
            x_load_wen <= 1'b0;
            w_load_wen <= 1'b0;

            // Latch completion so software can poll STATUS.
            if (core_done) begin
                busy        <= 1'b0;
                done_sticky <= 1'b1;
            end

            if (cmd_ready) begin
                rsp_valid <= cmd_valid;

                if (cmd_valid) begin
                    if (cmd_read) begin
                        case (cmd_addr[11:0])
                            REG_ID:
                                rsp_rdata <= ACCEL_ID;

                            REG_SCRATCH:
                                rsp_rdata <= scratch;

                            REG_CTRL:
                                rsp_rdata <= 32'b0;

                            // STATUS[0] = busy, STATUS[1] = done
                            REG_STATUS:
                                rsp_rdata <= {
                                    30'b0, done_sticky, busy
                                };

                            REG_GROUP:
                                rsp_rdata <= {27'b0, group_reg};

                            REG_X_ADDR:
                                rsp_rdata <= {26'b0, x_addr_reg};

                            REG_X_DATA:
                                rsp_rdata <= {16'b0, x_data_reg};

                            REG_W_BANK:
                                rsp_rdata <= {29'b0, w_bank_reg};

                            REG_W_ADDR:
                                rsp_rdata <= {21'b0, w_addr_reg};

                            REG_W_DATA:
                                rsp_rdata <= {16'b0, w_data_reg};

                            REG_BIAS_INDEX:
                                rsp_rdata <= {29'b0, bias_index_reg};

                            REG_BIAS_LO:
                                rsp_rdata <= bias_regs[bias_index_reg][31:0];

                            REG_BIAS_HI:
                                rsp_rdata <= {
                                    24'b0, bias_regs[bias_index_reg][39:32]
                                };

                            REG_RESULT_INDEX:
                                rsp_rdata <= {29'b0, result_index_reg};

                            REG_RESULT_LO: begin
                                case (result_index_reg)
                                    0: rsp_rdata <= core_result_0[31:0];
                                    1: rsp_rdata <= core_result_1[31:0];
                                    2: rsp_rdata <= core_result_2[31:0];
                                    3: rsp_rdata <= core_result_3[31:0];
                                    4: rsp_rdata <= core_result_4[31:0];
                                    5: rsp_rdata <= core_result_5[31:0];
                                    6: rsp_rdata <= core_result_6[31:0];
                                    7: rsp_rdata <= core_result_7[31:0];
                                    default: rsp_rdata <= 32'b0;
                                endcase
                            end

                            REG_RESULT_HI: begin
                                case (result_index_reg)
                                    0: rsp_rdata <= {16'b0, core_result_0[47:32]};
                                    1: rsp_rdata <= {16'b0, core_result_1[47:32]};
                                    2: rsp_rdata <= {16'b0, core_result_2[47:32]};
                                    3: rsp_rdata <= {16'b0, core_result_3[47:32]};
                                    4: rsp_rdata <= {16'b0, core_result_4[47:32]};
                                    5: rsp_rdata <= {16'b0, core_result_5[47:32]};
                                    6: rsp_rdata <= {16'b0, core_result_6[47:32]};
                                    7: rsp_rdata <= {16'b0, core_result_7[47:32]};
                                    default: rsp_rdata <= 32'b0;
                                endcase
                            end

                            default:
                                rsp_rdata <= 32'b0;
                        endcase

                    end else begin
                        case (cmd_addr[11:0])
                            REG_SCRATCH: begin
                                for (i = 0; i < 4; i = i + 1) begin
                                    if (cmd_wmask[i])
                                        scratch[8*i +: 8]
                                            <= cmd_wdata[8*i +: 8];
                                end
                            end

                            REG_CTRL: begin
                                // CTRL[0] = START. Ignore while busy.
                                if (cmd_wdata[0] && !busy) begin
                                    core_start  <= 1'b1;
                                    busy        <= 1'b1;
                                    done_sticky <= 1'b0;
                                end
                            end

                            // STATUS[1] is write-one-to-clear.
                            REG_STATUS: begin
                                if (cmd_wdata[1])
                                    done_sticky <= 1'b0;
                            end

                            REG_GROUP:
                                group_reg <= cmd_wdata[4:0];

                            REG_X_ADDR:
                                x_addr_reg <= cmd_wdata[5:0];

                            REG_X_DATA: begin
                                x_data_reg <= cmd_wdata[15:0];
                                x_load_wen <= 1'b1;
                            end

                            REG_W_BANK:
                                w_bank_reg <= cmd_wdata[2:0];

                            REG_W_ADDR:
                                w_addr_reg <= cmd_wdata[10:0];

                            REG_W_DATA: begin
                                w_data_reg <= cmd_wdata[15:0];
                                w_load_wen <= 1'b1;
                            end

                            REG_BIAS_INDEX:
                                bias_index_reg <= cmd_wdata[2:0];

                            REG_BIAS_LO: begin
                                for (i = 0; i < 4; i = i + 1) begin
                                    if (cmd_wmask[i])
                                        bias_regs[bias_index_reg][8*i +: 8]
                                            <= cmd_wdata[8*i +: 8];
                                end
                            end

                            REG_BIAS_HI: begin
                                if (cmd_wmask[0])
                                    bias_regs[bias_index_reg][39:32]
                                        <= cmd_wdata[7:0];
                            end

                            REG_RESULT_INDEX:
                                result_index_reg <= cmd_wdata[2:0];

                            default: begin
                                // Unimplemented or read-only address.
                            end
                        endcase
                    end
                end
            end
        end
    end

endmodule