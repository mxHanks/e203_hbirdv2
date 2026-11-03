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
    output reg         rsp_err,
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
    // xu additive extensions; the original 0x000--0x03c ABI is unchanged.
    localparam [11:0] REG_CACHE_CTRL   = 12'h040;
    localparam [11:0] REG_CACHE_ADDR   = 12'h044;
    localparam [11:0] REG_CACHE_LO     = 12'h048;
    localparam [11:0] REG_CACHE_HI     = 12'h04C;
    localparam [11:0] REG_CAPABILITY   = 12'h050;
    localparam [31:0] XU_CAPABILITY   = 32'h58550003;
    // xu S2a is additive: the frozen H1/H2 capability remains bit-exact.
    localparam [11:0] REG_W_CURSOR    = 12'h054;
    localparam [11:0] REG_W_STREAM16  = 12'h058;
    localparam [11:0] REG_W_PACK32    = 12'h05C;
    localparam [11:0] REG_X_CURSOR    = 12'h060;
    localparam [11:0] REG_X_STREAM16  = 12'h064;
    localparam [11:0] REG_X_PACK32    = 12'h068;
    localparam [11:0] REG_STREAM_STATUS = 12'h06C;
    localparam [11:0] REG_STREAM_CAP  = 12'h070;
    localparam [31:0] XU_STREAM_CAP  = 32'h58530003;
    // Preserve the frozen S2a exact CAP probe and load image, just as S2a
    // preserved the old H1/H2 CAP. Packed feature bits have a separate word.
    localparam [11:0] REG_PACKED_CAP = 12'h074;
    localparam [31:0] XU_PACKED_CAP = 32'h58500003;
    // S2c has its own capability; all three existing CAP words stay exact.
    localparam [11:0] REG_AUTO_CAP = 12'h078;
    localparam [31:0] XU_AUTO_CAP = 32'h58410001;
    localparam [11:0] REG_ROW_CTRL = 12'h0C0;
    localparam [11:0] REG_ROW_STATUS = 12'h0C4;
    localparam [11:0] REG_ROW_POP = 12'h0C8;
    wire row_active, row_running, row_valid, compute_start;
    wire [1:0] row_count;
    wire [5:0] row_produced, row_retired;
    wire [4:0] row_group;
    wire [47:0] row_result [0:7];
    reg row_start;
    // Metadata follows the one response slot. No head movement at acceptance.
    reg rsp_row_pop;
    wire row_pop = rsp_valid && rsp_ready && rsp_row_pop;
    wire row_start_accept = cmd_valid && cmd_ready && !cmd_read &&
        cmd_addr[11:0] == REG_ROW_CTRL && cmd_wmask == 4'hf &&
        cmd_wdata == 32'd1 && cache_enable && !busy && !row_start &&
        !core_start && !x_load_wen && !w_load_wen && row_count == 0;
    wire row_protected_write = !cmd_read && (row_active || row_start) &&
        ((cmd_addr[11:0] >= REG_GROUP && cmd_addr[11:0] <= REG_BIAS_HI) ||
         (cmd_addr[11:0] >= REG_CACHE_CTRL && cmd_addr[11:0] <= REG_CACHE_HI) ||
         (cmd_addr[11:0] >= REG_W_CURSOR && cmd_addr[11:0] <= REG_X_PACK32));

    reg [14:0] w_cursor;
    reg [6:0] x_cursor;
    reg stream_pending;
    reg stream_is_w;
    reg stream_error;
    reg [2:0] stream_bank;
    reg [10:0] stream_addr;
    reg [31:0] stream_data;
    reg [14:0] stream_cursor;
    reg stream_pair;
    reg stream_second;

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
    reg cache_enable;
    reg [7:0] cache_addr_reg;
    wire [39:0] cache_read_data;
    wire cache_write_lo = cmd_valid && cmd_ready && !cmd_read &&
                          cmd_addr[11:0] == REG_CACHE_LO && !row_active && !row_start;
    wire cache_write_hi = cmd_valid && cmd_ready && !cmd_read &&
                          cmd_addr[11:0] == REG_CACHE_HI && !row_active && !row_start;
    wire [39:0] cache_write_data = cache_write_hi ?
        {cmd_wdata[7:0], 32'b0} : {8'b0, cmd_wdata};
    wire [4:0] cache_write_mask = cache_write_hi ?
        {cmd_wmask[0], 4'b0} : {1'b0, cmd_wmask};

    reg core_start;
    reg manual_busy;
    wire busy = manual_busy || row_active || row_start;
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

    // One captured stream beat owns the scalar core write port until commit.
    // Its response cannot retire before the sampled internal write edge.
    assign cmd_ready = (!rsp_valid || rsp_ready) && !stream_pending;

    // Chisel-generated calculation core.
    // E203 reset is active-low; Chisel reset is active-high.
    LinearAutoGroupCore u_linear_compute_core (
        .clock          (clk),
        .reset          (!rst_n),
        .io_start       (core_start),
        .io_rowStart    (row_start),
        .io_rowPop      (row_pop),
        .io_rowActive   (row_active),
        .io_rowRunning  (row_running),
        .io_rowValid    (row_valid),
        .io_rowCount    (row_count),
        .io_rowProduced (row_produced),
        .io_rowRetired  (row_retired),
        .io_rowGroup    (row_group),
        .io_rowResult_0 (row_result[0]),
        .io_rowResult_1 (row_result[1]),
        .io_rowResult_2 (row_result[2]),
        .io_rowResult_3 (row_result[3]),
        .io_rowResult_4 (row_result[4]),
        .io_rowResult_5 (row_result[5]),
        .io_rowResult_6 (row_result[6]),
        .io_rowResult_7 (row_result[7]),
        .io_computeStart(compute_start),
        .io_group       (group_reg),
        .io_cacheEnable (cache_enable),
        .io_cacheWriteEnable (cache_write_lo || cache_write_hi),
        .io_cacheWriteAddr (cache_addr_reg),
        .io_cacheWriteData (cache_write_data),
        .io_cacheWriteMask (cache_write_mask),
        .io_cacheReadAddr (cache_addr_reg),
        .io_cacheReadData (cache_read_data),

        .io_xLoadWen    (x_load_wen || (stream_pending && !stream_is_w)),
        .io_xLoadAddr   ((stream_pending && !stream_is_w) ? stream_addr[5:0] : x_addr_reg),
        .io_xLoadData   ((stream_pending && !stream_is_w) ? stream_data[15:0] : x_data_reg),

        .io_wLoadWen    (w_load_wen || (stream_pending && stream_is_w)),
        .io_wLoadBank   ((stream_pending && stream_is_w) ? stream_bank : w_bank_reg),
        .io_wLoadAddr   ((stream_pending && stream_is_w) ? stream_addr : w_addr_reg),
        .io_wLoadData   ((stream_pending && stream_is_w) ? stream_data[15:0] : w_data_reg),

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
            rsp_err         <= 1'b0;
            w_cursor        <= 15'b0;
            x_cursor        <= 7'b0;
            stream_pending  <= 1'b0;
            stream_is_w     <= 1'b0;
            stream_error    <= 1'b0;
            stream_bank     <= 3'b0;
            stream_addr     <= 11'b0;
            stream_data     <= 32'b0;
            stream_cursor   <= 15'b0;
            stream_pair     <= 1'b0;
            stream_second   <= 1'b0;
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
            cache_enable <= 1'b0;
            cache_addr_reg <= 8'b0;

            core_start      <= 1'b0;
            manual_busy     <= 1'b0;
            row_start       <= 1'b0;
            rsp_row_pop     <= 1'b0;
            done_sticky     <= 1'b0;

            for (i = 0; i < 8; i = i + 1)
                bias_regs[i] <= 40'b0;

        end else begin
            // These are one-cycle write/start strobes.
            core_start <= 1'b0;
            row_start <= 1'b0;
            if (rsp_valid && rsp_ready) rsp_row_pop <= 1'b0;
            x_load_wen <= 1'b0;
            w_load_wen <= 1'b0;

            // Latch completion so software can poll STATUS.
            if (core_done && !row_active) begin
                manual_busy <= 1'b0;
                done_sticky <= 1'b1;
            end

            // The captured port inputs are sampled by the core on this edge.
            // Advance only once at commit, then hold the response under stall.
            if (stream_pending) begin
                if (stream_pair && !stream_second) begin
                    // Even logical W positions occupy adjacent banks at the
                    // same physical address; X occupies adjacent row entries.
                    // The second element reuses the unchanged scalar core port.
                    if (stream_is_w) stream_bank <= stream_bank + 3'd1;
                    else stream_addr <= stream_addr + 11'd1;
                    stream_data <= {16'b0, stream_data[31:16]};
                    stream_second <= 1'b1;
                end else begin
                    stream_pending <= 1'b0;
                    if (stream_is_w)
                        w_cursor <= stream_cursor + (stream_pair ? 15'd2 : 15'd1);
                    else x_cursor <= stream_cursor[6:0] + (stream_pair ? 7'd2 : 7'd1);
                    rsp_valid <= 1'b1;
                    rsp_rdata <= 32'b0;
                    rsp_err <= 1'b0;
                end
            end

            if (cmd_ready) begin
                rsp_valid <= cmd_valid;

                if (cmd_valid) begin
                    rsp_err <= 1'b0;
                    if (cmd_read && cmd_addr[11:0] >= 12'h100 &&
                        cmd_addr[11:0] <= 12'h13c && cmd_addr[1:0] == 0) begin
                        // Side-effect-free snapshot into the response slot.
                        // Head and all 384 bits remain owned until POP retires.
                        if (!row_valid || row_pop) begin
                            rsp_rdata <= 0;
                            rsp_err <= 1'b1;
                        end else if (cmd_addr[2])
                            rsp_rdata <= {16'b0, row_result[cmd_addr[5:3]][47:32]};
                        else rsp_rdata <= row_result[cmd_addr[5:3]][31:0];
                    end else if (cmd_read) begin
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

                            REG_CACHE_CTRL:
                                rsp_rdata <= {31'b0, cache_enable};
                            REG_CACHE_ADDR:
                                rsp_rdata <= {24'b0, cache_addr_reg};
                            REG_CACHE_LO:
                                rsp_rdata <= cache_read_data[31:0];
                            REG_CACHE_HI:
                                rsp_rdata <= {24'b0, cache_read_data[39:32]};
                            REG_CAPABILITY:
                                rsp_rdata <= XU_CAPABILITY;
                            REG_W_CURSOR:
                                rsp_rdata <= {17'b0, w_cursor};
                            REG_X_CURSOR:
                                rsp_rdata <= {25'b0, x_cursor};
                            REG_STREAM_STATUS:
                                rsp_rdata <= {30'b0, stream_error, stream_pending};
                            REG_STREAM_CAP:
                                rsp_rdata <= XU_STREAM_CAP;
                            REG_PACKED_CAP:
                                rsp_rdata <= XU_PACKED_CAP;
                            REG_AUTO_CAP: rsp_rdata <= XU_AUTO_CAP;
                            REG_ROW_CTRL, REG_ROW_POP: rsp_rdata <= 0;
                            REG_ROW_STATUS:
                                rsp_rdata <= {2'b0, row_retired, 2'b0, row_produced,
                                    5'b0, row_group, row_count, row_running,
                                    (row_produced == 32), row_valid, row_active};

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

                            // Contiguous, side-effect-free result window. Each
                            // lane occupies LO32 followed by zero-extended HI16.
                            12'h080: rsp_rdata <= core_result_0[31:0];
                            12'h084: rsp_rdata <= {16'b0, core_result_0[47:32]};
                            12'h088: rsp_rdata <= core_result_1[31:0];
                            12'h08C: rsp_rdata <= {16'b0, core_result_1[47:32]};
                            12'h090: rsp_rdata <= core_result_2[31:0];
                            12'h094: rsp_rdata <= {16'b0, core_result_2[47:32]};
                            12'h098: rsp_rdata <= core_result_3[31:0];
                            12'h09C: rsp_rdata <= {16'b0, core_result_3[47:32]};
                            12'h0A0: rsp_rdata <= core_result_4[31:0];
                            12'h0A4: rsp_rdata <= {16'b0, core_result_4[47:32]};
                            12'h0A8: rsp_rdata <= core_result_5[31:0];
                            12'h0AC: rsp_rdata <= {16'b0, core_result_5[47:32]};
                            12'h0B0: rsp_rdata <= core_result_6[31:0];
                            12'h0B4: rsp_rdata <= {16'b0, core_result_6[47:32]};
                            12'h0B8: rsp_rdata <= core_result_7[31:0];
                            12'h0BC: rsp_rdata <= {16'b0, core_result_7[47:32]};

                            default:
                                rsp_rdata <= 32'b0;
                        endcase

                    end else if (row_protected_write) begin
                        // Unlike the historical manual path, autonomous rows
                        // own all operand storage through final POP retirement.
                        rsp_err <= 1'b1;
                    end else begin
                        case (cmd_addr[11:0])
                            REG_ROW_CTRL: begin
                                if (row_start_accept) row_start <= 1'b1;
                                else rsp_err <= 1'b1;
                            end
                            REG_ROW_POP: begin
                                if (cmd_wmask != 4'hf || cmd_wdata != 1 ||
                                    !row_valid || row_pop) rsp_err <= 1'b1;
                                else rsp_row_pop <= 1'b1;
                            end
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
                                    manual_busy <= 1'b1;
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

                            REG_W_CURSOR: begin
                                if (cmd_wmask != 4'hf || busy || cmd_wdata > 32'd16384) begin
                                    rsp_err <= 1'b1;
                                    stream_error <= 1'b1;
                                end else w_cursor <= cmd_wdata[14:0];
                            end
                            REG_X_CURSOR: begin
                                if (cmd_wmask != 4'hf || busy || cmd_wdata > 32'd64) begin
                                    rsp_err <= 1'b1;
                                    stream_error <= 1'b1;
                                end else x_cursor <= cmd_wdata[6:0];
                            end
                            REG_W_STREAM16, REG_X_STREAM16,
                            REG_W_PACK32, REG_X_PACK32: begin
                                if (cmd_wmask != 4'hf || busy ||
                                    ((cmd_addr[11:0] == REG_W_STREAM16 ||
                                      cmd_addr[11:0] == REG_W_PACK32) && w_cursor >= 15'd16384) ||
                                    ((cmd_addr[11:0] == REG_X_STREAM16 ||
                                      cmd_addr[11:0] == REG_X_PACK32) && x_cursor >= 7'd64) ||
                                    (cmd_addr[11:0] == REG_W_PACK32 && w_cursor[0]) ||
                                    (cmd_addr[11:0] == REG_X_PACK32 && x_cursor[0])) begin
                                    rsp_err <= 1'b1;
                                    stream_error <= 1'b1;
                                end else begin
                                    // Capture every input and the original cursor.
                                    // No live command/cursor signal is used at commit.
                                    stream_is_w <= cmd_addr[11:0] == REG_W_STREAM16 ||
                                                   cmd_addr[11:0] == REG_W_PACK32;
                                    stream_pair <= cmd_addr[11:0] == REG_W_PACK32 ||
                                                   cmd_addr[11:0] == REG_X_PACK32;
                                    stream_second <= 1'b0;
                                    stream_bank <= w_cursor[2:0];
                                    stream_addr <= (cmd_addr[11:0] == REG_W_STREAM16 ||
                                                    cmd_addr[11:0] == REG_W_PACK32) ?
                                        {w_cursor[7:3], w_cursor[13:8]} : {5'b0, x_cursor[5:0]};
                                    stream_cursor <= (cmd_addr[11:0] == REG_W_STREAM16 ||
                                                      cmd_addr[11:0] == REG_W_PACK32) ?
                                        w_cursor : {8'b0, x_cursor};
                                    stream_data <= cmd_wdata;
                                    stream_pending <= 1'b1;
                                    rsp_valid <= 1'b0;
                                end
                            end
                            REG_STREAM_STATUS: begin
                                if (cmd_wmask[0] && cmd_wdata[1]) stream_error <= 1'b0;
                            end

                            REG_RESULT_INDEX:
                                result_index_reg <= cmd_wdata[2:0];

                            REG_CACHE_CTRL: begin
                                if (cmd_wmask[0]) cache_enable <= cmd_wdata[0];
                            end
                            REG_CACHE_ADDR: begin
                                if (cmd_wmask[0]) cache_addr_reg <= cmd_wdata[7:0];
                            end

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
