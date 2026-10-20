/**
 * @author mxhanks
 * 
 * 与 ICB 总线连接
 * 先测试：能否使得 CPU 读取到固定内存地址编码
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

    reg [31:0] scratch;
    integer i;

    assign rsp_err = 1'b0;

    // One outstanding response at most.
    assign cmd_ready = !rsp_valid || rsp_ready;

    always @(posedge clk or negedge rst_n) begin
        // 低电平有效的异步复位
        if (!rst_n) begin
            rsp_valid <= 1'b0;
            rsp_rdata <= 32'b0;
            scratch   <= 32'b0;

        end else if (cmd_ready) begin
            rsp_valid <= cmd_valid;

            if (cmd_valid) begin
                rsp_rdata <= 32'b0;

                if (cmd_read) begin
                    case (cmd_addr[11:0])
                        12'h000: rsp_rdata <= ACCEL_ID;
                        12'h004: rsp_rdata <= scratch;
                        default: rsp_rdata <= 32'b0;
                    endcase

                end else if (cmd_addr[11:0] == 12'h004) begin
                    for (i = 0; i < 4; i = i + 1) begin
                        if (cmd_wmask[i]) begin
                            scratch[8*i +: 8] <=
                                cmd_wdata[8*i +: 8];
                        end
                    end
                end
            end
        end
    end

endmodule
