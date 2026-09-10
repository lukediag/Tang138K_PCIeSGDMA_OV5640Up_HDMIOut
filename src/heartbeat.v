// ------------------------------------------------------------
// 通用心跳模块（参数化计数上限）
// 时钟频率由外部提供，计数上限由参数 COUNT_MAX 决定
// 输出在每个计数周期结束时翻转，实现 50% 占空比
// ------------------------------------------------------------
module heartbeat #(
    parameter COUNT_MAX = 12_500_000 - 1   // 默认半秒（25MHz下）
) (
    input  wire       clk,      // 输入时钟
    input  wire       rst_n,    // 低有效异步复位
    output reg        heart     // 心跳输出（高亮）
);

    // 计数器位宽自动适应：使用 $clog2 计算所需位数
    localparam CNT_WIDTH = $clog2(COUNT_MAX + 1);

    reg [CNT_WIDTH-1:0] counter;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter <= 0;
            heart   <= 0;
        end else begin
            if (counter == COUNT_MAX) begin
                counter <= 0;
                heart   <= ~heart;      // 翻转
            end else begin
                counter <= counter + 1;
            end
        end
    end

endmodule