# xk264 工程综合报错修复记录

- 日期：2026-10-31
- 工具：Vivado 2020.2
- 器件：xc7z100ffg900-2
- 顶层：xk264_axi_top

本文档记录本次综合（synth_1）中出现的两个报错、原因分析、代码修改及验证结果。

---

## 一、[Synth 8-91] ambiguous clock in event control

### 1.1 报错原文

```
[Synth 8-91] ambiguous clock in event control
["D:/work_project/FPGA_Prj/zynq_xk264_r1/zynq_xk264_r1.srcs/sources_1/imports/axi_xk264/hdl/core/db/db_control.v":931]
```

### 1.2 出错位置

文件：`zynq_xk264_r1.srcs/sources_1/imports/axi_xk264/hdl/core/db/db_control.v`
模块：`db_control`
位置：`ram_out` 的时序块（第 931 行起）

修改前代码：

```verilog
//////ram_out
reg [127:0] ram_out;
always@(posedge clk or negedge rst)begin
	if(!rst)
		ram_out <= 128'd0;

	if(state==Y)begin           // ← 复位分支后缺少 else
		...
	end

	else if(state==CbCr)begin
		...
	end

	else
		ram_out <= ram_out;
end
```

### 1.3 原因分析

异步复位模板 `always @(posedge clk or negedge rst)` 要求块内**所有**寄存器赋值都被复位条件完全约束，即必须写成：

```verilog
if (!rst) begin
    reg <= 复位值;
end
else begin
    ... 正常时序逻辑 ...
end
```

本例中复位分支后没有 `else`，`if(state==Y) ...` 与复位分支成为并列的两条语句。综合器分析后可知：`rst` 的下降沿到来时，后续 `if(state==Y)/else if(state==CbCr)` 分支同样会执行到 `ram_out` 的赋值，即 `rst` 边沿也可能更新该寄存器。于是同一组触发器同时受 `clk` 和 `rst` 两个事件控制，综合器无法判定哪个是真正的时钟，报 `ambiguous clock in event control`。

该写法在仿真层面也是错误的：复位有效期间 `ram_out` 可能被后续分支覆盖，仿真与综合结果可能不一致。

**佐证**：同一文件其余异步复位块（`wadd_l`、`wen_l`、`wadd_out`、`wen_out` 等）均使用 `if(!rst) ... else if(...)`，只有此块缺少 `else`，因此只有第 931 行报错，可排除时钟网络或复位源问题。

### 1.4 修改内容

将复位分支后的 `if(state==Y)begin` 改为 `else if(state==Y)begin`：

```verilog
always@(posedge clk or negedge rst)begin
	if(!rst)
		ram_out <= 128'd0;

	else if(state==Y)begin     // ← 增加 else
		...
	end

	else if(state==CbCr)begin
		...
	end

	else
		ram_out <= ram_out;
end
```

修改后复位分支完整守卫后续时序逻辑，功能无变化；复位有效期间 `ram_out` 稳定为 0。

---

## 二、[Synth 8-2913] Unsupported Dual Port Block-RAM template for mbuf_reg

### 2.1 报错原文

```
Reason is one or more of the following :
	1: RAM is sensitive to asynchronous reset signal. this RTL style is not supported.
ERROR: [Synth 8-2913] Unsupported Dual Port Block-RAM template for mbuf_reg
ERROR: [Synth 8-6156] failed synthesizing module 'xk264_ext_ctrl'
[".../hdl/wrapper/xk264_ext_ctrl.v":23]
```

### 2.2 出错位置

文件：`zynq_xk264_r1.srcs/sources_1/imports/axi_xk264/hdl/wrapper/xk264_ext_ctrl.v`
模块：`xk264_ext_ctrl`
对象：32×64bit 宏块暂存缓冲 `mbuf`

### 2.3 原因分析

`mbuf` 的访问模式如下：

1. **多写口**：存在两个写进程
   - store 捕获路径：一拍写两个相邻地址（`mbuf[{cap_addr_q,1'b0}]` 与 `mbuf[{cap_addr_q,1'b1}]`）
   - DDR 填充路径：`E_FILL` 状态中 `mbuf[wcur] <= rd_data`
2. **多异步读口**：同一拍需要从多个不同地址组合读取拼装数据
   - `play_d64`：一拍访问 4 个地址
   - `play_d128`：一拍访问 2 个地址
   - `wr_wdata = mbuf[wcur]`
3. **写进程带异步复位**：`always @(posedge clk or negedge rst_n)`，而 Block-RAM 模板不支持挂在异步复位上的存储阵列。

Block-RAM 无法同时满足多写口 + 多异步读口，本身就无法映射；Vivado 在尝试按 BRAM 模板推断时，首先因“RAM 对异步复位敏感”触发 `8-2913` 报错。该缓冲本质是一个小型暂存阵列，应使用寄存器实现。

### 2.4 修改内容

修改一：显式指定寄存器实现，避免触发 BRAM 模板推断

```verilog
    // 32 x 64b macroblock staging buffer
    // 同一拍需要从多个不同地址读出（重放时拼装 64b/128b 数据），
    // 无法映射到 Block-RAM，强制用寄存器实现，避免 BRAM 模板推断报错。
    (* ram_style = "registers" *)
    reg [63:0] mbuf [0:31];
```

修改二：存储写端口改为纯同步，并将两条写路径合并到一个进程

修改前：

```verilog
    // store capture buffer writes
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // no reset of contents required
        end else if (cap_vld_q) begin
            mbuf[{cap_addr_q, 1'b0}] <= ext_rdata_i[63:0];
            mbuf[{cap_addr_q, 1'b1}] <= ext_rdata_i[127:64];
        end
    end
    ...
                E_FILL: begin
                    if (rd_data_valid) begin
                        mbuf[wcur] <= rd_data;
                        wcur      <= wcur + 6'd1;
```

修改后：

```verilog
    // staging buffer writes: store capture (two words per beat) and DDR fill.
    // 内容不需要复位，因此写端口用纯同步逻辑（不能挂在异步复位上，
    // 否则 Vivado 无法按 BRAM 模板推断并报 8-2913）。
    wire fill_wen = rd_data_valid && (state == E_FILL);
    always @(posedge clk) begin
        if (cap_vld_q) begin
            mbuf[{cap_addr_q, 1'b0}] <= ext_rdata_i[63:0];
            mbuf[{cap_addr_q, 1'b1}] <= ext_rdata_i[127:64];
        end
        if (fill_wen) begin
            mbuf[wcur] <= rd_data;
        end
    end
```

```verilog
                E_FILL: begin
                    if (rd_data_valid) begin
                        /* mbuf[wcur] <= rd_data; 已移至统一写进程 */
                        wcur      <= wcur + 6'd1;
```

### 2.5 等效性说明

- `mbuf` 内容原本就不需要复位（原代码复位分支为空），因此去掉异步复位不改变功能，上电初值仍为不定态（与原设计一致）。
- `cap_vld_q` 有效时写入，与原逻辑一致。
- 填充写入条件 `rd_data_valid && state==E_FILL` 与原来 `E_FILL` 分支内的 `if(rd_data_valid)` 完全等价，同一拍、同一地址、同一数据。
- 两条写路径分别只在 `E_CAP` 与 `E_FILL` 状态有效，不会冲突。

---

## 三、验证结果

### 3.1 单元验证（仅综合 xk264_ext_ctrl）

```
Synthesis finished with 0 errors, 0 critical warnings and 0 warnings.
```

`mbuf` 被推断为 32 个 64bit 寄存器（2048 个 FF），无 BRAM 模板报错。

### 3.2 全工程验证（synth_1）

- 日志：`zynq_xk264_r1.runs/synth_1/runme.log`
- 结果：`Synthesis finished with 0 errors, 0 critical warnings and 220 warnings`
- 日志中已无 `8-2913`、`8-91`
- `xk264_axi_top.dcp` 重新生成成功

资源占用（`xk264_axi_top_utilization_synth.rpt`）：

| 资源 | 使用量 | 占比 |
|---|---|---|
| Slice LUTs | 116356 | 41.95% |
| Slice Registers | 73964 | 13.33% |
| Block RAM Tile | 56 | 7.42% |

### 3.3 备注

综合期间工程仍被 Vivado GUI（pid 23400）占用，批处理会话查询 run 状态返回 `Scripts Generated / 0%`，属 GUI 工程锁导致的状态未刷新；实际 runme.log 已完整跑完，无需重复综合。

---

## 四、经验总结

1. 异步复位时序块中，复位分支必须用 `else` 完整守卫后续逻辑，否则复位边沿会被当作第二个时钟，触发 `Synth 8-91`。
2. 存储阵列不能挂在异步复位进程上；无需复位内容时应使用纯同步写进程。
3. 同一拍需要多地址读取的暂存缓冲无法映射 Block-RAM，应通过 `(* ram_style = "registers" *)` 明确指定寄存器实现，避免工具误判 BRAM 模板。
