# FluidNC 上位机串口通信与绝对坐标控制协议

## 1. 文档目的

本文档用于指导另一台单片机、PLC、工控机或 PC 上位机，通过串口控制运行 FluidNC 的 ESP32 控制板。

本文档针对以下控制方式：

- 使用 ESP32 UART0 作为上位机通信接口。
- 使用 Grbl 1.1 兼容的 ASCII 文本协议。
- 上位机逐次发送普通 G-code。
- 每条运动命令显式使用毫米、绝对坐标和指定进给速度。
- 不使用 <code>$J</code> Jog 作为主要运动控制方式。
- 每次只发送一个目标，等待命令被接收并等待机器到位后，再发送下一个目标。

推荐的完整运动命令格式为：

~~~text
G21 G90 G94 G53 G1 X300 Y200 Z1 F300\n
~~~

这条命令表示：以毫米为单位，使用绝对机器坐标，以 300 mm/min 的合成进给速度，直线运动到 X=300、Y=200、Z=1。

> **当前配置的重要安全警告**：以上字符串的协议格式正确，但不能据此断定该坐标在当前机器上安全。按 [example_configs/4x_2209_atc.yaml](example_configs/4x_2209_atc.yaml) 中的 `mpos_mm: 0`、`positive_direction` 和 `max_travel_mm: 1000` 推导，X 的配置工作区约为 `0...1000`，Y 和 Z 的配置工作区约为 `-1000...0`。因此 `Y200 Z1` 与这份示例配置定义的 Y/Z 机器坐标方向相反。并且该 YAML 没有启用 `soft_limits`，其默认值为 `false`，固件不一定会在越界前拒绝该运动。首次运行前必须先回零、用 `?` 核对 MPos、低速验证各轴正负方向，并确认目标位于真实机械行程内。

## 2. 最重要的结论

上位机必须遵守以下规则：

1. 普通命令必须以 CR 或 LF 结束；自动控制程序推荐只发送一个 LF。
2. CRLF 也能执行命令，但在当前 UART0 行解析路径中可能额外形成一个空行和一个额外的 <code>ok</code>。
3. 每发送一条普通命令，必须等待与该命令对应的 <code>ok</code> 或 <code>error:n</code>；无待确认命令时收到的 <code>ok</code> 应忽略。
4. <code>ok</code> 只表示命令已被接受，不表示机械已经到位。
5. 对于逐次运动，收到 <code>ok</code> 后应周期发送单字节 <code>?</code>。
6. 只有收到状态 <code>&lt;Idle|...&gt;</code> 后，才发送下一条运动命令。
7. 不要每 10 ms 重复发送相同的 G-code。
8. 使用 <code>G53</code> 机器坐标前，应完成回零，否则机器坐标可能不可信。
9. 收到 <code>error:</code> 或 <code>ALARM:</code> 后，必须停止发送新的运动命令。
10. 状态消息和提示消息可能插入在命令与 <code>ok</code> 之间，不能把“收到任意一行”当作命令完成。

## 3. 通信结构

UART0 的典型通信链路如下：

~~~text
上位机/另一台单片机
        |
        | 3.3V TTL UART，115200 8N1
        v
ESP32 UART0
        |
        v
FluidNC UartChannel
        |
        v
命令分类：G-code / $系统命令 / 实时命令
        |
        v
G-code解析器 -> 运动规划器 -> 步进脉冲 -> TMC2209 -> 电机
~~~

这条链路包含三种不同层次，不能把它们混为同一种“串口控制”：

1. 上位机到 FluidNC：UART0 上传 ASCII G-code 和实时命令。
2. FluidNC 到 TMC2209：UART1 读写驱动芯片寄存器，用于电流、细分、运行模式和诊断。
3. FluidNC 到电机驱动器的实时运动信号：I2S 串行输出扩展为 STEP、DIR、ENABLE；每个 STEP 脉冲才真正推进电机一步。

因此，TMC2209 并不是通过 UART1 持续接收“走到 X300”这样的目标。目标由 FluidNC 运动规划器转换成按时间排列的 STEP/DIR 信号，UART1 只负责驱动器配置与状态读取。

UART0 是 ESP32 的真实硬件 UART，不是 ESP32 内部的虚拟串口。

如果控制板带有 CP2102、CH340 等 USB 转串口芯片，那么电脑上的虚拟 COM 口通常也是通过该芯片连接到同一个 UART0：

~~~text
电脑 USB -> USB转串口芯片 -> ESP32 UART0
~~~

因此，USB 控制和外部单片机直接连接 UART0，可能会共享同一组 UART0 信号。

## 4. UART0 物理层

### 4.1 串口参数

| 参数 | 数值 |
|---|---|
| 波特率 | 115200 |
| 数据位 | 8 |
| 校验位 | 无 |
| 停止位 | 1 |
| 硬件流控 | 无 |
| 信号类型 | 3.3V TTL UART |
| 空闲电平 | 高电平 |
| 字节发送顺序 | UART 标准 LSB first |

代码中的 UART0 默认波特率定义在 [Config.h](FluidNC/src/Config.h)，UART0 初始化位于 [UartChannel.cpp](FluidNC/src/UartChannel.cpp)。

### 4.2 接线

普通 ESP32 的 UART0 默认引脚通常为：

| ESP32 信号 | 常见 GPIO | 方向 |
|---|---:|---|
| U0TXD | GPIO1 | ESP32 输出 |
| U0RXD | GPIO3 | ESP32 输入 |

典型接线：

| 上位单片机 | FluidNC 控制板 |
|---|---|
| TX | UART0 RX / GPIO3 |
| RX | UART0 TX / GPIO1 |
| GND | GND |

必须交叉连接 TX 和 RX：

~~~text
上位机 TX  ----> ESP32 RX
上位机 RX  <---- ESP32 TX
上位机 GND ----- ESP32 GND
~~~

如果两块板分别供电，通常不连接 4Pin 接口中的 VCC，只连接 TX、RX、GND。

### 4.3 电气注意事项

- ESP32 GPIO 不耐受 5V。
- 5V 单片机必须使用电平转换器。
- 不得直接连接带正负电压的传统 RS-232 接口。
- UART TTL、RS-232 和 RS-485 是不同电气标准。
- 两端必须共地。
- 如果 USB 转串口芯片也驱动 UART0 RX，应避免它与外部单片机同时发送。
- 烧录 ESP32 固件时，应让外部单片机停止发送或将 TX 设置为高阻态。
- UART0 引脚是否真正引出到 4Pin 接口，需要以控制板原理图为准。

## 5. 每个 UART 从谁读取、读取什么

以下结论针对仓库中的 [example_configs/4x_2209_atc.yaml](example_configs/4x_2209_atc.yaml) 和经典 ESP32。其他 YAML、其他 ESP32 型号或重新布线的控制板可能不同。

| ESP32 串口 | ESP32 RX 从谁读取 | ESP32 读取的内容 | ESP32 TX 发给谁 | ESP32 发出的内容 | 参数和引脚 |
|---|---|---|---|---|---|
| UART0 | PC 的 USB-UART 芯片，或外部 MCU 的 TX | G-code、`$` 系统命令、实时命令字节 | PC/外部 MCU | `ok`、`error:n`、`ALARM:n`、`<...>` 状态和日志 | 115200 8N1；经典 ESP32 通常 TX=GPIO1、RX=GPIO3 |
| UART1 | 多个 TMC2209 的 UART 返回线 | 寄存器响应、芯片版本、`IFCNT`、`TSTEP`、`SG_RESULT` 等诊断值 | X/Y/Z TMC2209 | 电流、细分、斩波模式、StealthChop/CoolStep、使能相关寄存器读写请求 | 115200 8N1；TX=GPIO16、RX=GPIO4 |
| UART2 | 串口 IO 扩展器或另一外部串口设备的 TX | 扩展器应答、输入事件，也可由 `uart_channel2` 接收命令行 | 串口 IO 扩展器或外部设备 | 扩展器请求、命令应答、状态报告 | 1000000 8N1；TX=GPIO25、RX=GPIO26 |

### 5.1 UART0：上位机主控制通道

UART0 在 [UartChannel.cpp](FluidNC/src/UartChannel.cpp) 中以 `Uart0(0, true)` 创建，并用 `BAUD_RATE`、8 数据位、无校验、1 停止位初始化。`BAUD_RATE` 在 [Config.h](FluidNC/src/Config.h) 中为 115200。

UART0 的接收数据进入 FluidNC 的通道解析器：

~~~text
外部 MCU TX / USB-UART TX
        -> ESP32 UART0 RX
        -> Uart::read()
        -> UartChannel::lineComplete()
        -> Channel / Protocol
        -> GCode.cpp 或系统命令处理器
~~~

UART0 是本文推荐给上位机使用的接口。它既接收，也返回完整的 Grbl 协议响应。若 USB-UART 和外部 MCU 同时接在 UART0 RX 上，只能有一个发送方主动驱动该信号，否则会发生电气冲突和字节混杂。

### 5.2 UART1：TMC2209 寄存器通道

三个轴在 YAML 中分别使用 TMC 地址 0、1、2，但共用 UART1。FluidNC 通过地址区分 X、Y、Z 驱动器。

启动和模式切换时，FluidNC 会写入或读取 TMC2209 寄存器。例如：

- 设置运行电流和保持电流。
- 设置 16 微步细分。
- 设置 StealthChop、CoolStep 或 StallGuard 相关参数。
- 读取版本寄存器确认芯片存在。
- 读取写计数 `IFCNT` 验证一次写操作是否成功。
- 调试时读取 `TSTEP` 和 `SG_RESULT`。

UART1 不接收 G-code，也不负责逐脉冲运动。真正的 X/Y/Z 运动信号来自 YAML 中的 `i2so.*` 引脚：

| 轴 | ENABLE | DIR | STEP |
|---|---|---|---|
| X | `i2so.0` | `i2so.1:low` | `i2so.2` |
| Y | `i2so.7` | `i2so.4:low` | `i2so.5` |
| Z | `i2so.8` | `i2so.9:low` | `i2so.10` |

这些 `i2so.*` 是 I2S 串行输出扩展位，不是 ESP32 UART。相关配置和代码位于 YAML、[TMC2209Driver.cpp](FluidNC/src/Motors/TMC2209Driver.cpp)、[TrinamicUartDriver.cpp](FluidNC/src/Motors/TrinamicUartDriver.cpp) 和 [StandardStepper.cpp](FluidNC/src/Motors/StandardStepper.cpp)。

### 5.3 UART2：当前配置的扩展通道

当前 YAML 同时定义了 `uart2` 和 `uart_channel2`。`uart_channel2` 会注册为 FluidNC 通道，因此能够参与命令行接收；初始化时它还会发送扩展器识别请求，底层会识别串口 IO 扩展器的特殊输入事件字节。

`report_interval_ms: 75` 表示 UART2 在运动期间可大约每 75 ms 主动输出一次状态报告，状态变化、引脚变化等事件也可能触发报告。因此 UART2 上的 `<...>` 可能没有对应的 `?` 请求，是正常的异步消息。若要把 UART2 改作普通上位机通道，必须让上位机适配 1 Mbaud、初始化扩展器消息、自动状态上报和该接口现有硬件连接。除非有明确的硬件设计理由，外部 MCU 应优先使用 UART0。

## 6. 应用层协议

FluidNC 使用 Grbl 兼容的文本协议。

协议特点：

- 普通命令是可打印 ASCII 字符组成的一行文本。
- 普通命令末尾需要行结束符。
- 实时命令是单字节，不需要行结束符。
- FluidNC 不要求包头、长度字段或 CRC。
- 命令和应答按发送顺序对应。
- 协议本身没有事务编号。
- 上位机应保持最多一条未确认的普通命令，简化应答关联。

### 6.1 输入行结束符

FluidNC 接受：

| 结束方式 | 字节 |
|---|---|
| CR | <code>0x0D</code> |
| LF | <code>0x0A</code> |
| CRLF | <code>0x0D 0x0A</code>；会被看作连续两个行结束符 |

推荐 MCU 自动控制程序只发送 LF：

~~~c
uart_send("G21 G90 G94 G53 G1 X300 Y200 Z1 F300\n");
~~~

这里的 <code>\n</code> 必须由程序转换为一个 `0x0A` 字节，不能发送反斜杠和字母 n 两个可见字符。

当前 [UartChannel.cpp](FluidNC/src/UartChannel.cpp) 使用 `Lineedit::step()`，CR 和 LF 都会各自结束一行；[ProcessSettings.cpp](FluidNC/src/ProcessSettings.cpp) 又把空行作为同步行并返回 `ok`。所以发送 CRLF 时，典型接收结果可能是：

~~~text
上位机 -> G1 X10\r\n
FluidNC -> ok          ; G1 X10 的应答
FluidNC -> ok          ; CRLF 中第二个结束符形成的空行应答
~~~

若现有硬件库固定发送 CRLF，也可以继续使用，但接收状态机必须只在“确实有待确认普通命令”时消费 `ok`。最简单且应答一一对应的做法是统一发送单个 LF。

### 6.2 输出行结束符

UART0 会把输出换行转换为 CRLF。上位机接收器仍建议同时兼容：

- LF
- CRLF
- 偶发的空行

推荐接收算法：

1. 每收到一个字节就追加到接收缓冲区。
2. 忽略 CR。
3. 收到 LF 时完成一行。
4. 删除行首尾空白。
5. 空行直接忽略。
6. 根据整行内容或首字符分类。

### 6.3 最大命令长度

FluidNC 的通道行缓冲区约为 255 字节。建议上位机将单条命令限制在 254 个 ASCII 字符以内，并尽量保持命令简短。

本文推荐的绝对运动命令通常少于 80 字节。

## 7. 命令分类

### 7.1 普通 G-code

普通 G-code 需要行结束符，并最终返回 <code>ok</code> 或 <code>error:n</code>。

示例：

~~~text
G21 G90 G94 G53 G1 X300 Y200 Z1 F300\n
~~~

### 7.2 FluidNC/Grbl 系统命令

系统命令以美元符号开头，也需要行结束符。

常用命令：

| 命令 | 功能 |
|---|---|
| <code>$H</code> | 全轴回零 |
| <code>$HX</code> | X 轴回零 |
| <code>$HY</code> | Y 轴回零 |
| <code>$HZ</code> | Z 轴回零 |
| <code>$X</code> | 解除普通报警锁定 |
| <code>$$</code> | 显示 Grbl 兼容设置 |
| <code>$G</code> | 显示当前 G-code 模态状态 |
| <code>$#</code> | 显示坐标偏移 |
| <code>$I</code> | 显示固件构建信息 |
| <code>$C</code> | 切换 G-code 检查模式 |
| <code>$SS</code> | 显示启动日志 |

系统命令也必须等待最终的 <code>ok</code> 或 <code>error:n</code>。部分查询命令会先输出若干数据行，最后才输出 <code>ok</code>。

### 7.3 实时命令

实时命令是单字节，可以插入普通命令数据流，不需要 CRLF，也通常不返回 <code>ok</code>。

| 字节 | ASCII/数值 | 功能 |
|---|---:|---|
| <code>?</code> | 0x3F | 请求实时状态 |
| <code>!</code> | 0x21 | Feed Hold，暂停进给 |
| <code>~</code> | 0x7E | Cycle Start，恢复运行 |
| Ctrl-X | 0x18 | 软件复位 |
| Safety Door | 0x84 | 安全门事件 |
| Jog Cancel | 0x85 | 取消 Jog |
| Macro 0..3 | 0x87..0x8A | 触发宏 |
| Feed Override Reset | 0x90 | 进给倍率恢复 100% |
| Feed Override +10% | 0x91 | 进给倍率粗增 |
| Feed Override -10% | 0x92 | 进给倍率粗减 |
| Feed Override +1% | 0x93 | 进给倍率细增 |
| Feed Override -1% | 0x94 | 进给倍率细减 |
| Rapid Override Reset | 0x95 | 快速倍率恢复 |
| Rapid Override Medium | 0x96 | 快速倍率中档 |
| Rapid Override Low | 0x97 | 快速倍率低档 |
| Spindle Override Reset | 0x99 | 主轴倍率恢复 |
| Spindle Override +10% | 0x9A | 主轴倍率粗增 |
| Spindle Override -10% | 0x9B | 主轴倍率粗减 |
| Spindle Override +1% | 0x9C | 主轴倍率细增 |
| Spindle Override -1% | 0x9D | 主轴倍率细减 |
| Spindle Stop Toggle | 0x9E | 主轴停止切换 |
| Flood Toggle | 0xA0 | 洪流冷却切换 |
| Mist Toggle | 0xA1 | 雾化冷却切换 |

本文的逐次绝对坐标方案主要使用 <code>?</code>、<code>!</code>、<code>~</code> 和必要时的 Ctrl-X。

### 7.4 GCode.cpp 如何“解包”一行命令

这里的“解包”不是二进制协议解包。UART0 收到的是一行 ASCII 字符，行结束后 [GCode.cpp](FluidNC/src/GCode.cpp) 的 `gc_execute_line()` 按 RS274/NGC 规则解析。源码流程可概括为四步。

#### 第 0 步：规范化文本

`collapseGCode()` 会直接在原字符串中处理：

- 删除空格、制表符等空白。
- 把小写字母转成大写。
- 去掉 `(注释)`。
- 遇到分号 `;` 时，把其后内容作为行尾注释去掉。
- 忽略意外混入行内的 CR。

例如：

~~~text
g21 g90 g53 g1 x300 y200 z1 f300 ; move
~~~

会被规范化成近似：

~~~text
G21G90G53G1X300Y200Z1F300
~~~

#### 第 1 步：建立临时解析块

解析器创建临时的 `gc_block`，并把当前 `gc_state` 中的模态状态复制进去。新命令先修改临时块，不会立即修改系统状态。这样只要后续任一校验失败，整行命令都可以被拒绝，已有模态状态不会被半更新。

#### 第 2 步：逐个读取“字母 + 数值”单词

解析器扫描每个 G-code word。例如目标命令会被拆成：

| Word | 解析结果 |
|---|---|
| `G21` | 单位模式：毫米 |
| `G90` | 距离模式：绝对 |
| `G94` | 进给模式：单位/分钟 |
| `G53` | 非模态机器坐标覆盖 |
| `G1` | 直线插补运动模式 |
| `X300` | X 目标值 300 |
| `Y200` | Y 目标值 200 |
| `Z1` | Z 目标值 1 |
| `F300` | 进给速度 300 |

G、M 命令按模态组记录，X/Y/Z/F 等值参数按位记录。这样可以发现同一行中的重复 word、同一模态组冲突和不支持的命令。例如同一行同时出现 `G90 G91` 会触发模态组冲突。

#### 第 3 步：语义校验和坐标预计算

解析完成后，源码统一检查：

- 行号、数值范围和负值限制。
- 需要进给速度的运动是否已有有效 F。
- 轴参数是否缺失、重复或用于不允许的命令。
- 坐标系、平面、圆弧、探针等组合是否合法。
- 是否还有未被任何命令使用的参数。
- `G53` 是否与允许的运动模式配合；普通用法应配合 `G0` 或 `G1`。

同时完成单位和坐标换算：

- `G20` 的英寸值乘以 25.4 转成内部毫米。
- `G90` 按绝对目标处理，`G91` 按当前位置加增量处理。
- 未写出的轴保持当前机器目标位置。
- 普通 G90 工件坐标会加上当前 WCS、G92 和刀长偏移，换成内部机器坐标。
- 使用 `G53` 时跳过工件坐标偏移，X/Y/Z 直接作为本行机器坐标。

如果这一步返回错误，整行不进入运动规划器，外部通道收到 `error:n`。

#### 第 4 步：执行和加入规划器

全部校验通过后，解析器才会：

1. 更新允许持久化的 G-code 模态状态。
2. 设置本行进给速度、主轴和冷却等规划数据。
3. 把直线目标提交给运动规划器。
4. 由规划器按最大速度和加速度生成运动段。
5. 由步进模块输出 I2S STEP/DIR/ENABLE 信号。

`G21`、`G90`、`G94` 和 `G1` 是模态状态，可以影响后续行；`G53` 是非模态覆盖，只对当前行生效。本文仍建议每条目标命令完整写出全部模式，避免上位机和下位机因复位、手工操作或其他通道命令而产生模态状态分歧。

## 8. 启动与连接同步

### 8.1 正常启动

FluidNC 初始化完成后会输出类似：

~~~text
Grbl 1.1... [FluidNC ... '$' for help]
~~~

启动期间还可能输出配置、驱动器和网络日志，例如：

~~~text
[MSG:...]
[MSG:INFO: ...]
[MSG:ERR: ...]
~~~

上位机不能把任意启动日志当作“已经可以运动”。

### 8.2 推荐启动流程

~~~text
打开UART
   |
清空上位机本地接收缓冲
   |
等待 "Grbl " 欢迎行，或者发送 ? 并收到合法状态
   |
解析状态
   |
Alarm -> 执行 $H 或按故障原因处理
Idle  -> 可以下发运动
~~~

详细步骤：

1. 初始化 UART0 为 115200 8N1。
2. 启用持续接收。
3. 等待以 <code>Grbl </code> 开头的欢迎行。
4. 如果 ESP32 早已启动、没有收到欢迎行，则发送单字节 <code>?</code>。
5. 收到以 <code>&lt;</code> 开头、以 <code>&gt;</code> 结束的状态行后，认为串口协议已同步。
6. 如果状态为 <code>Alarm</code>，不要发送运动命令。
7. 已配置回零开关时发送 <code>$H\n</code>。
8. 等待回零命令完成并确认状态为 <code>Idle</code>。
9. 开始发送绝对坐标运动。

### 8.3 软件复位后的处理

发送 Ctrl-X，即字节 <code>0x18</code>，会终止当前任务、清空运动状态并重新输出欢迎信息。

上位机收到新的 <code>Grbl ...</code> 行时必须：

- 清空所有待确认命令。
- 将当前命令标记为未知结果。
- 停止自动发送后续运动。
- 重新查询状态。
- 必要时重新回零。

不要为了普通超时立即发送 Ctrl-X，因为它会停止机器。

## 9. FluidNC 输出消息分类

上位机必须支持异步消息分类。

| 格式 | 含义 | 是否结束当前普通命令 |
|---|---|---|
| <code>ok</code> | 当前普通命令成功 | 是 |
| <code>error:n</code> | 当前普通命令失败 | 是 |
| <code>ALARM:n</code> | 机器进入报警 | 否，但应立即停止发送 |
| <code>&lt;...&gt;</code> | 实时状态 | 否 |
| <code>[MSG:...]</code> | 提示、错误或日志 | 否 |
| <code>[GC:...]</code> | G-code 模态报告 | 否 |
| <code>[G54:...]</code> 等 | 坐标偏移报告 | 否 |
| <code>Grbl ...</code> | 启动或复位完成 | 使原会话失效 |
| 空行 | 分隔或启动输出 | 否 |

例如，以下接收顺序是合法的：

~~~text
上位机 -> G21 G90 G94 G53 G1 X300 Y200 Z1 F300\n
FluidNC -> [MSG:某条提示]
FluidNC -> <Run|MPos:10.000,5.000,0.000|FS:300,0>
FluidNC -> ok
~~~

上位机必须继续等待明确的 <code>ok</code> 或 <code>error:</code>，不能在收到 <code>[MSG:...]</code> 或状态行后直接发送下一条普通命令。

## 10. 普通命令流控

### 10.1 停止等待模式

本文推荐始终只保留一条未确认命令：

~~~text
发送普通命令
    |
等待 ok / error:
    |
成功且是运动命令
    |
周期发送 ?
    |
等待 Idle
    |
发送下一条普通命令
~~~

这种方式吞吐量不高，但最容易实现，最适合“上位机逐次给定一个绝对目标”的应用。

### 10.2 ok 的准确含义

对于普通运动命令：

~~~text
G21 G90 G94 G53 G1 X300 Y200 Z1 F300\n
~~~

收到：

~~~text
ok
~~~

表示：

- 命令格式有效。
- 当前状态允许处理该命令。
- 命令已经由 G-code 解析器接受。
- 运动通常已经进入规划流程。

不表示：

- 电机已经开始运动。
- 电机已经运动完成。
- 机械实际位置一定正确。
- 编码器已经确认位置。

FluidNC 对普通开环步进电机通常只知道其内部计算位置，不知道是否机械失步。

### 10.3 error 的处理

收到 <code>error:n</code> 时：

1. 当前命令没有按预期成功执行。
2. 不要自动发送下一条运动命令。
3. 记录错误码和原始命令。
4. 发送 <code>?</code> 查询状态。
5. 根据错误原因决定修改命令、回零、解锁或人工处理。

不要在没有判断原因的情况下无限重发。

## 11. 坐标系统

### 11.1 G90

<code>G90</code> 表示轴坐标采用绝对模式。

例如：

~~~text
G21 G90 G1 X300 Y200 Z1 F300
~~~

如果当前使用 G54，则 X300/Y200/Z1 是 G54 工件坐标，不一定是机器坐标。

### 11.2 G53

<code>G53</code> 表示本行使用机器坐标。

它是非模态命令，只对当前这一行有效。因此每条需要机器坐标的运动命令都应重复写 <code>G53</code>：

~~~text
G21 G90 G94 G53 G1 X300 Y200 Z1 F300\n
G21 G90 G94 G53 G1 X100 Y50 Z1 F300\n
~~~

不要假设上一条命令中的 G53 会自动保留。

### 11.3 G54 等工件坐标

如果不使用 G53，G90 坐标通常受当前 G54-G59、G92 和刀具长度偏移影响。

关系可简化为：

~~~text
工件位置 WPos = 机器位置 MPos - 工件坐标偏移 WCO
~~~

因此：

- 要控制绝对机器位置，使用 G53。
- 要控制加工零点对应的绝对工件位置，使用 G54-G59，不使用 G53。

### 11.4 回零的重要性

机器坐标只有在以下条件成立时才可信：

- 上电后已完成回零；或
- 有可靠的绝对位置反馈并由固件同步；或
- 操作者以其他可靠方式建立了机器坐标。

普通步进电机在断电后不能保留真实机械位置。若未回零直接使用 G53，内部坐标可能与机械位置不一致。

## 12. 推荐的绝对运动命令

推荐格式：

~~~text
G21 G90 G94 G53 G1 X{X目标} Y{Y目标} Z{Z目标} F{速度}\n
~~~

字段说明：

| 字段 | 含义 |
|---|---|
| G21 | 毫米单位 |
| G90 | 绝对坐标 |
| G94 | F 使用单位/分钟 |
| G53 | 当前行使用机器坐标 |
| G1 | 直线插补 |
| X/Y/Z | 目标位置，单位 mm |
| F | 合成路径进给速度，单位 mm/min |

目标 X=300、Y=200、Z=1，速度 300 mm/min：

~~~text
G21 G90 G94 G53 G1 X300 Y200 Z1 F300\n
~~~

这只说明“怎样编码这个目标”，不等于当前机器一定允许该目标。对本仓库的 `4x_2209_atc.yaml` 而言，按配置推导的机器坐标范围为：

| 轴 | 回零方向 | 回零点 MPos | 配置行程 | 推导的机器坐标区间 |
|---|---|---:|---:|---|
| X | 负方向 | 0 | 1000 mm | 约 `0...1000` |
| Y | 正方向 | 0 | 1000 mm | 约 `-1000...0` |
| Z | 正方向 | 0 | 1000 mm | 约 `-1000...0` |

如果“Y 距离正端回零点 200 mm、Z 距离正端回零点 1 mm”才是实际机械意图，那么按这份配置，其机器坐标更可能是 `Y-200 Z-1`。这必须通过低速点动、回零后的 MPos 和机械图纸验证，不能仅凭此表直接执行。

如果不需要改变某个轴，可以省略该轴。省略的轴保持当前目标位置：

~~~text
G21 G90 G94 G53 G1 X300 Y200 F300\n
~~~

该命令只改变 X、Y，Z 保持不变。

### 12.1 多轴运动轨迹

将 X、Y、Z 写在同一条 G1 中时，三个轴进行协调直线插补。

这意味着刀具从当前位置沿一条三维直线路径移动到目标，而不是依次先走 X、再走 Y、最后走 Z。

如果工艺要求分步运动，应拆成多条命令，并在每一步到达 Idle 后发送下一步。例如：

~~~text
G21 G90 G94 G53 G1 Z-10 F100\n
等待 ok 和 Idle

G21 G90 G94 G53 G1 X300 Y-200 F300\n
等待 ok 和 Idle

G21 G90 G94 G53 G1 Z-1 F50\n
等待 ok 和 Idle
~~~

具体安全高度和顺序必须根据机械结构确定，本文中的数值仅作协议示例。

## 13. 完整的单次运动事务

目标：运动到机器坐标 X=300、Y=200、Z=1。

### 13.1 查询当前状态

上位机发送单字节：

~~~text
?
~~~

可能收到：

~~~text
<Idle|MPos:0.000,0.000,0.000|FS:0,0>
~~~

只有状态允许运动时才继续。

### 13.2 发送运动命令

~~~text
G21 G90 G94 G53 G1 X300 Y200 Z1 F300\n
~~~

### 13.3 等待命令确认

成功：

~~~text
ok
~~~

失败：

~~~text
error:错误码
~~~

### 13.4 查询运动过程

建议每 100 ms 发送一次 <code>?</code>：

~~~text
? -> <Run|MPos:15.000,10.000,0.100|FS:300,0>
? -> <Run|MPos:150.000,100.000,0.500|FS:300,0>
? -> <Idle|MPos:300.000,200.000,1.000|FS:0,0>
~~~

### 13.5 判断完成

完成条件建议同时包括：

- 状态为 <code>Idle</code>。
- 收到的位置字段存在。
- 报告位置与目标在允许误差范围内。
- 未收到 <code>ALARM:</code>。

例如位置容差设为 0.01 mm：

~~~text
abs(X实际 - 300.0) <= 0.01
abs(Y实际 - 200.0) <= 0.01
abs(Z实际 - 1.0)   <= 0.01
~~~

对于开环步进系统，这只是固件计算位置的确认，不是独立编码器反馈。

## 14. 实时状态报告

发送 <code>?</code> 后，状态行以 <code>&lt;</code> 开头、以 <code>&gt;</code> 结束：

~~~text
<状态|字段1|字段2|...>
~~~

示例：

~~~text
<Run|MPos:120.000,80.000,0.400|FS:300,0|WCO:0.000,0.000,0.000>
~~~

### 14.1 状态名称

| 状态文本 | 含义 | 是否允许发送新运动 |
|---|---|---|
| Idle | 空闲，规划运动已完成 | 可以 |
| Run | 正在运行 | 本方案不发送新运动 |
| Hold:0 | 暂停已经完成 | 不可以 |
| Hold:1 | 正在减速进入暂停 | 不可以 |
| Home | 正在回零 | 不可以 |
| Jog | 正在 Jog | 不可以 |
| Alarm | 报警或配置报警 | 不可以 |
| Check | G-code 检查模式，不实际运动 | 不可以用于真实运动 |
| Door:0 | 门已关闭，等待恢复 | 不可以 |
| Door:1 | 门打开且已完成撤回 | 不可以 |
| Door:2 | 正在安全门撤回 | 不可以 |
| Door:3 | 正在恢复 | 不可以 |
| Sleep | 休眠 | 不可以 |

### 14.2 常见字段

| 字段 | 示例 | 含义 |
|---|---|---|
| MPos | MPos:300.000,200.000,1.000 | 机器坐标 |
| WPos | WPos:300.000,200.000,1.000 | 工件坐标 |
| Bf | Bf:15,240 | 规划缓冲可用量、串口接收可用量 |
| Ln | Ln:123 | 当前 G-code 行号 |
| FS | FS:300,0 | 实时进给速度、主轴速度 |
| Pn | Pn:XYZP | 当前有效输入引脚，字符取决于配置 |
| WCO | WCO:0.000,0.000,0.000 | 工件坐标偏移 |
| Ov | Ov:100,100,100 | 进给、快速、主轴倍率 |
| A | A:SFM | 主轴和冷却液状态 |

状态字段不保证每一帧全部出现。WCO 和 Ov 会按一定周期插入。上位机必须按字段名称解析，不能依赖固定字段数量或固定顺序。

如果状态报告提供的是 `WPos` 而不是 `MPos`，可按下式恢复机器坐标：

~~~text
MPos = WPos + WCO
~~~

`WCO` 不保证每帧都出现，上位机应缓存最近一次有效 WCO。若从未收到 WCO，就不能把 WPos 直接当成机器坐标完成值。

### 14.3 状态查询周期

推荐：

- 运动中：50 至 200 ms 查询一次。
- 常用值：100 ms，即 10 Hz。
- 空闲时：200 至 1000 ms，或按需查询。

不推荐每 10 ms 查询一次。100 Hz 状态查询会增加串口和实时任务负担，没有必要。

## 15. 暂停、恢复和停止

### 15.1 暂停

发送单字节：

~~~text
!
~~~

随后查询状态，可能依次看到：

~~~text
<Hold:1|...>
<Hold:0|...>
~~~

### 15.2 恢复

发送单字节：

~~~text
~
~~~

状态会重新进入 Run，完成后进入 Idle。

### 15.3 软件复位

发送：

~~~text
0x18
~~~

软件复位用于严重异常或需要立即终止 FluidNC 当前流程的情况。它不是普通的“停止后继续”命令。

软件复位后：

- 当前运动被终止。
- 原命令队列失效。
- 上位机需要等待新的 Grbl 欢迎行。
- 机器位置可能需要重新回零。

## 16. 回零和解锁

### 16.1 推荐回零

~~~text
$H\n
~~~

回零可能持续较长时间。上位机不应使用普通运动命令的短超时判断回零失败。

回零过程中可发送 <code>?</code>，应看到：

~~~text
<Home|...>
~~~

完成后应收到命令应答，并最终看到：

~~~text
<Idle|...>
~~~

### 16.2 解除报警

~~~text
$X\n
~~~

<code>$X</code> 只解除允许解除的报警锁定，不会自动建立可信的机器坐标。

如果后续使用 G53，优先执行回零，而不是仅使用 $X。

### 16.3 配置报警

如果启动信息包含配置无效，或状态为 Alarm 且提示配置错误，则不能依靠 $X 恢复运动。必须修正 FluidNC 的 YAML 配置并重新启动。

## 17. 错误码

普通命令失败时返回：

~~~text
error:n
~~~

常用错误码：

| 错误码 | 含义 |
|---:|---|
| 1 | 期待 G-code 命令字母 |
| 2 | G-code 数字格式错误 |
| 3 | 无效的 $ 命令 |
| 4 | 参数不允许为负数 |
| 8 | 命令要求机器处于 Idle |
| 9 | 报警或锁定状态禁止执行 G-code |
| 10 | 软限位错误 |
| 11 | 命令行过长 |
| 12 | 最大步进速率超限 |
| 13 | 检查安全门 |
| 14 | 启动行过长 |
| 18 | 未配置回零周期 |
| 19 | 不允许单轴回零 |
| 20 | 不支持的 G-code |
| 21 | 同一模态组命令冲突 |
| 22 | 未定义进给速度 |
| 23 | 命令值应为整数 |
| 24 | 轴命令冲突 |
| 25 | 同一参数字重复 |
| 26 | 缺少轴参数 |
| 27 | 行号无效 |
| 28 | 缺少必要参数 |
| 29 | 不支持的坐标系 |
| 30 | G53 使用了无效运动模式 |
| 31 | 出现不允许的轴参数 |
| 32 | 当前平面缺少轴参数 |
| 33 | 无效目标 |
| 34 | 圆弧半径错误 |
| 35 | 圆弧平面缺少偏移 |
| 36 | 存在未使用参数 |
| 38 | 数值超过允许范围 |
| 40 | 控制引脚状态异常 |
| 120 | 另一个接口正在占用 |
| 152 | 配置无效 |

如果启用了详细错误，<code>error:n</code> 后可能还有文本错误说明。上位机仍应以 <code>error:n</code> 作为当前命令失败的正式标志。

## 18. 报警码

报警输出格式：

~~~text
ALARM:n
~~~

| 报警码 | 含义 |
|---:|---|
| 1 | 硬限位触发 |
| 2 | 软限位触发 |
| 3 | 运动周期中止 |
| 4 | 探针初始状态错误 |
| 5 | 探针未接触目标 |
| 6 | 回零过程中复位 |
| 7 | 回零过程中安全门触发 |
| 8 | 回零拉脱失败 |
| 9 | 回零接近失败 |
| 10 | 主轴控制错误 |
| 11 | 控制引脚启动时有效 |
| 12 | 回零开关状态有歧义 |
| 13 | Hard Stop |
| 14 | 轴未回零 |
| 15 | 初始化报警 |
| 16 | 串口 IO 扩展器复位 |

收到任何 ALARM 后：

1. 立即停止发送普通运动命令。
2. 记录报警码。
3. 发送 <code>?</code> 获取状态。
4. 根据原因决定回零、复位、检查限位或人工处理。
5. 不要无条件循环发送 $X。

## 19. 超时和重发

### 19.1 普通运动命令

单条 G1 命令在只有一条待确认命令时，通常会较快返回 ok。

建议：

- 普通命令确认超时：1 至 2 秒，可配置。
- 状态查询超时：500 ms 至 1 秒。
- 运动完成超时：根据距离和 F 计算，并增加余量。
- 回零超时：按机器尺寸和回零速度单独设置，通常远大于普通命令超时。

运动理论时间可粗略估算：

~~~text
距离 = sqrt(dx^2 + dy^2 + dz^2)
理论时间分钟 = 距离 / F
~~~

实际还需要考虑加减速，因此超时必须大于理论时间。

### 19.2 未收到 ok 时

不要立刻重复发送同一命令。

正确处理：

1. 停止发送新普通命令。
2. 继续接收，避免应答只是延迟。
3. 发送单字节 <code>?</code> 查询机器状态。
4. 如果状态为 Run，原命令可能已经被接受并正在运行。
5. 如果状态为 Idle，但命令结果未知，比较当前位置和目标。
6. 无法确认时进入人工或重新同步流程。

虽然绝对坐标目标通常具有一定幂等性，但重复发送仍可能造成队列、日志和状态判断混乱。

### 19.3 断线重连

重连后：

1. 不要假设断线前的命令未执行。
2. 不要直接重发最后一条命令。
3. 发送 <code>?</code> 获取状态和位置。
4. 若机器仍在 Run，等待或执行安全停止策略。
5. 若状态为 Idle，核对报告位置。
6. 若状态为 Alarm，根据报警处理。
7. 需要可信机器坐标时重新回零。

## 20. 为什么不能每 10 ms 重复发送运动命令

不推荐以下行为：

~~~text
每10ms发送一次：
G21 G90 G94 G53 G1 X300 Y200 Z1 F300\r\n
~~~

原因：

- 这相当于每秒发送 100 条普通命令。
- Grbl 协议要求普通命令受 ok/error 流控。
- UART 接收缓冲区和命令处理任务可能积压。
- 第一条命令已经建立相同目标，后续重复目标没有运动价值。
- 上位机忽略 error 或 ALARM 后，无法知道机器是否运动。
- 如果未来目标发生变化，旧目标会形成排队和控制延迟。
- G-code 规划队列不是 100 Hz 实时位置闭环接口。

正确做法：

~~~text
发送一次目标
    |
等待 ok
    |
每100ms发送 ?
    |
等待 Idle
    |
再发送新目标
~~~

如果应用必须每 10 ms 更新位置设定值，应设计专用实时控制协议并修改 FluidNC 固件，或者使用适合位置伺服的控制器，而不是把 G-code 队列当作实时设定值寄存器。

## 21. 上位机推荐状态机

~~~text
POWER_ON
   |
   v
UART_OPEN
   |
   v
WAIT_READY
   | 收到 "Grbl ..." 或合法 <...>
   v
QUERY_INITIAL_STATE
   | 发送 ?
   |
   +-- Alarm ------> RECOVER_ALARM
   +-- Idle -------> READY
   +-- 其他状态 ---> WAIT_OR_STOP

RECOVER_ALARM
   |
   +-- 需要回零 ---> 发送 $H -> 等待完成 -> QUERY_INITIAL_STATE
   +-- 可解锁 -----> 发送 $X -> 等待 ok -> QUERY_INITIAL_STATE
   +-- 配置错误 ---> FAULT

READY
   |
   | 获得新的绝对目标
   v
SEND_MOTION
   | 发送一条 G21 G90 G94 G53 G1 ...
   v
WAIT_ACK
   |
   +-- ok ---------> MONITOR_MOTION
   +-- error:n ----> COMMAND_ERROR
   +-- ALARM:n ----> FAULT
   +-- Grbl ... ---> RESYNC

MONITOR_MOTION
   | 每100ms发送 ?
   |
   +-- Run --------> MONITOR_MOTION
   +-- Hold/Door --> PAUSED
   +-- Alarm ------> FAULT
   +-- Idle -------> VERIFY_POSITION

VERIFY_POSITION
   |
   +-- 位置正确 ---> READY
   +-- 位置异常 ---> FAULT
~~~

## 22. 上位机接收解析伪代码

~~~c
void on_uart_byte(uint8_t byte)
{
    if (byte == '\r') {
        return;
    }

    if (byte != '\n') {
        if (rx_length < RX_LINE_MAX - 1) {
            rx_line[rx_length++] = (char)byte;
        } else {
            protocol_fault = RX_LINE_OVERFLOW;
        }
        return;
    }

    rx_line[rx_length] = '\0';
    rx_length = 0;

    if (rx_line[0] == '\0') {
        return;
    }

    process_fluidnc_line(rx_line);
}

void process_fluidnc_line(const char *line)
{
    if (strcmp(line, "ok") == 0) {
        if (command_pending) {
            command_result = COMMAND_OK;
            command_pending = false;
        } else {
            log_unsolicited_ack(line);
        }
        return;
    }

    if (starts_with(line, "error:")) {
        if (command_pending) {
            command_error_code = parse_integer(line + 6);
            command_result = COMMAND_ERROR;
            command_pending = false;
        } else {
            log_unsolicited_error(line);
        }
        return;
    }

    if (starts_with(line, "ALARM:")) {
        alarm_code = parse_integer(line + 6);
        controller_state = CONTROLLER_ALARM;
        stop_sending_motion();
        return;
    }

    if (line[0] == '<' && line[strlen(line) - 1] == '>') {
        parse_status_report(line);
        return;
    }

    if (starts_with(line, "Grbl ")) {
        clear_pending_commands();
        controller_state = CONTROLLER_RESTARTED;
        return;
    }

    if (line[0] == '[') {
        process_information_message(line);
        return;
    }

    log_unknown_line(line);
}
~~~

## 23. 逐次绝对运动伪代码

~~~c
bool move_machine_absolute(
    float x,
    float y,
    float z,
    float feed_mm_per_min)
{
    char command[128];

    if (controller_state != CONTROLLER_IDLE) {
        return false;
    }

    snprintf(
        command,
        sizeof(command),
        "G21 G90 G94 G53 G1 X%.3f Y%.3f Z%.3f F%.1f\n",
        x,
        y,
        z,
        feed_mm_per_min);

    clear_command_result();
    command_pending = true;
    uart_send_string(command);

    if (!wait_for_command_result(2000)) {
        enter_resync_state();
        return false;
    }

    if (command_result != COMMAND_OK) {
        return false;
    }

    uint32_t start_ms = millis();

    while (millis() - start_ms < calculate_motion_timeout()) {
        uart_send_byte('?');

        if (!wait_for_status_report(1000)) {
            enter_resync_state();
            return false;
        }

        if (controller_state == CONTROLLER_ALARM) {
            return false;
        }

        if (controller_state == CONTROLLER_IDLE) {
            return position_is_close(
                machine_x, x,
                machine_y, y,
                machine_z, z,
                0.01f);
        }

        delay_ms(100);
    }

    motion_timeout = true;
    return false;
}
~~~

## 24. 完整通信示例

### 24.1 启动并回零

~~~text
FluidNC -> Grbl 1.1... [FluidNC ... '$' for help]

上位机 -> ?
FluidNC -> <Alarm|MPos:0.000,0.000,0.000|FS:0,0>

上位机 -> $H\n
FluidNC -> <Home|MPos:...|FS:...>
FluidNC -> ok

上位机 -> ?
FluidNC -> <Idle|MPos:0.000,0.000,0.000|FS:0,0>
~~~

实际回零过程中，状态行和 ok 的具体先后与命令执行时机有关，上位机应分别解析，不应依赖所有消息严格相邻。

### 24.2 运动到 X=300、Y=200、Z=1

~~~text
上位机 -> G21 G90 G94 G53 G1 X300 Y200 Z1 F300\n
FluidNC -> ok

上位机 -> ?
FluidNC -> <Run|MPos:5.000,3.333,0.017|FS:300,0>

上位机 -> ?
FluidNC -> <Run|MPos:150.000,100.000,0.500|FS:300,0>

上位机 -> ?
FluidNC -> <Idle|MPos:300.000,200.000,1.000|FS:0,0>
~~~

### 24.3 下一次绝对运动

只有在上一条运动已经 Idle 后发送：

~~~text
上位机 -> G21 G90 G94 G53 G1 X100 Y50 Z1 F300\n
FluidNC -> ok
~~~

这里的 X100/Y50/Z1 是新的绝对机器目标，不是相对增量。

### 24.4 暂停与恢复

~~~text
上位机 -> !
上位机 -> ?
FluidNC -> <Hold:1|...>

上位机 -> ?
FluidNC -> <Hold:0|...>

上位机 -> ~
上位机 -> ?
FluidNC -> <Run|...>
~~~

## 25. 上位机实现检查清单

### 25.1 硬件

- [ ] UART 为 3.3V TTL。
- [ ] TX/RX 已交叉连接。
- [ ] 两端 GND 已连接。
- [ ] 不存在 USB 转串口芯片与外部 MCU 同时驱动 RX。
- [ ] 烧录固件时外部 MCU 不干扰 UART0。

### 25.2 接收器

- [ ] 支持 CRLF 和 LF。
- [ ] 接收使用环形缓冲区或中断/DMA。
- [ ] 单行缓冲区至少 256 字节。
- [ ] 能分类 ok、error、ALARM、状态、欢迎和信息消息。
- [ ] 不把 MSG 或状态行当作 ok。
- [ ] 收到 Grbl 欢迎行后清空待确认事务。

### 25.3 发送器

- [ ] 普通命令末尾带单个 LF；若使用 CRLF，能忽略额外空行应答。
- [ ] 同时最多一条未确认普通命令。
- [ ] 收到 ok/error 前不发送下一条普通命令。
- [ ] 运动完成前不发送下一个绝对目标。
- [ ] 状态查询使用单字节问号，不加 CRLF也可以。
- [ ] 不以 10 ms 周期重复发送相同运动命令。

### 25.4 安全

- [ ] 使用 G53 前已完成回零。
- [ ] 已确认机器坐标正负方向和允许范围。
- [ ] 已确认 X=300、Y=200、Z=1 不会超出机械行程。
- [ ] 已配置合理的最大速度、加速度和软限位。
- [ ] 收到 Alarm 或 error 后停止自动运动。
- [ ] 通信中断时采用明确的安全策略。

## 26. 与本协议直接相关的源码

| 功能 | 文件 |
|---|---|
| UART0 初始化 | [FluidNC/src/UartChannel.cpp](FluidNC/src/UartChannel.cpp) |
| UART 抽象读写 | [FluidNC/src/Uart.cpp](FluidNC/src/Uart.cpp) |
| ESP32 UART 底层 | [FluidNC/esp32/uart.cpp](FluidNC/esp32/uart.cpp) |
| 通道行解析和 ok/error | [FluidNC/src/Channel.cpp](FluidNC/src/Channel.cpp) |
| 多通道轮询 | [FluidNC/src/Serial.cpp](FluidNC/src/Serial.cpp) |
| 主协议循环 | [FluidNC/src/Protocol.cpp](FluidNC/src/Protocol.cpp) |
| 普通命令分类 | [FluidNC/src/ProcessSettings.cpp](FluidNC/src/ProcessSettings.cpp) |
| G-code 解析与执行 | [FluidNC/src/GCode.cpp](FluidNC/src/GCode.cpp) |
| 实时命令定义 | [FluidNC/src/RealtimeCmd.h](FluidNC/src/RealtimeCmd.h) |
| 状态报告生成 | [FluidNC/src/Report.cpp](FluidNC/src/Report.cpp) |
| 错误码定义 | [FluidNC/src/Error.h](FluidNC/src/Error.h) |
| 报警码定义 | [FluidNC/src/Protocol.h](FluidNC/src/Protocol.h) |
| 当前 TMC2209 示例配置 | [example_configs/4x_2209_atc.yaml](example_configs/4x_2209_atc.yaml) |

## 27. 最终推荐

对于“上位机逐次控制机器运动到一个绝对机器坐标”的需求，建议固定使用以下策略：

~~~text
1. 启动后等待 Grbl 欢迎消息或状态响应。
2. 回零，建立可信机器坐标。
3. 确认状态为 Idle。
4. 发送一条：
   G21 G90 G94 G53 G1 X... Y... Z... F...\n
5. 等待 ok 或 error。
6. 收到 ok 后，每 100 ms 发送一次 ?。
7. 收到 Idle 并核对位置。
8. 再发送下一个绝对目标。
9. 收到 error 或 ALARM 时停止自动控制。
10. 不重复高频发送相同运动命令。
~~~

目标 X=300、Y=200、Z=1 的最终命令为：

~~~text
G21 G90 G94 G53 G1 X300 Y200 Z1 F300\n
~~~
