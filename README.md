# MSPM0G3507 小车综合工程

## 当前默认模式：灰度 ADC 调试

`project_mode.h` 当前选择 `PROJECT_MODE_GRAY_ADC_DEBUG`。该模式保持电机输出关闭，
依次选择 8 路灰度传感器并在 OLED 上显示原始 12 位 ADC 数字。

| 功能 | 引脚 |
| --- | --- |
| 灰度 AD0/AD1/AD2 | PA24 / PA25 / PA26 |
| 灰度 OUT | PA27 / ADC0 channel 0 |
| 灰度 EN | PB24，低电平使能 |
| 灰度 ERR | PB25 |
| OLED SCL/SDA | PA1 / PA0，I2C0，100 kHz |

OLED 优先尝试地址 `0x3C`，地址无应答时再尝试 `0x3D`。最后一行的 `F:n`
持续增加表示主循环仍在刷新；`ADC OK` 表示本轮 8 路采样全部完成，`ADC ERR`
表示至少一次 ADC 等待超时。

切换功能时修改 `project_mode.h` 中的 `PROJECT_MODE`：

- `PROJECT_MODE_BLUETOOTH_TUNING`：原 TB6612/蓝牙测试。
- `PROJECT_MODE_H2024_ITEM_1` 至 `PROJECT_MODE_H2024_ITEM_4`：赛题逻辑。
- `PROJECT_MODE_GRAY_ADC_DEBUG`：8 路原始 ADC + OLED 调试。

注意：拓展板文档预留的 OLED `PB2/PB3` 当前与 `UART_BLUETOOTH` 冲突。本调试模式
沿用已上板验证过的 `I2C0 PA0/PA1` 接线，不能把 OLED 同时接到 `PB2/PB3`。

## TB6612-1 开环双轮测试

本工程只测试拓展板上的 `TB6612-1`。当前没有编码器测速，也没有 PID；VOFA+ 中显示的是控制命令，不是实际转速。

## 固定引脚

| 功能 | MSPM0G3507 引脚 |
| --- | --- |
| 左轮 PWMA | PB12 / TIMA0_CCP1 / 20 kHz |
| 左轮 AIN1、AIN2 | PA14、PA15 |
| 右轮 PWMB | PB4 / TIMA0_CCP2 / 20 kHz |
| 右轮 BIN1、BIN2 | PA16、PA17 |
| TB6612-1 STBY | PA28 |
| 蓝牙 TX、RX | PA8、PA9 / UART1 / 115200 8N1 |
| VOFA+ 调试 TX、RX | PA10、PA11 / UART0 / 115200 8N1 |

外部串口必须交叉连接，并且共地：`PA8 -> 蓝牙 RX`、`PA9 <- 蓝牙 TX`、`GND -- GND`。TB6612 的逻辑电源接 3.3 V，电机电源 VM 按电机额定电压连接，主控、驱动板和电机电源必须共地。

## 蓝牙命令

| 命令 | 动作 |
| --- | --- |
| `F` | 前进 |
| `B` | 后退 |
| `L` | 原地左转 |
| `R` | 原地右转 |
| `S`、`X`、`0` | 停止 |
| `+`、`-` | 占空比增加、减少 5% |

默认占空比 20%，最低 10%，最高限制 60%。大小写均可，回车和换行会被忽略。运动命令 2 秒内没有刷新时会自动停车；长时间测试需要每隔小于 2 秒重复发送当前运动命令。

上电时 PWM 为 0%，四个方向脚为 Low，STBY 为 Low。停车也会恢复到同一状态。每次改变动作前，程序会先清零两路 PWM 并拉低 STBY，再切换方向。

## VOFA+ FireWater

板载 CH340 对应 UART0。VOFA+ 选择正确 COM 口，设置 `115200, 8 data bits, 1 stop bit, no parity, no flow control`，协议选择 `FireWater`。程序每 100 ms 发送一行纯 ASCII 数字：

```text
state,left_cmd,right_cmd,duty_percent,rx_count,error_count,failsafe_count,uptime_ms\r\n
```

例如前进 20%：

```text
1,20,20,20,1,0,0,530
```

`state`：0=停止、1=前进、2=后退、3=左转、4=右转。`left_cmd/right_cmd` 是有符号占空比命令，不是编码器速度；FireWater 可以直接解析整数，不需要发送二进制 float。

## 第一次悬空测试

1. 先断开电机 VM，只给主控和 TB6612 逻辑侧供电并烧录。
2. 复位后确认电机不会自动转，VOFA+ 应持续看到 `state=0`、左右命令为 0。
3. 架空两个车轮，再接通电机 VM；先发送一次 `F`，两轮只以 20% 运行，并应在 2 秒后自动停止。
4. 运行期间发送 `S`，应立即停车；再分别短测 `B`、`L`、`R`。
5. 若 `F` 时只有一侧车辆前进方向相反，只把 `tb6612.h` 中对应的 `TB6612_*_FORWARD_IN1_HIGH` 从 1 改成 0，重新编译烧录。
6. 开环启停和方向全部连续通过 10 次后，再接编码器并建立速度环。

## 双路增量编码器

| 信号 | MSPM0G3507 引脚 | 用法 |
| --- | --- | --- |
| 左轮 A | PA7 / `ENC_A_PIN_AL_PIN` | 双边沿 GPIO 中断 |
| 左轮 B | PA22 / `ENC_B_PIN_BL_PIN` | GPIO 电平输入 |
| 右轮 A | PA30 / `ENC_A_PIN_AR_PIN` | 双边沿 GPIO 中断 |
| 右轮 B | PA31 / `ENC_B_PIN_BR_PIN` | GPIO 电平输入 |

`encoder.c/.h` 使用 x2 解码：每个 A 相上升沿和下降沿都计数一次，中断入口为 GPIOA 对应的 `GROUP1_IRQHandler()`。主程序初始化：

```c
#include "encoder.h"

SYSCFG_DL_init();
Encoder_Init();
```

累计计数可直接读取：

```c
int32_t leftTotal  = Encoder_GetLeftCount();
int32_t rightTotal = Encoder_GetRightCount();
```

固定周期测速时读取增量，不会清除累计里程：

```c
int32_t leftTicks  = Encoder_GetLeftDelta();
int32_t rightTicks = Encoder_GetRightDelta();
```

例如每 10 ms 调用一次，返回值就是该 10 ms 内的计数增量。首次调用的基准为 `Encoder_Init()` 时的 0；需要重新开始测量时调用 `Encoder_ResetCounts()`。若车辆前进时某侧累计值为负，只将 `encoder.h` 中对应的 `ENC_LEFT_SIGN` 或 `ENC_RIGHT_SIGN` 从 `+1` 改成 `-1`。

当前 `GROUP1_IRQHandler()` 由编码器模块占用。后续若增加其他 GPIOA 中断，必须合并到同一个入口里分发，不能再定义第二个同名中断函数。
