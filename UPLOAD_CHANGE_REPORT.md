# 本次上传改动说明

1. 新增灰度 ADC 调试模式 `PROJECT_MODE_GRAY_ADC_DEBUG`，并设为当前默认模式。
2. 新增 SSD1306/SH1106 OLED 驱动，OLED 使用 I2C0：PA0=SDA、PA1=SCL。
3. 在 OLED 上实时显示 8 路灰度传感器的 ADC 原始值。
4. 在 OLED 上显示灰度模块 EN、ERR 引脚电平、ADC 读取状态和递增帧号。
5. 增加灰度模块引脚配置：AD0=PA24、AD1=PA25、AD2=PA26、OUT=PA27、EN=PB24、ERR=PB25。
6. 将 ADC 配置为重复单通道模式；每路切换后等待 10 us，并采样 4 次取平均。
7. 修复平台层灰度 GPIO 端口宏名称错误。
8. 将工程使用的 TI Arm Clang 版本调整为本机可用的 4.0.4 LTS。
9. 新增 `PROJECT_MEMORY.md`，用于持续记录项目状态和调试结果。
