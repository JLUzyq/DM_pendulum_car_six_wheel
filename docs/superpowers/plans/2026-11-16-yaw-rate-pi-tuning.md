# 偏航角速度 PI 在线调参与遥测 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** 在保留原运动帧和控制路径的前提下，增加默认关闭、可心跳超时关闭的偏航角速度遥测，以及可在线设置 PI 增益的 Chrome Web Serial 调参页。

**Architecture:** USART1 复用现有 DMA 接收通道解析旧运动帧和 14 字节调参帧。调参配置以 Modbus CRC-16 校验，参数存 RAM；底盘任务每 50 ms 尝试 DMA 上传理论值和 IMU 实测值。独立 HTML 页面使用 Web Serial 和 Canvas 绘制 10 秒曲线，并管理串口、遥测心跳、增益配置及 CSV 导出。

**Tech Stack:** STM32H723、STM32 HAL UART DMA、FreeRTOS、C11、HTML/CSS/原生 JavaScript、Chrome Web Serial API。

---

## 文件与职责

- `User/APP/Chaiss_Communication.h/.c`：调参帧格式、混合流解析、CRC 校验、RAM 增益/遥测状态、配置确认和非阻塞遥测发送。
- `User/APP/vehicle_config.h`：运行时 Kp/Ki 默认值与允许范围常量。
- `User/APP/chassis_task.c`：PI 运算读取在线增益；以 20 Hz 将斜坡后理论值和滤波 IMU 值送给通信层。
- `User/APP/chassis_task.h`：如需，明确观测值的字段语义；避免另建重复控制状态。
- `tuning/yaw_rate_tuner.html`：单页 Web Serial UI、帧编码/解码、Canvas 曲线和 CSV 导出。
- `tuning/README.md`：Chrome/localhost/USB 转串口操作、SBUS 调参控制方式和增益范围。
- `底盘串口通信协议.md`：增加新调参帧与遥测帧章节，明确旧运动帧保持不变。

## Task 1: 增加调参协议与状态

**Files:** 修改 `User/APP/Chaiss_Communication.h`、`User/APP/Chaiss_Communication.c`、`User/APP/vehicle_config.h`。

- [x] 在 `vehicle_config.h` 定义 Kp/Ki 默认值和边界：默认沿用 `YAW_RATE_KP=0.30f`、`YAW_RATE_KI=0.00f`，允许 `KP 0..10`、`KI 0..1`。
- [x] 在通信头文件声明 14 字节调参帧常量、类型码（配置 `0x01`、遥测 `0x81`、写入确认 `0x82`、查询确认 `0x83`）、flags 常量、`Chaiss_YawRateTuningConfig_t` 及读配置/处理 TX API。
- [x] 在通信实现中加入 Modbus CRC-16（初值 `0xFFFF`、多项式 `0xA001`），覆盖帧字节 1..10，CRC 低字节先发；加入小端 float 读写。
- [x] 将流解析按首字节分派：旧帧仍要求 `0xA5`/`0x4A` 并沿用现有命令使能门控；调参帧要求 `0xC3`/`0x3C`、14 字节、合法 CRC，并独立于运动命令门控解析。未知首字节继续丢弃一个字节并重同步。
- [x] 配置帧布局为 `C3 01 flags Kp[4] Ki[4] CRC[2] 3C`，只允许 bit0（遥测使能）、bit1（写入增益）和 bit2（查询当前增益）。CRC 正确且保留位为 0 时更新遥测状态/心跳；仅 bit1=1 时验证并更新有限且在边界内的 Kp/Ki，bit1=0 时忽略帧内增益。bit1 请求回 `C3 82` 配置确认，bit2 请求回 `C3 83` 查询确认，心跳帧不确认；status=0 表示接受，非 0 表示拒绝并回报当前生效值。
- [x] 遥测上电默认关闭；加入读取配置 API，并在有效遥测心跳超过 1500 ms 后自动关闭。遥测布局为 `C3 81 seq target[4] actual[4] CRC[2] 3C`，数据单位 rad/s。
- [x] RX 回调只更新配置/心跳和有界 FIFO 中的待确认记录，不在中断中发送。增加通信层 TX 处理 API，由底盘任务调用：在 `huart1.gState == HAL_UART_STATE_READY` 时优先发送 FIFO 中最早的待确认帧，其次发送到期遥测帧；TX 忙时不等待并保留待确认记录，直到下次循环发出。

## Task 2: 接入 PI 在线参数与 20 Hz 遥测

**Files:** 修改 `User/APP/chassis_task.c`；必要时修改 `User/APP/chassis_task.h`。

- [x] 每个 5 ms 底盘循环从通信层读取 Kp、Ki，并在 PI 公式中用运行时值替代两个宏；保留原有误差定义、积分限幅、修正限幅和运动学前馈。
- [x] 每个 5 ms 底盘循环调用通信层 TX 处理 API，传入当前 tick、`yaw_rate_applied` 和 `yaw_rate_imu_filt`；通信层在遥测开启且距上次发送至少 50 ms 时发送遥测，在状态关闭时跳过遥测。
- [x] 每 5 ms 循环都更新 IMU Z 轴 LPF，使静止/失能时的实际角速度遥测保持实时；`yaw_rate_applied` 在失能分支仍按既有逻辑清零。
- [x] 每个循环都处理待确认帧；遥测仅受调参状态/心跳控制，即使车辆运动指令失能也可上传当前理论目标与 IMU 实测值。保留现有积分清零和电机放松行为。
- [x] 确认 TX DMA 发起失败或忙不会改变控制输出、阻塞 200 Hz 控制任务或影响 USART1 RX 重启。

## Task 3: 创建 Chrome Web Serial 调参页

**Files:** 创建 `tuning/yaw_rate_tuner.html`。

- [x] 创建暗色单页，顶部显示连接状态、串口连接/断开按钮和独立遥测开关；显示理论值、实际值、误差及已确认生效的 Kp/Ki。
- [x] 配置连接参数为 115200 baud、8 data bits、无校验、1 stop bit；在“连接串口”用户点击回调中调用 `navigator.serial.requestPort()`。不支持 Web Serial 时显示 Chrome/安全上下文提示。
- [x] 用 `ReadableStream` 循环读取字节，允许任意分片/合帧；按 `C3` 头、14 字节固定长度、`3C` 尾和 CRC 校验后解析确认帧和遥测帧。CRC 错误、未知类型不更新 UI。
- [x] 配置帧编码遵循 `C3 01 flags Kp[4] Ki[4] CRC[2] 3C`。启用遥测时设置 bit0+bit2 查询当前增益；关闭时只发 flags=0。500 ms 心跳只设置 bit0，不写入增益且不产生确认。用户点击应用时设置 bit1，并校验 Kp `0..10`/Ki `0..1`。页面按 `0x82`/`0x83` 区分写入确认和查询确认，待确认期间禁用另一个会产生确认帧的操作；与当前待确认类型不同的旧确认帧忽略。
- [x] 断开、串口读写异常或设备拔出时释放 reader/writer/port 锁，停止心跳和曲线更新，状态显示断开；MCU 由心跳超时自动停止上传。
- [x] 用原生 Canvas 实时绘制最多最近 10 秒的理论值/实测值双线，标注时间和 rad/s；支持暂停/继续、清空曲线和 CSV 下载，CSV 列为时间戳、理论角速度、实际角速度、误差。

## Task 4: 更新协议与使用说明

**Files:** 修改 `底盘串口通信协议.md`；创建 `tuning/README.md`。

- [x] 在协议文档区分旧 14 字节运动帧与新 14 字节调参帧，列出每个字节、CRC 范围/算法、频率、单位、超时和兼容性规则。
- [x] 说明 `localhost`/HTTPS 安全上下文、桌面 Chrome、USB 串口独占、115200 8N1；调参页不发送运动指令，调参期间通过 SBUS 驾驶并按现有 CH8 逻辑选择控制源。
- [x] 说明遥测默认关闭、500 ms 心跳、1500 ms 自动关闭、20 Hz 上传、Kp/Ki 范围和 RAM 参数重启恢复默认值。
- [x] README 给出从仓库根目录启动本地静态服务器的命令：`python -m http.server 8000`；页面 URL 为 `http://localhost:8000/tuning/yaw_rate_tuner.html`。

## 集成检查

- [x] 运行 `git diff --check`，确认新增改动无空白错误；在该 worktree运行 `cmake --preset Debug`，再运行 `cmake --build --preset Debug`，确认 STM32 固件可以编译。
- [x] 不新增或运行测试套件；当前用户请求没有要求测试。
- [x] 检查最终 diff，确认 `Core/Src/freertos.c` 及 `绘图文件/` 中已有未提交用户修改未被纳入或覆盖。
