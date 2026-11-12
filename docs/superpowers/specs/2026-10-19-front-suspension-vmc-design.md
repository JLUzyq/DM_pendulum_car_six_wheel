# 前轮悬挂与差速运动学设计

## 目标

适配前轮距 375 mm、中轮与后轮距 535 mm 的六轮差速底盘；新增两台前轮摆臂 G36 的参数与反馈路由；建立独立的 VMC 算法接口，使摆臂关节角度与速度可换算为轮心状态和电机目标力矩。

## 底盘几何与运动学

车辆配置区分前、中、后三轴轮距：前轴 0.375 m，中轴和后轴均为 0.535 m。逆运动学根据车体线速度和偏航角速度，按各车轮所在轴的半轮距计算左右轮线速度，再换算成六台驱动轮电机角速度。正运动学使用六个轮子的反馈线速度及各自轮距做最小二乘估计，输出车体纵向速度和偏航角速度。现有轮径、安装方向、速度限幅及 G36 驱动协议保持现有约定。

## 摆臂电机配置与反馈

左摆臂 G36 接 CAN1，命令 ID 为 0x07、反馈 ID 为 0x17；右摆臂 G36 接 CAN2，命令 ID 为 0x08、反馈 ID 为 0x18。两台电机的反馈进入独立的摆臂电机状态对象，不混入六个驱动轮反馈数组。使用既有 G36 MIT 编码和解码协议。

本次仅增加配置、反馈路由和可复用 VMC 算法接口；不在现有实时底盘任务中发送摆臂力矩，以免零位尚未标定时直接控制机构。

## VMC 模型与接口

VMC 以每侧的摆臂电机位置 `q`（rad）和速度 `dq`（rad/s）为输入，摆臂长度固定实测为 0.070 m，电机零位偏置和方向符号由每侧配置提供。刚性摆臂模型按轮心相对摆臂转轴的竖直位移计算：

```text
theta = direction * q + zero_offset
x_current = arm_length * sin(theta)
v_current = arm_length * cos(theta) * direction * dq
J = dx_current/dq = arm_length * cos(theta) * direction
F_vmc = kp * (xref - x_current) - kd * v_current + G_qiankui
tau = J * F_vmc
```

`zero_offset` 的定义是电机读数为零时摆臂相对水平线的角度；`xref` 与 `x_current` 都表示相对摆臂转轴的竖直位移。每侧分别输入 `xref`（m）、`kp`（N/m）、`kd`（N·s/m）和 `G_qiankui`（N）。`G_qiankui` 作为轮心竖直方向前馈力由调用方提供。算法输出轮心位置、轮心速度、虚拟力、雅可比和电机目标力矩；电机实际发送及零位标定流程不在本次范围内。

## 文件边界

- `User/APP/vehicle_config.h`：轴距、摆臂电机总线/ID/回复 ID、摆臂几何默认长度和电机方向配置。
- `User/APP/kinematics.h`、`User/APP/kinematics.c`：六轮分轴距逆解与正解接口。
- `User/APP/chassis_task.h`：保存两台摆臂电机反馈状态。
- `User/Bsp/can_bsp.c`：依据 CAN 总线和回复 ID 分发六个驱动轮及两台摆臂的反馈。
- `User/Algorithm/VMC/vmc.h`、`User/Algorithm/VMC/vmc.c`：独立的摆臂几何、弹簧阻尼力和雅可比力矩计算。
- `CMakeLists.txt`、`MDK-ARM/CtrlBoard-H7_IMU.uvprojx`：将新增 VMC 源文件纳入 GCC 和 Keil 工程。

## 错误与边界处理

反馈帧仅在总线、回复 ID 和 8 字节帧长匹配时写入对应状态。VMC 初始化拒绝非正摆臂长度；计算接口对空指针、非有限输入或无效几何配置返回失败，不输出新的力矩。电机零位由调用方传入并可在标定后更新。MIT 电机量程裁剪仍由现有电机编码器负责。

## 验收标准

1. 前、中、后轴分别按 0.375、0.535、0.535 m 参与运动学分配和反馈估计。
2. CAN1 0x17 只更新左摆臂状态，CAN2 0x18 只更新右摆臂状态；命令 ID 为 0x07、0x08。
3. 摆臂水平位置处 `x_current=0`、`J` 的符号由方向配置决定，位置/速度/力矩计算符合上述公式。
4. 非法输入返回失败且不产生新的目标力矩。
5. GCC 与 Keil 工程都包含新增 VMC 源文件。
