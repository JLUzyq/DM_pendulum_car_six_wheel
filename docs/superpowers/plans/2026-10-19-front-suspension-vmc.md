# 前轮悬挂与差速运动学实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 更新六轮差速模型以支持不同轴距，接入两台前摆臂电机的 G36 反馈，并提供由摆臂角度计算轮心虚拟力及电机力矩的独立 VMC 模块。

**Architecture:** `vehicle_config.h` 保存前/中/后轴距及摆臂电机 CAN 配置；`kinematics.c` 对每个车轮按本轴轮距分配速度，并以六轮反馈最小二乘估计车体速度。CAN 层把摆臂反馈写入独立状态，`vmc.c/.h` 提供参数化的刚性摆臂正运动学和虚拟力到关节力矩换算，不在本次改动中将摆臂力矩接入实时控制任务。

**Tech Stack:** STM32H723、C11、G36 MIT 电机驱动、CMake/Ninja GCC 工程、Keil MDK 工程。

---

### Task 1: 分轴距运动学

**Files:**
- Modify: `User/APP/vehicle_config.h`
- Modify: `User/APP/kinematics.h`
- Modify: `User/APP/kinematics.c`
- Modify: `User/APP/chassis_task.c`

- [x] 在 `vehicle_config.h` 用前、中、后轮距常量替换单一轮距常量，分别设为 `0.375f`、`0.535f`、`0.535f`。
- [x] 将运动学设点接口改为接收车体线速度和偏航角速度；根据每个 `wheel_index_e` 的轴位和左右侧符号计算各自轮速。
- [x] 正运动学把每个反馈角速度换算成带安装方向的接地速度，再按 `v_i = v_x + s_i * omega * track_i/2` 最小二乘求 `v_x` 与 `omega`；同时保持左右平均速度输出。
- [x] 更新 `chassis_task.c` 调用点，直接传入闭环后的 `yaw_rate_control`，去掉用单一轮距计算共享 `turn_out` 的旧路径。
- [x] 保留现有线速度、电机速度限幅和安装方向逻辑。

### Task 2: 摆臂电机配置与反馈路由

**Files:**
- Modify: `User/APP/vehicle_config.h`
- Modify: `User/APP/chassis_task.h`
- Modify: `User/Bsp/can_bsp.c`

- [x] 定义左/右摆臂索引和配置：左侧 CAN1 命令 ID `0x07`、反馈 ID `0x17`；右侧 CAN2 命令 ID `0x08`、反馈 ID `0x18`。
- [x] 在底盘状态中增加两个独立的 `HM_Motor_t` 摆臂反馈对象。
- [x] 将 CAN 分发逻辑扩展为先匹配总线和反馈 ID，再更新对应驱动轮或摆臂对象；所有匹配仍要求 8 字节有效帧。
- [x] 不在当前控制任务中使能摆臂电机或发送 VMC 力矩。

### Task 3: 参数化 VMC 模块

**Files:**
- Create: `User/Algorithm/VMC/vmc.h`
- Create: `User/Algorithm/VMC/vmc.c`

- [x] 定义每侧参数结构，包含摆臂长度（实测 `0.070f` m）、电机零位角（调用方提供）、电机方向符号、`kp`、`kd`、`xref` 和 `G_qiankui`。
- [x] 暴露单侧计算函数，输入参数与 G36 反馈的电机位置/速度，输出结构包含轮心位置、轮心速度、雅可比、虚拟力和目标电机力矩。
- [x] 使用 `theta = direction*q + zero_offset`、`x=L*sin(theta)`、`v=L*cos(theta)*direction*dq`、`J=L*cos(theta)*direction`、`F=kp*(xref-x)-kd*v+G_qiankui`、`tau=J*F`。
- [x] 对空指针、非有限输入、非正摆臂长度、非法方向和负增益返回失败；失败时不覆盖输出结果。
- [x] 保持模块不依赖 HAL、CAN 或实时任务，便于后续标定和独立复用。

### Task 4: 工程文件登记与构建检查

**Files:**
- Modify: `CMakeLists.txt`
- Modify: `MDK-ARM/CtrlBoard-H7_IMU.uvprojx`

- [x] 将 `User/Algorithm/VMC/vmc.c` 添加到 GCC 用户源文件列表。
- [x] 将 `vmc.c` 添加到 Keil Algorithm 分组，并确认对应头文件可通过现有 include 路径找到。
- [x] 运行现有 Debug 构建，检查新增源文件、接口变更和链接结果。

## 实施说明

本计划不新增或运行测试套件；按仓库现有构建流程做编译检查。工作目录当前没有 Git 元数据，因此实施过程中无法创建 Git 提交。
