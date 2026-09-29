# Horkin Core：设计说明

> 本文说明本目录骨架的**设计思想**，以及**每个目录 / 计划中的文件**要实现什么。
> 当前仓库是 CMake 空骨架：逻辑库多为 INTERFACE 占位，算法源码尚未落地。
> 理论与力控切分可对照外部文档（若有）：`stack_framework`、`force_control_framework`、`hybrid_force_position_control`。

---

## 1. 要解决什么问题

搭一套 **Horkin Core**：同一份控制律，接真机是实机，接仿真是仿真；换电机协议不必改 FK，换 DH 不必改 CAN，换仿真接触模型不必改导纳。

至少包含这些块：

| 块 | 职责一句话 |
|----|------------|
| `types` / `ports` | 公共数据与后端接口，最先冻结 |
| `kine` / `dyn` | 自建运动学、动力学，真机与仿真共用 |
| `signal` | 力补偿、滤波、坐标变换 |
| `control` | 力控 / 位控 / 轨迹（纯逻辑） |
| `runtime` | 定周期 RT 循环、缓冲、看门狗 |
| `hal` | 真机适配（CAN、电机、力传感器） |
| `sim` | 仿真植物与假驱动/假力觉 |
| `app_real` / `app_sim` | 两个装配根，唯一允许「接线」的地方 |
| `tools` | 标定、回放、烟雾测试 |

检验句：

> 删掉整个 `hal/`，只留 `app_sim`，控制仍能在弹簧环境里跑通。  
> 删掉整个 `sim/`，只留 `app_real`，同一份控制仍能编译。  
> 换几何只改 `kine/`，仿真末端与真机 FK 一起变。

---

## 2. 设计思想（为什么这样切）

### 2.1 依赖只许向下，端口是唯一「插头」

```
app_real ──► hal  ──► ports
app_sim  ──► sim  ──► ports
                ▲
runtime ──► control ──► kine / dyn / signal ──► types
```

- `control` / `kine` / `dyn` / `signal` **禁止**出现 SocketCAN、串口、ROS、仿真器、ODE/MuJoCo 头文件。
- `hal` 与 `sim` **平级**：都实现同一套 `ports`，互不引用。
- 只有 `app_*` 同时认识「控制」和「后端」，负责 `new` 与注入。
- 跨模块只传 **POD** 和 **端口指针**，没有全局 `Robot::instance()`。

这样换后端 = 换 wiring，而不是在控制里写 `if (sim)`。

### 2.2 先冻结契约，再并行实现

接口（`types` + `ports` + kine/dyn 函数签名 + 单位）写死之后，四条线可同时开工：运动学、动力学、控制、仿真/真机。  
原则是：**先签名后实现**——`fk()` 可以暂时返回假数据，下游照样编译。

### 2.3 仿真不是 if 分支，是另一套后端

- `SimJointDriver`：实现 `IJointDriver`；Write 收指令，内部积分植物，Read 回 `q,dq`。
- `SimForceSensor`：实现 `IForceSensor`；按接触模型给出**传感器系原始力**，再走与真机同一条 `signal/` 补偿链。

不要给仿真单独做「已经补偿好的完美力」接口，否则补偿 bug 只能在真机上暴露。

### 2.4 一事一文件，文件保持短

- 一个公开函数或一个小类型 = 一对 hpp/cpp。
- 实现以约 150～300 行为宜，超过约 400 行就拆。
- 禁止 `utils.cpp` / `common.cpp` 垃圾桶。
- `kine/fk.cpp` 不要顺便算雅可比；`sim/plant` 不要又积分又接触又绘图。

### 2.5 单位与关节约定（写进 types，全栈遵守）

| 量 | 约定 |
|----|------|
| 转动关节 | rad、rad/s、N·m |
| J3（直线轴） | **米、m/s、牛顿**（逻辑层永远不见丝杠圈数） |
| 力旋量 | N、N·m；离开 `signal/` 之后一律 **TCP 系、已补偿** |
| 时间 | 单调时钟纳秒 |
| 姿态 | 计算用旋转矩阵，插值用四元数（转换函数单独文件） |

`144 mm/转` 这类传动比**只允许**出现在 `hal/` 的传动文件；仿真植物在关节空间积分时，J3 同样用米。

### 2.6 配置按「谁改谁读」拆分

见 `config/` 一节。仿真读 `world.yaml`，真机读 `can.yaml`；两边都读 `robot.yaml` / `admit.yaml`。控制函数收 `const Params&`，**自己不读文件**。

### 2.7 CMake 目标与目录同构

每个目录一个 `CMakeLists.txt`，产出一个 `horkin_*` target。依赖用 `target_link_libraries` 表达，而不是乱 include 路径。  
逻辑库现阶段是 **INTERFACE 占位**；第一个 `.cpp` 落地时改为 `STATIC`，并在对应 `CMakeLists.txt` 的注释示例中取消注释。

Include 约定（`HORKIN_INCLUDE_ROOT` = 本仓库根）：

```cpp
#include "types/wrench.hpp"
#include "ports/force_sensor.hpp"
```

---

## 3. 目录与 CMake 目标一览

| 目录 | CMake 目标 | 类型（现状 → 目标） | 允许依赖 | 禁止依赖 |
|------|------------|---------------------|----------|----------|
| `types/` | `horkin_types` | INTERFACE | 无 | 一切实现 |
| `ports/` | `horkin_ports` | INTERFACE | `horkin_types` | 硬件、算法 |
| `kine/` | `horkin_kine` | INTERFACE → STATIC | `types` | control、hal、sim、ROS |
| `dyn/` | `horkin_dyn` | INTERFACE → STATIC | `kine` | control、hal、sim |
| `signal/` | `horkin_signal` | INTERFACE → STATIC | `types` | hal、sim、control 公式 |
| `control/force/` | `horkin_control_force` | INTERFACE → STATIC | `types`（及 kine/dyn 调用） | hal、sim |
| `control/` | `horkin_control` | INTERFACE 聚合 | `horkin_control_force` | — |
| `runtime/` | `horkin_runtime` | INTERFACE → STATIC | ports + control + signal + kine + dyn | 具体 hal/sim 实现 |
| `hal/` | `horkin_hal` | INTERFACE → STATIC | `ports` | sim、控制公式 |
| `sim/` | `horkin_sim` | INTERFACE → STATIC | ports、kine、dyn | hal |
| `app_real/` | `app_real` | 可执行文件 | runtime + hal | sim |
| `app_sim/` | `app_sim` | 可执行文件 | runtime + sim | hal |
| `tools/` | 各工具可执行文件 | 按需 | ports / signal 等 | 视工具而定 |

根选项见 `cmake/HorkinOptions.cmake`：`HORKIN_BUILD_SIM` / `HAL` / `TOOLS` / `TESTS`。

---

## 4. 各目录职责与计划文件

下列文件名是**计划实现清单**。骨架里可能尚未创建 `.hpp/.cpp`，以本表为准往里长。

### 4.1 `types/` — 零依赖 POD

**思想：** 全栈共享的「形状」；接口可变，数据尽量不变。

| 计划文件 | 功能 |
|----------|------|
| `time.hpp` | `TimeNs`：单调时钟纳秒；采样时间戳用采集时刻 |
| `wrench.hpp` | `Vector6` / `Wrench` / `WrenchSample`；布局 `[fx,fy,fz,mx,my,mz]`，单位 N / N·m |
| `joint.hpp` | `JointVec`、`JointState`（q/dq/tau）；**`q[2]` 永远是丝杠位移（米）** |
| `command.hpp` | `JointCommand`：q、dq、tau_ff、kp、kd（MIT 式） |
| `pose.hpp` | 位置 + 旋转矩阵（或另文件做四元数转换） |
| `errors.hpp` | 超时、超限、看门狗等错误枚举 |
| `admit_state.hpp` | 导纳内部状态 e、v（显式传入传出） |
| `contact_state.hpp` | Free / Approach / Contact / Lost |
| `calib.hpp` | `FtCalib`（mass、com、f0、m0）、`FrameCalib`（R_ts、p_ts） |

**不放：** 虚函数、滤波、补偿、CAN、ROS。

---

### 4.2 `ports/` — 后端窄接口

**思想：** 只抽象「可换的 I/O」，不抽象「可换的算法」。起步三个就够。

| 计划文件 | 功能 |
|----------|------|
| `force_sensor.hpp` | `IForceSensor::Read(WrenchSample&)`：非阻塞；**传感器系原始力**；失败则 `valid=false` |
| `joint_driver.hpp` | `IJointDriver`：Enable / Disable / Read(`JointState`) / Write(`JointCommand`) |
| `clock.hpp` | `IClock::Now()`：便于单测注入假时钟 |

**故意不做端口：** `IKinematics`（纯函数即可）、`IAdmittance`、`ILogger`（先注入函数指针或环形日志）。

真机、仿真、回放都是这些接口的**实现**，不是 if 分支。

---

### 4.3 `kine/` — 运动学（Pinocchio 2.7 适配）

**思想：** 控制 / 仿真 / 工具只调公开函数（`Pose` / `JointVec`）；内部用 **Pinocchio 2.7**，头文件不泄漏 `pinocchio::`。模型在启动时建一次，RT 里只算 `fk(q)`。IK 做成**一拍增量**，与 RT 循环同拍。依赖安装见仓库 `scripts/setup_deps.sh`。

| 计划文件 | 功能 |
|----------|------|
| `model_params.hpp/.cpp` | DH/几何常数、杆长（或从 yaml 填入的 POD） |
| `fk.hpp/.cpp` | `Pose fk(q)`：只算末端位姿 |
| `fk_links.hpp/.cpp` | （可选）各连杆位姿，供仿真显示/碰撞 |
| `jacobian.hpp/.cpp` | 几何雅可比（默认坐标系在头文件写死，另一个用显式参数） |
| `ik_dls.hpp/.cpp` | 阻尼最小二乘一步：`ik_step(q, dx, params)` |
| `scale.hpp/.cpp` | 关节尺度（处理 J3 量纲与姿态权重） |
| `limits.hpp/.cpp` | 是否越限、到限距离 |
| `test/test_fk.cpp` | 已知姿态对照单测 |

---

### 4.4 `dyn/` — 自建动力学

**思想：** 控制用 `gravity` + `friction` 做力矩前馈；仿真植物用 `mass` + `rnea` 做正向积分；**两边必须链同一个 `dyn` 库**。

| 计划文件 | 功能 |
|----------|------|
| `inertial_params.hpp/.cpp` | 各连杆质量、质心、惯量张量 |
| `rnea.hpp/.cpp` | 递归牛顿-欧拉：`tau = rnea(q, dq, ddq)` |
| `gravity.hpp/.cpp` | `gravity(q)`（可即 `rnea(q,0,0)`） |
| `mass.hpp/.cpp` | 惯性矩阵 `M(q)` |
| `friction.hpp/.cpp` | 摩擦模型（如 tanh），与刚体动力学分开 |
| `test/test_rnea.cpp` | 能量/重力方向等检查 |

禁止在控制文件里手写 `tau = m*g*cos(q)`。

---

### 4.5 `signal/` — 力信号链

**思想：** 端口只出原始传感器系力；补偿与滤波全在这里。离开本模块后，力一律视为 TCP 系、已补偿外力。

| 计划文件 | 功能 |
|----------|------|
| `wrench_transform.hpp/.cpp` | 传感器系 → TCP 的力旋量伴随变换（含 `[p]×`） |
| `gravity_comp.hpp/.cpp` | 已知 m、com、f0、m0 时做重力/零偏减法 |
| `identify_ls.hpp/.cpp` | 两步最小二乘辨识标定参数 |
| `filter_median.hpp/.cpp` | 中值滤波 |
| `filter_butterworth.hpp/.cpp` | 二阶低通，状态显式传入传出 |
| `diff_lpf.hpp/.cpp` | 滤波微分 |

滤波器做成无状态纯函数（状态作参数），才能用录制数据离线重放调参。

---

### 4.6 `control/` — 控制律

**思想：** 只管「怎么动」，不管「数据从哪来」。力控按单轴 → 6D → 选择矩阵 → FSM → 限幅拆开。

#### `control/force/`

| 计划文件 | 功能 |
|----------|------|
| `admittance_axis.hpp/.cpp` | **单轴**半隐式欧拉导纳，不含 6D、不含 IK |
| `admittance_6d.hpp/.cpp` | 六轴循环调用单轴 |
| `selection.hpp/.cpp` | 选择矩阵 S；在接触系作用再伴随到基座 |
| `contact_fsm.hpp/.cpp` | FREE / APPROACH / CONTACT / LOST |
| `safety_limit.hpp/.cpp` | 步长、速度、力阈值 |

#### `control/motion/`、`control/traj/`（后加）

| 目录 | 功能 |
|------|------|
| `motion/` | 关节/笛卡尔纯位控 |
| `traj/` | 轨迹发生 |

与力控并列，由 FSM 或任务层切换，**不要写进导纳文件**。

---

### 4.7 `runtime/` — 实时编排

**思想：** RT 线程只做编排：读端口 → signal → kine → control → dyn 前馈 → 写端口。`rt_loop` 保持一屏可读（约百行量级）。

| 计划文件 | 功能 |
|----------|------|
| `spsc.hpp` | 单生产者单消费者无锁队列（任务线程 ↔ RT） |
| `rt_loop.hpp/.cpp` | 定周期主循环；只认端口指针 |
| `watchdog.hpp/.cpp` | 超时、丢帧计数 |

规则：

- **只有 RT 线程**碰端口的 Read/Write。
- 参数用双缓冲切换，禁止 RT 里解析 yaml。
- 力传感器若另有采集线程，只把 `WrenchSample` 丢进 SPSC；RT 取最新帧，允许丢旧，不许阻塞。

一拍示意：

```
joints.Read() → sensor.Read()
→ signal（变换/补偿/滤波）
→ kine.fk / jacobian
→ control.force（FSM → 混控 → 导纳 → 限幅）
→ kine.ik_step
→ dyn.gravity → 填 tau_ff
→ joints.Write()
```

---

### 4.8 `hal/` — 真机适配

**思想：** 协议与传动换皮；逻辑关节量（J3=米）在离开本层之前已经换算好。

| 计划文件 | 功能 |
|----------|------|
| `can_socket.hpp/.cpp` | 原始 CAN 收发，不懂 MIT |
| `motor_mit.hpp/.cpp` | MIT 打包/解包，实现 `IJointDriver` |
| `motor_j3_trans.hpp/.cpp` | 丝杠 m ↔ 电机 rad；**仅此处出现导程** |
| `sensor_*.hpp/.cpp` | 各厂商六维力协议，实现 `IForceSensor` |
| `clock_realtime.hpp/.cpp` | 实现 `IClock` |

禁止在 `hal` 里做 FK；禁止调用 `sim`。

---

### 4.9 `sim/` — 仿真后端

**思想：** 与 `hal` 平级；植物用同一套 `kine`/`dyn`，避免真机与仿真悄悄分叉。

| 计划路径 | 功能 |
|----------|------|
| `plant/` | 关节空间积分：`M ddq = tau_net` |
| `contact/` | 环境几何 + 接触力（先弹簧阻尼） |
| `joint_sim.hpp/.cpp` | 实现 `IJointDriver` |
| `sensor_sim.hpp/.cpp` | 接触力变到传感器系，实现 `IForceSensor` |
| `servo_sim.hpp/.cpp` | 用 kp/kd 把 `JointCommand` 变成力矩（模拟 MIT 内环） |
| `world.hpp/.cpp` | 环境刚体/平面/海绵参数 |

保真度可分阶段（S0 无接触积分 → S1 弹簧+假六维力 → …），不必一上来上 MuJoCo。

---

### 4.10 `app_real/` / `app_sim/` — 装配根

**思想：** 差异只在 wiring；`RtLoop` 类型相同。

| 计划文件 | 功能 |
|----------|------|
| `config.cpp` | 把 yaml 填进 POD 配置结构 |
| `wiring.cpp` | **唯一** `#include` 具体后端并 `new`、注入的地方 |
| `main.cpp` | 解析参数、装 RT、跑到退出 |

现状：`main_placeholder.cpp` 仅占位，保证空骨架能链接。

---

### 4.11 `tools/` — 标定与回放

| 子目录 | 功能 |
|--------|------|
| `smoke_wrench/` | 六维力 types/ports 早期烟雾（读 stub、打印） |
| `force_identify/` | 空载扫姿、最小二乘标定工具 |
| `sensor_replay/` | 录包实现 `IForceSensor`，离线调滤波/导纳 |

回放也是端口实现，因此调参路径与真机一致。

---

### 4.12 `config/` — 配置文件归属

| 文件 | 属于 | 谁改 / 谁读 |
|------|------|-------------|
| `robot.yaml` | 几何、限位、J3 导程 | kine + hal 传动 |
| `inertial.yaml` | 连杆惯量 | dyn |
| `sensor.yaml` | 安装位姿 `R_ts,p_ts` | signal + hal |
| `calib.yaml` | 力传感器 m、com、f0、m0 | signal |
| `admit.yaml` | 导纳、FSM | control/force |
| `filter.yaml` | 滤波 | signal |
| `rt.yaml` | 周期、看门狗 | runtime |
| `can.yaml` | 仅真机 | hal / app_real |
| `world.yaml` | 仅仿真：弹簧、地面 | sim / app_sim |

---

### 4.13 `cmake/` — 构建选项

| 文件 | 功能 |
|------|------|
| `HorkinOptions.cmake` | `HORKIN_BUILD_*` 选项；定义 `HORKIN_INCLUDE_ROOT` |

根 `CMakeLists.txt` 按选项 `add_subdirectory`，使无 CAN 机可只编仿真、无仿真机可只编真机。

---

## 5. 推荐落地顺序

1. **契约：** `types/time.hpp`、`types/wrench.hpp`、`ports/force_sensor.hpp`（可先做六维力一条线）。  
2. 补齐 `joint.hpp` / `command.hpp` / `IJointDriver` / `IClock`。  
3. `tools/smoke_wrench` 用 stub 实现跑通编译与打印。  
4. 并行：`kine` 单测、`dyn` 单测、`signal` 补偿、`sim` S0 积分、`hal` 单电机 CAN。  
5. `runtime/rt_loop` 接上；`app_sim` 先于或并行于 `app_real` 闭环。  
6. 力控：`admittance_axis` → 6D → selection → FSM → 限幅。  

旧工程（若有）**不混链进 Horkin Core**；需要对照时用同一组 `q` 在测试里比末端位姿和重力矩。

---

## 6. 迁移时动哪里

| 要换的 | 动哪里 | 不动哪里 |
|--------|--------|----------|
| USB2CAN 品牌 | `hal/can_socket` | kine、dyn、control、sim |
| 电机协议 | `hal/motor_*.cpp` | 导纳、FK |
| 力传感器 | `hal/sensor_*.cpp` | 补偿公式可复用 |
| 机械臂几何 | `kine/model_params` + `robot.yaml` | 导纳单轴、CAN |
| 惯量参数 | `dyn/inertial_params` | 混控选择矩阵 |
| 仿真接触 | `sim/contact` | control、hal |
| 换物理引擎 | `sim/plant` + `sim/contact` | ports、control |
| 仿 ↔ 真 | 换 `app_*` | `control/` `kine/` `dyn/` |

---

## 7. 构建

```bash
cmake -S . -B build
cmake --build build

# 可选裁剪
cmake -S . -B build -DHORKIN_BUILD_HAL=OFF    # 只编仿真
cmake -S . -B build -DHORKIN_BUILD_SIM=OFF    # 只编真机
```

当前可执行占位目标：`app_real`、`app_sim`、`smoke_wrench`。

---

## 8. 一句话总结设计思想

> **契约在中间，算法在上、设备在下；仿真与真机是同一套插头的两套实现；装配只在两个 main 里发生。**  
> 文件短、依赖单向、单位写死——这比「先把力控公式堆进一个大 cpp」更能长期演进。
