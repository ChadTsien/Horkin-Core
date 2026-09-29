# Horkin Core — 可换后端的运动控制内核（骨架）

独立工程：公共类型 / 端口、自建运动学动力学、力信号、控制、真机 HAL、仿真、双装配根。

## 设计说明（必读）

完整设计思想、依赖规则、以及**每个目录与计划中文件要实现的功能**，见：

- **[docs/DESIGN.md](docs/DESIGN.md)**

## 依赖关系（简图）

```
app_real ──► hal  ──► ports
app_sim  ──► sim  ──► ports
                ▲
runtime ──► control / signal / kine / dyn ──► types
```

## 环境依赖

运动学 / 动力学使用 **Pinocchio 2.7.0**（C++，不要 3.x / 4.x）。Ubuntu 22.04 **x86_64** 用 robotpkg 安装到 `/opt/openrobots`，并钉死 `hpp-fcl 2.4.4`。

```bash
./scripts/setup_deps.sh
source scripts/env.sh
```

`setup_deps.sh` 会加 robotpkg 源、按版本安装、写入 apt pin、`apt-mark hold`。之后每次开终端 `source scripts/env.sh`（或把其中的 `export` 写进 `~/.bashrc`）。

本机已装好时也可只：

```bash
source scripts/env.sh
pkg-config --modversion pinocchio   # 期望 2.7.0
```

aarch64 / Jetson 不能用这份 amd64 包，需自行安装同版本的 Pinocchio 2.7。

## 构建

```bash
source scripts/env.sh
cmake -S . -B build
cmake --build build
```

选项（见 `cmake/HorkinOptions.cmake`）：

| 选项 | 默认 | 含义 |
|------|------|------|
| `HORKIN_BUILD_SIM` | ON | 编仿真库 + `app_sim` |
| `HORKIN_BUILD_HAL` | ON | 编真机库 + `app_real` |
| `HORKIN_BUILD_TOOLS` | ON | 编工具（如 `smoke_wrench`） |
| `HORKIN_BUILD_TESTS` | OFF | 编 kine/dyn 单测 |

## Include 约定

```cpp
#include "types/wrench.hpp"
#include "ports/force_sensor.hpp"
```

（`HORKIN_INCLUDE_ROOT` = 本仓库根目录。）

## 现状

- 目录与 CMake 目标已齐；逻辑库多为 **INTERFACE 占位**。
- `app_*` / `smoke_wrench` 使用 `main_placeholder.cpp`，保证空骨架可配置、可链接。
- **下一步建议：** 落地 `types/time.hpp`、`types/wrench.hpp`、`ports/force_sensor.hpp`。

## 目录速查

| 目录 | 一句话 |
|------|--------|
| `types/` | 零依赖 POD |
| `ports/` | `IForceSensor` / `IJointDriver` / `IClock` |
| `kine/` `dyn/` | 运动学 / 动力学（Pinocchio 2.7 适配） |
| `signal/` | 补偿、滤波、力坐标变换 |
| `control/` | 力控（及后续位控/轨迹） |
| `runtime/` | RT 循环、SPSC、看门狗 |
| `hal/` | 真机 CAN / 电机 / 力传感器 |
| `sim/` | 植物、接触、假驱动与假力觉 |
| `app_real/` `app_sim/` | 唯一接线处 |
| `tools/` | 烟雾、标定、回放 |
| `config/` | 按模块拆分的 yaml |
| `docs/` | 设计文档 |
