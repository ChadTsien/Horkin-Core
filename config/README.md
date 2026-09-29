# 运行配置说明

配置按「谁拥有参数谁改文件」拆分；控制/算法函数只收 POD，不在 RT 里读 yaml。

| 文件 | 内容 | 主要读者 |
|------|------|----------|
| `robot.yaml` | 几何、限位、J3 导程 | kine、hal 传动 |
| `inertial.yaml` | 连杆惯量 | dyn |
| `sensor.yaml` | 安装位姿 `R_ts`, `p_ts` | signal、hal |
| `calib.yaml` | 力传感器 mass / com / f0 / m0 | signal |
| `admit.yaml` | 导纳参数、接触 FSM | control/force |
| `filter.yaml` | 滤波参数 | signal |
| `rt.yaml` | 控制周期、看门狗 | runtime |
| `can.yaml` | CAN 接口与电机 ID | 仅 app_real / hal |
| `world.yaml` | 弹簧环境、地面等 | 仅 app_sim / sim |

文件尚未创建时，由 `app_*/config.cpp` 落地时再补；归属规则以 [DESIGN.md](../docs/DESIGN.md) §4.12 为准。
