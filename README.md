# Mini Cheetah `RL_JOINT_PD` 策略部署

本仓库基于 MIT Biomimetics Lab 的 Cheetah-Software，用于 Mini Cheetah
控制器开发。原始工程的构建、仿真和硬件说明可参考
[MIT Cheetah-Software](https://github.com/mit-biomimetics/Cheetah-Software)。

当前 `RL_JOINT_PD` 使用 C++ 控制进程内的 CPU LibTorch 推理，直接加载一个
由 `legged_gym` 导出的确定性 Actor。其他 FSM 状态、模式切换和硬件控制链路
保持原样。`RL_JOINT_PD` 对应的控制模式常量为 `K_RL_JOINT_PD=53`。

## 当前策略文件

运行时只加载：

```text
rl-checkpoints/legged_gym_policy_latest.jit
```

当前文件来自训练运行 `rough_minich/Jul14_17-07-42_/model_4001.pt`：

```text
训练 checkpoint SHA256:
9d359f46b5a2b95a07ef5f90993c12b37c82a1e87a8b16f2d1168332d14dbfc7

部署 TorchScript SHA256:
ac9da57984a0d3bdb35725b38c5d46dc959f85f70c5cce2a9e8c35358f778a65
```

模型是单个 `235 -> 512 -> 256 -> 128 -> 12` ELU Actor，输入和输出均为
CPU `float32`：

```text
input_shape=[1, 235]
action_shape=[1, 12]
```

目录中原有的 `adaptation_module_latest.jit` 和 `body_latest.jit` 可以保留，
但 `RL_JOINT_PD` 不再加载它们，也不再维护历史观测或 latent 向量。替换策略时
必须保持新文件名为 `legged_gym_policy_latest.jit`。

## 235 维输入观测

输入张量形状为 `[1,235]`。235维由下列分段顺序拼接：

```text
3 + 3 + 3 + 3 + 12 + 12 + 12 + 187 = 235
```

| 索引 | 维数 | 含义 | 写入模型前的数值 |
|---|---:|---|---|
| `0:3` | 3 | 机身坐标系线速度 `vBody`，分别为 x/y/z | `vBody * 2.0` |
| `3:6` | 3 | 机身坐标系角速度 `omegaBody`，分别为 roll/pitch/yaw 轴 | `omegaBody * 0.25` |
| `6:9` | 3 | 世界重力方向投影到机身坐标系后的单位向量 | 不缩放，水平静止时约为 `[0,0,-1]` |
| `9:12` | 3 | 期望前向速度、侧向速度和偏航角速度 `[vx,vy,wz]` | 分别乘以 `[2.0,2.0,0.25]` |
| `12:24` | 12 | 当前关节角相对默认站立角的偏差 | `q - q_default` |
| `24:36` | 12 | 当前关节角速度 | `qd * 0.05` |
| `36:48` | 12 | 上一次 Actor 输出，用于描述策略自身上一控制步的动作 | `last_action` |
| `48:235` | 187 | 平地高度观测 | 将 `clip(base_z - 0.5,-1,1) * 5` 重复187次 |

拼接完成后，整个观测向量逐元素裁剪到 `[-100,100]`。Runner 会在裁剪前拒绝
包含 NaN 或 Inf 的状态输入，并在每次推理后检查输出形状、类型和有限性。

### 坐标系与关节顺序

- `vBody`、`omegaBody` 和投影重力均为机身坐标系数据。
- 策略中的12个关节按 `FL, FR, RL, RR` 排列。
- 每条腿内部按 `hip/abad, thigh, calf` 排列。
- Cheetah 控制器内部按 `FR, FL, RR, RL` 排列，`RL_JOINT_PD` 会在策略输入、
  输出边界进行双向映射。

策略顺序下的默认关节角为：

| 腿 | hip/abad | thigh | calf |
|---|---:|---:|---:|
| FL | `0.10` | `-0.80` | `1.62` |
| FR | `-0.10` | `-0.80` | `1.62` |
| RL | `0.10` | `-0.80` | `1.62` |
| RR | `-0.10` | `-0.80` | `1.62` |

角度单位为弧度。

### 平地高度观测的限制

训练环境中的187维高度来自17×11个独立采样点。当前实机首版仅面向平地，
没有真实地形高度图，因此把同一个平地高度值重复187次。例如机身估计高度
`base_z=0.30 m` 时，每一个高度输入都是：

```text
(0.30 - 0.50) * 5 = -1.0
```

这一近似只适用于平地，不应视为粗糙地形感知。

## 12 维输出与关节控制

Actor 输出12维动作，顺序同样是 `FL, FR, RL, RR`。动作先按训练配置裁剪到
`[-100,100]`，随后生成关节目标角：

```text
q_target = q_default + 0.25 * action
```

PD 增益按使用环境严格分开，训练参数不会直接下发到实机：

| 使用环境 | hip/abad Kp/Kd | thigh Kp/Kd | calf Kp/Kd | 用途 |
|---|---:|---:|---:|---|
| Isaac Gym 训练 | `17 / 0.4` | `17 / 0.4` | `34 / 0.8` | 仅记录 Actor 的训练条件 |
| Cheetah 模拟器（`mit_ctrl m s`） | `20 / 0.5` | `20 / 0.5` | `20 / 0.5` | 独立模拟器配置 |
| Mini Cheetah 实机（`mit_ctrl m r`） | `20 / 0.5` | `20 / 0.5` | `20 / 0.5` | 修改前已验证的实机配置 |

模拟器和实机当前数值相同，但在代码中是两套独立常量；以后调整模拟器
增益不会连带修改实机值。运行时只会在“模拟器”和“实机”两套配置中选择，
训练增益只作为元数据保留。控制器启动日志会打印实际选择的 profile 和三组
`Kp/Kd`，实机启动时应明确显示 `real-mini-cheetah` 与
`Kp=[20, 20, 20]`、`Kd=[0.5, 0.5, 0.5]`。

策略以50 Hz运行；底层控制周期保持不变。控制命令继续使用 `qdDes=0` 和
`tauFeedForward=0`，最终目标仍经过 `RL_JOINT_PD` 现有的关节范围、姿态和
目标变化速率检查。

## 实机 CPU-only LibTorch 编译

机载电脑应使用 CPU-only LibTorch。当前部署已验证兼容非 cxx11-abi 的
LibTorch 1.10.1 CPU 包，即 `_GLIBCXX_USE_CXX11_ABI=0`。以下示例假设包位于：

```text
/opt/libtorch-1.10.1-cpu
```

在部署工作区根目录执行：

```bash
cmake -S Cheetah-Software-RL -B Cheetah-Software-RL/build-cpu \
  -DNO_SIM=ON \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DLIBTORCH_ROOT=/opt/libtorch-1.10.1-cpu \
  -DLIBTORCH_CPU_ONLY=ON

cmake --build Cheetah-Software-RL/build-cpu \
  --target mit_ctrl rapid_rl_policy_benchmark rapid_rl_policy_config_test -j
```

如果链接时报缺少 `mkl_*` 符号，重新配置时增加 MKL 动态库目录：

```bash
cmake -S Cheetah-Software-RL -B Cheetah-Software-RL/build-cpu \
  -DNO_SIM=ON \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DLIBTORCH_ROOT=/opt/libtorch-1.10.1-cpu \
  -DLIBTORCH_CPU_ONLY=ON \
  -DLIBTORCH_MKL_ROOT=/path/to/mkl/lib
```

不要直接把另一台电脑上使用 `-march=native` 编译的 `mit_ctrl` 复制到机载电脑；
应在最终运行策略的机载电脑上重新配置和构建。

## 部署验证

先运行观测、关节映射、默认角、动作缩放和 PD 增益测试：

```bash
ctest --test-dir Cheetah-Software-RL/build-cpu --output-on-failure
```

检查控制器二进制没有 CUDA 依赖或未解析动态库：

```bash
ldd Cheetah-Software-RL/build-cpu/user/MIT_Controller/mit_ctrl \
  | grep -E "cuda|not found"
```

上面的 `ldd` 检查必须没有任何输出。然后在机载电脑上运行1000次推理：

```bash
./Cheetah-Software-RL/build-cpu/user/MIT_Controller/rapid_rl_policy_benchmark 1000
```

输出应明确包含当前单 Actor 路径和形状：

```text
[LeggedGymRLBenchmark] policy=.../rl-checkpoints/legged_gym_policy_latest.jit
[LeggedGymRLBenchmark] input_shape=[1, 235] action_shape=[1, 12]
```

验收要求为所有输出有限且 `p95 < 18 ms`。控制器初始化时也应出现：

```text
[LeggedGymRL] Loaded LibTorch policy: .../legged_gym_policy_latest.jit
[LeggedGymRL] Shapes: input [1, 235], action [1, 12]
```

完整的 checkpoint 替换、CPU-only 构建和验证流程见
[`docs/rapid_rl_libtorch_deploy.md`](docs/rapid_rl_libtorch_deploy.md)。
