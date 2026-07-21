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

当前目录中的文件来自训练运行
`rough_minich/Jul17_23-38-03_hip_sym_zero_default_2048x4000/model_4000.pt`：

```text
训练 checkpoint SHA256:
404a259695c0c6f69f3fcf78d39bd32d1a8d056417fdc958a47be04b708c3148

部署 TorchScript SHA256:
65b98c3b00ec595638f10a24d45d392b022c7b7b1e147193cc2b37d5c982f94e
```

这是旧的 235 维 rough 策略，仅保留作迁移前记录；它不能与本源码重新构建后的
48 维平地接口一起运行。正式切换时必须同时替换控制器二进制和同名 TorchScript。

目标平地模型是单个 `48 -> 512 -> 256 -> 128 -> 12` ELU Actor，输入和输出均为
CPU `float32`：

```text
input_shape=[1, 48]
action_shape=[1, 12]
```

目录中原有的 `adaptation_module_latest.jit` 和 `body_latest.jit` 可以保留，
但 `RL_JOINT_PD` 不再加载它们，也不再维护历史观测或 latent 向量。替换策略时
必须保持新文件名为 `legged_gym_policy_latest.jit`。

## 48 维输入观测

输入张量形状为 `[1,48]`。48维由下列分段顺序拼接：

```text
3 + 3 + 3 + 3 + 12 + 12 + 12 = 48
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

### 不使用地形高度观测

平地策略训练与部署均不输入高度图或机身高度派生的占位值。它只适用于平整地面；
若要部署到粗糙地形，必须重新引入与训练一致的真实地形感知输入并重新训练。

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
| Cheetah 模拟器（`mit_ctrl m s`） | `20 / 0.5` | `20 / 0.5` | `20 / 0.5` | GUI 可实时覆盖的初始值 |
| Mini Cheetah 实机（`mit_ctrl m r`） | `20 / 0.5` | `20 / 0.5` | `20 / 0.5` | 修改前已验证的实机配置 |

模拟器和实机当前默认数值相同，但在代码中是两套独立常量；以后调整模拟器
增益不会连带修改实机值。仿真运行时，GUI 的 User Parameters 表可通过
`rl_kp_joint` 和 `rl_kd_joint` 覆盖三类关节的值，顺序为
`[abad, hip/thigh, knee/calf]`，四条腿共用；实机不会读取这两个覆盖项。
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

## 仿真 GUI 编译（RL 三关节 Kp/Kd 实时调参）

上面的实机 `build-cpu` 命令**没有变化**，并且应继续使用 `-DNO_SIM=ON`。
若要在仿真 GUI 中实时修改 `rl_kp_joint`、`rl_kd_joint`，需要单独的
`build-sim-cpu` 构建目录，并显式启用 `-DNO_SIM=OFF`：

```bash
cd Cheetah-Software-RL

cmake -S . -B build-sim-cpu \
  -DNO_SIM=OFF \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DLIBTORCH_ROOT=/opt/libtorch-1.10.1-cpu \
  -DLIBTORCH_CPU_ONLY=ON

cmake --build build-sim-cpu \
  --target sim mit_ctrl rapid_rl_policy_benchmark rapid_rl_policy_config_test -j

ctest --test-dir build-sim-cpu --output-on-failure
```

如果该 CPU LibTorch 包需要 MKL，在上述 `cmake` 配置命令中同样追加
`-DLIBTORCH_MKL_ROOT=/path/to/mkl/lib`。不要复用 `build-cpu`：其 CMake
缓存中 `NO_SIM=ON`，没有 GUI 的 `sim` 目标。

当前 GUI 按工作目录中的 `../config` 和 `../resources` 查找文件，因此
`build-sim-cpu` 必须直接位于源码根目录下，并从该目录启动：

```bash
# 终端 A
cd Cheetah-Software-RL/build-sim-cpu
./sim/sim

# 终端 B：也从同一个 build-sim-cpu 目录启动
cd Cheetah-Software-RL/build-sim-cpu
./user/MIT_Controller/mit_ctrl m s
```

在 GUI 中选择 Mini Cheetah / Simulator 并启动后，切到 `RL_JOINT_PD`
（控制模式 53），即可在 User Parameters 表实时编辑
`rl_kp_joint=[abad, hip/thigh, knee/calf]` 和 `rl_kd_joint`。这两个参数只在
`mit_ctrl m s` 的仿真 profile 中生效，`mit_ctrl m r` 的实机增益保持固定。

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
[LeggedGymRLBenchmark] input_shape=[1, 48] action_shape=[1, 12]
```

验收要求为所有输出有限且 `p95 < 18 ms`。控制器初始化时也应出现：

```text
[LeggedGymRL] Loaded LibTorch policy: .../legged_gym_policy_latest.jit
[LeggedGymRL] Shapes: input [1, 48], action [1, 12]
```

完整的 checkpoint 替换、CPU-only 构建和验证流程见
[`docs/rapid_rl_libtorch_deploy.md`](docs/rapid_rl_libtorch_deploy.md)。
