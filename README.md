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
rl-checkpoints/model_1000.jit
```

该文件来自 HIMLoco 训练运行：

```text
rough_minich/Aug06_16-16-21_smoke_minich/model_1000.pt
```

导出文件和部署副本的 SHA256 均为：

```text
f54bcdc97d274a26adaa341bd2b9073807d64261f9f85a139c71852228e39b0c
```

该 TorchScript 包含 HIM 速度估计器和 Actor，不是旧的单 Actor flat policy。输入和
输出均为 CPU `float32`：

```text
input_shape=[1, 270]
action_shape=[1, 12]
```

目录中原有的 `adaptation_module_latest.jit`、`body_latest.jit` 和
`legged_gym_policy_latest.jit` 可以保留，但 `RL_JOINT_PD` 不再加载它们。

## 270 维 HIM 历史观测

策略输入由 6 帧 45 维观测组成：

```text
[当前帧, t-1, t-2, t-3, t-4, t-5]
6 × 45 = 270
```

每个 45 维帧严格按照 HIMLoco 训练端的顺序拼接：

| 帧内索引 | 维数 | 含义 | 写入模型前的数值 |
|---|---:|---|---|
| `0:3` | 3 | 期望前向速度、侧向速度和偏航角速度 `[vx,vy,wz]` | 分别乘以 `[2.0,2.0,0.25]` |
| `3:6` | 3 | 机身坐标系角速度 `omegaBody` | `omegaBody * 0.25` |
| `6:9` | 3 | 世界重力方向投影到机身坐标系后的单位向量 | 不缩放，水平静止时约为 `[0,0,-1]` |
| `9:21` | 12 | 当前关节角相对默认站立角的偏差 | `q - q_default` |
| `21:33` | 12 | 当前关节角速度 | `qd * 0.05` |
| `33:45` | 12 | 上一次 Actor 输出 | `last_action` |

新策略的 actor 不直接接收机身线速度。TorchScript 内的 HIM estimator 根据 6 帧
历史观测估计 3 维速度和 16 维 latent，再与当前帧的 45 维观测拼接后送入 actor。
进入 `RL_JOINT_PD` 时历史缓冲清零；之后每次 50 Hz 推理都把最新帧放在最前并
将旧帧向后移动。每帧拼接完成后逐元素裁剪到 `[-100,100]`。速度命令范围与
训练配置一致：`vx/vy∈[-1,1] m/s`、`yaw∈[-3.14,3.14] rad/s`。

### 坐标系与关节顺序

- `vBody`、`omegaBody` 和投影重力均为机身坐标系数据。
- 策略中的12个关节按 `FR, FL, RR, RL` 排列。
- 每条腿内部按 `hip/abad, thigh, calf` 排列。
- Cheetah 控制器内部同样按 `FR, FL, RR, RL` 排列，`RL_JOINT_PD` 不再进行额外
  腿顺序交换。

策略顺序下的默认关节角为：

| 腿 | hip/abad | thigh | calf |
|---|---:|---:|---:|
| FL | `0.10` | `-0.80` | `1.62` |
| FR | `-0.10` | `-0.80` | `1.62` |
| RL | `0.10` | `-0.80` | `1.62` |
| RR | `-0.10` | `-0.80` | `1.62` |

角度单位为弧度。

### 不使用地形高度观测

HIM actor 的公开输入是 270 维本体历史观测，不输入部署侧高度图。训练时的高度
观测只用于 privileged critic，不属于导出的 estimator + actor 输入契约。

## 12 维输出与关节控制

Actor 输出12维动作，顺序同样是 `FR, FL, RR, RL`。动作先按训练配置裁剪到
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
LibTorch 1.10.1 CPU 包，即 `_GLIBCXX_USE_CXX11_ABI=0`。推荐把包放在源码根
目录下的相对位置：

```text
<源码根>/.deps/libtorch-1.10.1-cpu
```

CMake 会依次接受有效的显式路径、本地配置、环境变量 `LIBTORCH_ROOT`、源码内
`.deps` 和源码父目录的 `.deps`；因此仓库或上层工作区改名不会影响编译。旧
`CMakeCache.txt` 中已经失效的绝对路径也会自动跳过并重新发现。

以下命令假设当前位于源码根目录；首次创建实机构建目录时执行：

```bash
mkdir -p build-cpu
cd build-cpu

cmake .. -DNO_SIM=ON

cmake --build . \
  --target mit_ctrl rapid_rl_policy_benchmark rapid_rl_policy_config_test -j
```

之后日常增量编译只需要第二条 `cmake --build . ...`，无需再次书写 LibTorch、
MKL、CPU-only 或顶层目录路径。

部分 CPU LibTorch 包在最终链接时需要外部 MKL。CMake 会自动尝试有效缓存、
`LIBTORCH_MKL_ROOT` 环境变量、激活 Conda 环境的 `$CONDA_PREFIX/lib` 和源码
相对的 `.deps/mkl/lib`。若仍报告 `mkl_*` 符号，复制一次本地配置模板并只填写
该机器的实际目录：

```bash
cp ../cmake/RapidRLLocalConfig.cmake.example \
  ../cmake/RapidRLLocalConfig.cmake
# 编辑 ../cmake/RapidRLLocalConfig.cmake 中的
# RAPID_RL_LOCAL_MKL_ROOT；非标准 LibTorch 位置则填 RAPID_RL_LOCAL_LIBTORCH_ROOT。
cmake ..
```

不要直接把另一台电脑上使用 `-march=native` 编译的 `mit_ctrl` 复制到机载电脑；
应在最终运行策略的机载电脑上重新配置和构建。

## 仿真 GUI 编译（RL 三关节 Kp/Kd 实时调参）

`build-cpu` 不必重建为另一个目录。若想在它中生成 GUI，只需在该构建目录中
重新配置一次 `NO_SIM`；LibTorch/MKL 等路径仍由上面的 CMake 配置文件处理：

```bash
cd build-cpu

cmake .. -DNO_SIM=OFF

cmake --build . \
  --target sim mit_ctrl rapid_rl_policy_benchmark rapid_rl_policy_config_test -j

ctest --output-on-failure
```

`NO_SIM=OFF` 只增加 Qt GUI 的 `sim` 目标，不改变 `mit_ctrl` 的 CPU-only
LibTorch 或实机 profile。若以后要恢复无 GUI 的精简构建，在相同目录执行
`cmake .. -DNO_SIM=ON` 即可。

当前 GUI 按工作目录中的 `../config` 和 `../resources` 查找文件，因此
`build` 或 `build-cpu` 必须直接位于源码根目录下，并从该目录启动：

```bash
# 终端 A
cd build-cpu
./sim/sim

# 终端 B：也从同一个 build-cpu 目录启动
cd build-cpu
./user/MIT_Controller/mit_ctrl m s
```

在 GUI 中选择 Mini Cheetah / Simulator 并启动后，切到 `RL_JOINT_PD`
（控制模式 53），即可在 User Parameters 表实时编辑
`rl_kp_joint=[abad, hip/thigh, knee/calf]` 和 `rl_kd_joint`。这两个参数只在
`mit_ctrl m s` 的仿真 profile 中生效，`mit_ctrl m r` 的实机增益保持固定。

## 部署验证

以下命令也在当前 `build` 或 `build-cpu` 目录执行。先运行观测、关节映射、
默认角、动作缩放和 PD 增益测试：

```bash
ctest --output-on-failure
```

检查控制器二进制没有 CUDA 依赖或未解析动态库：

```bash
ldd user/MIT_Controller/mit_ctrl \
  | grep -E "cuda|not found"
```

上面的 `ldd` 检查必须没有任何输出。然后在机载电脑上运行1000次推理：

```bash
./user/MIT_Controller/rapid_rl_policy_benchmark 1000
```

输出应明确包含当前 HIMLoco 策略路径和形状：

```text
[LeggedGymRLBenchmark] policy=.../rl-checkpoints/model_1000.jit
[LeggedGymRLBenchmark] input_shape=[1, 270] action_shape=[1, 12]
```

验收要求为所有输出有限且 `p95 < 18 ms`。本机 100 次验证得到约
`p95=0.043 ms`。控制器初始化时也应出现：

```text
[LeggedGymRL] Loaded LibTorch policy: .../model_1000.jit
[LeggedGymRL] Shapes: input [1, 270], action [1, 12]
```

完整的 checkpoint 替换、CPU-only 构建和验证流程见
[`docs/rapid_rl_libtorch_deploy.md`](docs/rapid_rl_libtorch_deploy.md)。
