# Mini Cheetah `RL_JOINT_PD` 策略部署

本仓库基于 MIT Biomimetics Lab 的 Cheetah-Software，用于 Mini Cheetah
控制器开发。原始工程的构建、仿真和硬件说明可参考
[MIT Cheetah-Software](https://github.com/mit-biomimetics/Cheetah-Software)。

当前上层 FSM 包含两个独立状态：旧平地 MLP `RL_FLAT=54` 和 HIM ONNX
楼梯 `RL_STAIRS=53`（兼容原 `RL_JOINT_PD=53`）。LT+A 起立后，Start 请求
平地状态、Back 请求楼梯状态；切换时先停稳再过渡，
保留导航控制权。LT+X 恢复站立，LT+B 失能。两套历史持续更新，各自保持观测和
动作定义。操作、切换条件和双策略离线测试见 [导航与双控制器说明](docs/himloco_nav_stairs.md)。

## 当前策略文件

楼梯策略加载：

```text
rl-checkpoints/onnx/policy.onnx
```

平地策略另外加载 `actor_model/{actor_5000.txt,estimator_5000.txt,mean5000.csv,var5000.csv}`。
可用 CMake `-DRL_FLAT_MODEL_DIR=/absolute/path/to/actor_model` 指定部署目录。
两套模型都在进程初始化时加载；部署时需要同时携带这些文件。

当前文件的导出说明标注来源为
`rough_minich/Oct12_09-14-06_smoke_minich/model_1000.pt`。
网络包含 HIM encoder、latent 归一化和 Actor，固定 batch=1，输入
`obs: float32 [1,270]`，输出 `actions: float32 [1,12]`。
完整导出说明见 [README_ONNX.md](rl-checkpoints/onnx/README_ONNX.md)。

加载时自动读取输入输出名称，检查 float32 和固定形状，执行预热并拒绝非有限动作。
运行时仅加载这个 ONNX 文件。

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

新策略的 actor 不直接接收机身线速度。ONNX 网络内的 HIM estimator 根据 6 帧
历史观测估计 3 维速度和 16 维 latent，再与当前帧的 45 维观测拼接后送入 actor。
进入 `RL_JOINT_PD` 时历史缓冲清零；之后每次 50 Hz 推理都把最新帧放在最前并
将旧帧向后移动。每帧拼接完成后逐元素裁剪到 `[-100,100]`。速度命令范围与
训练配置一致：`vx/vy∈[-1,1] m/s`、`yaw∈[-3.14,3.14] rad/s`。

### 坐标系与关节顺序

- `vBody`、`omegaBody` 和投影重力均为机身坐标系数据。
- 策略中的12个关节按 `左前腿、右前腿、左后腿、右后腿` 排列。
- 每条腿内部按 `外展关节、大腿关节、小腿关节` 排列。
- Cheetah 控制器内部按 `右前腿、左前腿、右后腿、左后腿` 排列，
  `RL_JOINT_PD` 会在策略输入和输出边界交换左右腿顺序。

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

Actor 输出12维动作，顺序同样是 `左前腿、右前腿、左后腿、右后腿`。动作先按
训练配置裁剪到 `[-100,100]`，随后转换成控制器的
`右前腿、左前腿、右后腿、左后腿` 顺序，并生成关节目标角：

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

## ONNX 依赖安装与 CPU 编译

### 新增依赖及本次使用的版本

| 依赖 | 用途 | 本次验证版本 |
|---|---|---|
| ONNX Runtime C/C++ CPU 包 | `mit_ctrl` 内加载和运行 `policy.onnx`，编译需要头文件，运行需要共享库 | `1.28.1`，Linux x86_64 |
| 支持 C++17 的 C++ 编译器 | 编译 ONNX Runtime C++ 接口；仅控制器和推理基准目标使用 C++17 | GCC `9.4.0` |
| Python `onnx`（可选） | 训练端导出、检查 ONNX 模型 | `1.17.0` |
| Python `onnxruntime`（可选） | 训练端及离线 Python 数值对照 | `1.19.2`，CPU |
| Python `flatbuffers`（可选） | Python ONNX Runtime 的依赖 | `24.3.25` |

直接运行 C++ 控制器只需要 C/C++ 包，不需要 Python 环境。
`pip install onnxruntime` 安装的是 Python 包，不能替代本工程的 C++ 头文件和链接库。
此分支使用默认 CPU provider，不需要 CUDA、cuDNN、LibTorch 或 MKL。
C++ 包内的头文件和共享库必须来自同一版本。安装来源见
[官方安装说明](https://onnxruntime.ai/docs/install/) 和
[ONNX Runtime 1.28.1 发布页](https://github.com/microsoft/onnxruntime/releases/tag/v1.28.1)。

### 安装 C/C++ 库（推荐放在项目内）

下面命令从仓库根目录执行，适用于 `uname -m` 输出为 `x86_64` 的 Linux。
需要 `curl`、`tar`、CMake 和 C++ 编译器；Ubuntu 缺少工具时可执行：

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake curl ca-certificates
```

下载已验证版本的 CPU 包，校验官方发布文件的 SHA256，再解压到
`.deps/onnxruntime`。首次安装时执行以下命令；已有其他版本时请解压到单独目录，
通过 `ONNXRUNTIME_ROOT` 指定，避免混用头文件与共享库。

```bash
mkdir -p .deps
curl -fL --retry 3 \
  https://github.com/microsoft/onnxruntime/releases/download/v1.28.1/onnxruntime-linux-x64-1.28.1.tgz \
  -o .deps/onnxruntime-linux-x64-1.28.1.tgz
printf '%s  %s\n' \
  '2529aef968d0ad0603365054bc46ebefa7f0fe3bc12f28c5f729c99ddffe2a81' \
  '.deps/onnxruntime-linux-x64-1.28.1.tgz' | sha256sum -c -
mkdir -p .deps/onnxruntime
tar -xzf .deps/onnxruntime-linux-x64-1.28.1.tgz \
  -C .deps/onnxruntime --strip-components=1
```

SHA256 来自[官方发布附件](https://github.com/microsoft/onnxruntime/releases/expanded_assets/v1.28.1)。
如果目标机器是 ARM64，应选择同一发布页的 `onnxruntime-linux-aarch64-1.28.1.tgz`
并使用对应 SHA256；本次控制器测试在 x86_64 上完成。
`.deps/` 已被 Git 忽略，不需要将第三方二进制提交到仓库。

安装后的关键目录如下（解压时保留包内的其他头文件和库）：

```text
.deps/onnxruntime/
├── include/
│   ├── onnxruntime_c_api.h
│   ├── onnxruntime_cxx_api.h
│   └── ...
└── lib/
    ├── libonnxruntime.so
    ├── libonnxruntime.so.1
    ├── libonnxruntime.so.1.28.1
    └── ...
```

`libonnxruntime.so` 用于链接，`libonnxruntime.so.1` 是本次运行时加载的 SONAME。
请完整保留 `lib/` 及其中符号链接，不要只复制一个 `.so` 文件。
可用以下命令检查安装：

```bash
test -f .deps/onnxruntime/include/onnxruntime_cxx_api.h
test -f .deps/onnxruntime/lib/libonnxruntime.so
ldd .deps/onnxruntime/lib/libonnxruntime.so
```

`ldd` 不应出现 `not found`。若目标系统报告 `GLIBC_*` 或 `GLIBCXX_*` 版本不足，
需要为目标系统选择兼容的包或[在目标环境编译 ONNX Runtime](https://onnxruntime.ai/docs/build/inferencing.html)。

### 配置、编译与验证

以下命令均从仓库根目录执行。项目内安装完成后，CMake 会自动发现
`.deps/onnxruntime`。`NO_SIM` 表示“不编译模拟器”，因此开启 sim 用 `OFF`，
关闭 sim 用 `ON`。

**开启 sim（编译控制器和模拟器）：**

```bash
cmake -S . -B build-onnx -DNO_SIM=OFF
cmake --build build-onnx --target mit_ctrl sim -j4
```

生成 `build-onnx/user/MIT_Controller/mit_ctrl` 和 `build-onnx/sim/sim`。
模拟器额外需要 Qt5（Core、Widgets、Gui、Gamepad）和 OpenGL 开发库。

**关闭 sim（只编译控制器）：**

```bash
cmake -S . -B build-onnx -DNO_SIM=ON
cmake --build build-onnx --target mit_ctrl -j4
```

两种配置可以复用同一个 `build-onnx` 目录；切换时先重新执行对应的
`cmake -S ...` 配置命令，再编译。关闭 sim 后，之前生成的 `build-onnx/sim/sim`
可能仍保留在磁盘上，但不会继续参与构建。

需要运行离线验证时，再编译测试和基准目标：

```bash
cmake --build build-onnx \
  --target mit_ctrl rapid_rl_policy_benchmark rapid_rl_policy_config_test -j4
ctest --test-dir build-onnx -R 'legged_gym_rl_policy_config|himloco_onnx_policy_inference' --output-on-failure
./build-onnx/user/MIT_Controller/rapid_rl_policy_benchmark 1000
ldd build-onnx/user/MIT_Controller/mit_ctrl
```

安装在其他目录时，在首次配置中明确指定库根目录：

```bash
cmake -S . -B build-onnx -DNO_SIM=ON \
  -DONNXRUNTIME_ROOT=/absolute/path/to/onnxruntime
```

CMake 依次尝试有效的 CMake 参数/缓存 `ONNXRUNTIME_ROOT`、本地配置
`RAPID_RL_LOCAL_ONNXRUNTIME_ROOT`、环境变量 `ONNXRUNTIME_ROOT`、源码及父目录下的
`.deps/onnxruntime` 和 `/opt/onnxruntime`。本地配置可这样创建：

```bash
cp cmake/RapidRLLocalConfig.cmake.example cmake/RapidRLLocalConfig.cmake
# 编辑该文件，设置 RAPID_RL_LOCAL_ONNXRUNTIME_ROOT 为库根目录。
```

本次没有重新下载 C++ 库，而是复用了本机已有的 `1.28.1` 包，实际配置命令为：

```bash
cmake -S . -B build-onnx -DNO_SIM=ON \
  -DONNXRUNTIME_ROOT=/home/xjtx/rl-deploy/hunter130_collection-main/hunter130_deploy/src/ec_controller/thirdparty/onnxruntime
```

该绝对路径仅记录本次环境；其他机器请使用上述项目内安装方式或自己的库路径。
本次 `mit_ctrl`、推理基准及配置测试均编译成功，两项 CTest 通过。
1000 次 CPU 推理的 p95 约 `0.026 ms`，最大约 `0.168 ms`；
24 步历史输入（含重置）与 Python ONNX Runtime 对照最大绝对误差约 `1.31e-6`。
这些结果是本机离线验证，不代表实机控制表现。

默认模型为 `rl-checkpoints/onnx/policy.onnx`；可配置
`RAPID_RL_CHECKPOINT_DIR` 与 `RAPID_RL_CHECKPOINT_FILE`。
推理默认单线程，可通过 `RAPID_RL_ONNX_NUM_THREADS` 调整。
请使用新构建目录 `build-onnx`，避免沿用旧 LibTorch 构建缓存。

### 动态库部署

CMake 为构建出的控制器配置 ONNX Runtime 库目录的 RPATH。
直接在本机运行时无需全局安装或执行 `ldconfig`。
部署到另一台机器时，将完整 C/C++ 包和模型一起部署，并在最终运行策略的
机载电脑上重新配置和构建。不要复用本机 `-march=native` 生成的二进制。

如果库的位置变化，应重新运行 CMake 指定新的 `ONNXRUNTIME_ROOT` 并构建。
临时诊断动态库查找问题时，也可在仓库根目录执行：

```bash
export LD_LIBRARY_PATH="$PWD/.deps/onnxruntime/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
ldd build-onnx/user/MIT_Controller/mit_ctrl
./build-onnx/user/MIT_Controller/rapid_rl_policy_benchmark 1000
```

确认 `libonnxruntime.so.1` 指向预期目录，且所有依赖均没有 `not found`。

### 可选：Python 导出与数值验证依赖

已有 ONNX 模型时可跳过本节。重新导出需要训练环境中的 PyTorch 和 HIMLoco。
本次在已有的 `gym4` 环境中，将新增 Python 依赖装到临时目录：

```bash
source /opt/miniconda3/etc/profile.d/conda.sh
conda activate gym4
python3 -m pip install --target /tmp/minich_onnx_deps --no-cache-dir --no-deps \
  onnx==1.17.0 onnxruntime==1.19.2 flatbuffers==24.3.25
PYTHONPATH=/tmp/minich_onnx_deps python3 -c \
  'import onnx, onnxruntime; print(onnx.__version__, onnxruntime.__version__)'
```

上述 `--no-deps` 方式依赖现有 `gym4` 已提供 NumPy、protobuf、packaging 等依赖；
在单独准备的 Python 环境中可让 pip 一并安装依赖：

```bash
python3 -m pip install onnx==1.17.0 onnxruntime==1.19.2 flatbuffers==24.3.25
```

临时目录被清理后需要重新安装，并在运行导出/验证命令时设置 `PYTHONPATH`。
完整导出命令见 [模型导出说明](rl-checkpoints/onnx/README_ONNX.md)。

## 仿真 GUI 编译（RL 三关节 Kp/Kd 实时调参）

若想生成 GUI，在上面的 ONNX 构建目录中重新配置 `NO_SIM`：

```bash
cd build-onnx

cmake .. -DNO_SIM=OFF

cmake --build . \
  --target sim mit_ctrl rapid_rl_policy_benchmark rapid_rl_policy_config_test -j

ctest --output-on-failure
```

`NO_SIM=OFF` 只增加 Qt GUI 的 `sim` 目标，不改变 `mit_ctrl` 的 CPU-only
ONNX Runtime 或实机 profile。若以后要恢复无 GUI 的精简构建，在相同目录执行
`cmake .. -DNO_SIM=ON` 即可。

当前 GUI 按工作目录中的 `../config` 和 `../resources` 查找文件，因此
`build` 或 `build-onnx` 必须直接位于源码根目录下，并从该目录启动：

```bash
# 终端 A
cd build-onnx
./sim/sim

# 终端 B：也从同一个 build-onnx 目录启动
cd build-onnx
./user/MIT_Controller/mit_ctrl m s
```

在 GUI 中选择 Mini Cheetah / Simulator 并启动后，切到 `RL_JOINT_PD`
（控制模式 53），即可在 User Parameters 表实时编辑
`rl_kp_joint=[abad, hip/thigh, knee/calf]` 和 `rl_kd_joint`。这两个参数只在
`mit_ctrl m s` 的仿真 profile 中生效，`mit_ctrl m r` 的实机增益保持固定。

## 部署验证

以下命令也在当前 `build` 或 `build-onnx` 目录执行。先运行观测、关节映射、
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
[LeggedGymRLBenchmark] policy=.../rl-checkpoints/onnx/policy.onnx
[LeggedGymRLBenchmark] input_shape=[1, 270] action_shape=[1, 12]
```

验收要求为所有输出有限且 `p95 < 18 ms`。控制器初始化时也应出现：

```text
[LeggedGymRL] Loaded ONNX Runtime policy: .../policy.onnx
[LeggedGymRL] Shapes: input [1, 270], action [1, 12]
```

ONNX Runtime 的 C++ 会话与推理接口见
[官方 API 文档](https://onnxruntime.ai/docs/api/c/struct_ort_1_1_session.html)。
旧 LibTorch 流程保留在 [`docs/rapid_rl_libtorch_deploy.md`](docs/rapid_rl_libtorch_deploy.md)，
仅用于历史参考，不适用于此分支。

### HIM ONNX 导航爬楼梯

导航输入、RB/RT 接管、断流行为及离线测试见 [导航爬楼梯接入说明](docs/himloco_nav_stairs.md)。
