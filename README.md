## 动态鲁棒四足运动控制策略与状态估计器联合训练

本仓库用于 Mini Cheetah 相关实现代码。本软件基于 MIT Biomimetics
Lab 的 Cheetah-Software。原始构建、仿真和实机部署说明可参考：
https://github.com/mit-biomimetics/Cheetah-Software

本仓库在 `MIT_Controller` 中加入了 `RLJointPD` 控制器。

## 仿真运行

如需在仿真中运行控制器，先将变量 `isMinicheetah` 改为 `false`。该变量
位于：

```text
user/MIT_Controller/FSM_States/FSM_State_RLJointPD.cpp
```

然后构建并启动 `mit_ctrl`：

```bash
cd build
make -j4
./user/MIT_Controller/mit_ctrl m s
```

启动 `mit_ctrl` 后，在 `SimControlPanel` 中将 `use_rc` 改为 `0`，并将
`control_mode` 改为 `53`。

## 实机运行

如需在真实机器人上运行控制器，将 `isMinicheetah` 改为 `true`。如果实机
运行时该变量为 `false`，PD 增益会不匹配，机器人运动表现会异常。

遥控器模式中，原有其他 locomotion mode 已被本控制器替换。选择遥控器的
`run` 模式会进入 `RLJointPD`。

## 实机 CPU-only LibTorch 编译

`RL_JOINT_PD` 现在只使用 C++ 进程内 LibTorch 推理。机载计算机没有独显
时，必须使用 CPU-only LibTorch 编译，不要使用带 CUDA 的 PyTorch/LibTorch
包。

当前导出的策略对应 `_GLIBCXX_USE_CXX11_ABI=0`，因此优先使用非 cxx11-abi
的 LibTorch 1.10.1 CPU 包。下面示例假设 CPU-only LibTorch 已安装或解压到：

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
  --target mit_ctrl rapid_rl_policy_benchmark -j
```

如果链接时报缺少 `mkl_*` 符号，说明当前 CPU-only LibTorch 还需要 MKL
动态库。此时重新配置时传入 MKL 库目录：

```bash
cmake -S Cheetah-Software-RL -B Cheetah-Software-RL/build-cpu \
  -DNO_SIM=ON \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DLIBTORCH_ROOT=/opt/libtorch-1.10.1-cpu \
  -DLIBTORCH_CPU_ONLY=ON \
  -DLIBTORCH_MKL_ROOT=/path/to/mkl/lib
```

编译完成后检查实机控制器二进制不能依赖 CUDA，也不能有未解析动态库：

```bash
ldd Cheetah-Software-RL/build-cpu/user/MIT_Controller/mit_ctrl | grep -E "cuda|not found"
```

上面的命令必须没有任何输出。

然后用同一套 checkpoint 和 LibTorch 构建运行 benchmark：

```bash
./Cheetah-Software-RL/build-cpu/user/MIT_Controller/rapid_rl_policy_benchmark 1000
```

预期输出应包含：

```text
latent_shape=[1, 18] action_shape=[1, 12]
```

推理延迟要求：`p95 < 20 ms`；建议目标是 `p95 < 18 ms`，给 50 Hz 控制循环
留出余量。完整部署说明和 checkpoint 替换流程见：

```text
docs/rapid_rl_libtorch_deploy.md
```
