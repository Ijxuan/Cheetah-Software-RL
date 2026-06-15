# Rapid RL LibTorch Deployment

`RL_JOINT_PD` is deployed through in-process LibTorch inference only. The
controller does not read `config/rapid-rl-policy.yaml` and does not accept
policy actions from `rl_policy_cmd`.

## Checkpoints

Runtime TorchScript checkpoints live in:

```bash
rl-checkpoints/adaptation_module_latest.jit
rl-checkpoints/body_latest.jit
```

After retraining, replace only these two `.jit` files with the same filenames.
Training `.pt` files are not used by the robot controller.

## CPU-only LibTorch

The current training environment uses PyTorch 1.10.1 with
`_GLIBCXX_USE_CXX11_ABI=0`, so use the non-cxx11-abi CPU-only LibTorch 1.10.1
package.

Example user-writable install:

```bash
mkdir -p /home/xjtx/rl-deploy/.deps
curl -L --fail -o /home/xjtx/rl-deploy/.deps/libtorch-shared-with-deps-1.10.1+cpu.zip \
  https://download.pytorch.org/libtorch/cpu/libtorch-shared-with-deps-1.10.1%2Bcpu.zip
unzip /home/xjtx/rl-deploy/.deps/libtorch-shared-with-deps-1.10.1+cpu.zip \
  -d /home/xjtx/rl-deploy/.deps
mv /home/xjtx/rl-deploy/.deps/libtorch \
  /home/xjtx/rl-deploy/.deps/libtorch-1.10.1-cpu
```

Some LibTorch 1.10.1 CPU packages require MKL shared libraries at link/runtime.
Pass `LIBTORCH_MKL_ROOT` if CMake or the linker reports missing `mkl_*`
symbols.

## Build

```bash
cmake -S Cheetah-Software-RL -B Cheetah-Software-RL/build-cpu \
  -DNO_SIM=ON \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DLIBTORCH_ROOT=/home/xjtx/rl-deploy/.deps/libtorch-1.10.1-cpu \
  -DLIBTORCH_CPU_ONLY=ON \
  -DLIBTORCH_MKL_ROOT=/opt/miniconda3/envs/unitree-rl/lib

cmake --build Cheetah-Software-RL/build-cpu \
  --target mit_ctrl rapid_rl_policy_benchmark -j
```

`LIBTORCH_MKL_ROOT` can be omitted when the target system provides a CPU-only
LibTorch package that does not need separate MKL libraries.

## Verify

The CPU-only build must not depend on CUDA and must resolve all shared
libraries:

```bash
ldd Cheetah-Software-RL/build-cpu/user/MIT_Controller/mit_ctrl | grep -E "cuda|not found"
```

The command above must print nothing.

Benchmark the exact checkpoint and LibTorch build:

```bash
./Cheetah-Software-RL/build-cpu/user/MIT_Controller/rapid_rl_policy_benchmark 1000
```

Expected shape output:

```text
latent_shape=[1, 18] action_shape=[1, 12]
```

Target latency:

```text
p95 < 20 ms
```

The preferred target is `p95 < 18 ms` to leave margin in the 50 Hz control
loop.
