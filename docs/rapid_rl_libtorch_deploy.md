# Legged Gym RL LibTorch Deployment

`RL_JOINT_PD` is deployed through in-process LibTorch inference only. The
controller does not read `config/rapid-rl-policy.yaml` and does not accept
policy actions from `rl_policy_cmd`.

## Checkpoints

Runtime TorchScript checkpoints live in:

```bash
rl-checkpoints/legged_gym_policy_latest.jit
```

After retraining, export the deterministic actor and replace this single `.jit`
file with the same filename. Training `.pt` files are not used by the robot
controller. The old `adaptation_module_latest.jit` and `body_latest.jit` files
may remain in the directory, but `RL_JOINT_PD` does not load them. The source
tree now expects the flat-policy 48-D interface below, so the legacy 235-D
checkpoint must not be used after rebuilding the controller; replace both the
binary and this checkpoint together.

The flat policy is a CPU float32 `48 -> 12` actor. Its observations are:

```text
0:3     vBody * 2.0
3:6     omegaBody * 0.25
6:9     projected gravity
9:12    command * [2.0, 2.0, 0.25]
12:24   q - q_default
24:36   qd * 0.05
36:48   previous actor action
```

The complete observation is clipped to `[-100, 100]`. Policy joint order is
`FL, FR, RL, RR`; controller joint order is `FR, FL, RR, RL`.

## CPU-only LibTorch

The exported actor has been verified with non-cxx11-abi CPU-only LibTorch
1.10.1 and `_GLIBCXX_USE_CXX11_ABI=0`.

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
  --target mit_ctrl rapid_rl_policy_benchmark rapid_rl_policy_config_test -j
```

`LIBTORCH_MKL_ROOT` can be omitted when the target system provides a CPU-only
LibTorch package that does not need separate MKL libraries.

## Simulator GUI Build (runtime RL PD tuning)

The CPU-only deployment build above is unchanged: it deliberately uses
`NO_SIM=ON` and does not create the Qt `sim` target. To adjust
`rl_kp_joint` and `rl_kd_joint` at runtime in the simulator, configure a
separate build directory with `NO_SIM=OFF`:

```bash
cd Cheetah-Software-RL

cmake -S . -B build-sim-cpu \
  -DNO_SIM=OFF \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DLIBTORCH_ROOT=/home/xjtx/rl-deploy/.deps/libtorch-1.10.1-cpu \
  -DLIBTORCH_CPU_ONLY=ON \
  -DLIBTORCH_MKL_ROOT=/opt/miniconda3/envs/unitree-rl/lib

cmake --build build-sim-cpu \
  --target sim mit_ctrl rapid_rl_policy_benchmark rapid_rl_policy_config_test -j

ctest --test-dir build-sim-cpu --output-on-failure
```

Do not reuse `build-cpu`: its CMake cache has `NO_SIM=ON`, so it has no `sim`
target. The GUI currently resolves `../config` and `../resources` from its
working directory, so keep `build-sim-cpu` immediately under the source root
and start both programs from that directory:

```bash
# Terminal A
cd Cheetah-Software-RL/build-sim-cpu
./sim/sim

# Terminal B, using the same working directory
cd Cheetah-Software-RL/build-sim-cpu
./user/MIT_Controller/mit_ctrl m s
```

After starting Mini Cheetah / Simulator and selecting `RL_JOINT_PD` (mode 53),
the User Parameters table can edit `rl_kp_joint` and `rl_kd_joint` in
`[abad, hip/thigh, knee/calf]` order. The three values are shared by all four
legs, take effect in the simulator on the next low-level control cycle, and
are ignored by the `mit_ctrl m r` real-robot profile.

## Verify

Run the focused observation, mapping, default-angle, action-scale, and PD-gain
test:

```bash
ctest --test-dir Cheetah-Software-RL/build-cpu --output-on-failure
```

The PD test keeps three separately named profiles.  Training gains are policy
metadata only; `mit_ctrl m s` selects the simulator profile and `mit_ctrl m r`
selects the validated real-robot profile.  The latter must remain
`Kp=[20,20,20]`, `Kd=[0.5,0.5,0.5]` unless a separately reviewed hardware
change is intended.  Controller startup prints the selected profile and gains.

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
policy=.../rl-checkpoints/legged_gym_policy_latest.jit
input_shape=[1, 48] action_shape=[1, 12]
```

Target latency:

```text
p95 < 18 ms
```

All policy outputs must remain finite. Controller startup must report the same
single policy path and `[1,48] -> [1,12]` shapes.
