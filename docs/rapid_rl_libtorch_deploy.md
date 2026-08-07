# Legged Gym RL LibTorch Deployment

`RL_JOINT_PD` is deployed through in-process LibTorch inference only. The
controller does not read `config/rapid-rl-policy.yaml` and does not accept
policy actions from `rl_policy_cmd`.

## Checkpoints

Runtime TorchScript checkpoint:

```bash
rl-checkpoints/model_1000.jit
```

It is exported from the HIMLoco `model_1000.pt` checkpoint. Training `.pt`
files are not loaded by the robot controller. The old
`legged_gym_policy_latest.jit`, `adaptation_module_latest.jit`, and
`body_latest.jit` files may remain, but `RL_JOINT_PD` does not load them.

The HIM TorchScript module has a CPU float32 `[1,270] -> [1,12]` contract. Its
input is six 45-D frames ordered newest first:

```text
per frame:
0:3     command * [2.0, 2.0, 0.25]
3:6     omegaBody * 0.25
6:9     projected gravity
9:21    q - q_default
21:33   qd * 0.05
33:45   previous actor action

history:
[current, t-1, t-2, t-3, t-4, t-5]
```

Each frame is clipped to `[-100,100]`. The model contains the HIM estimator,
which produces estimated velocity and latent features internally, plus the
actor. Therefore deployment must not prepend measured `vBody` or otherwise
shift these fields. Policy joint order and controller joint order are both
`FR, FL, RR, RL`, with no additional leg permutation. Observation history is
cleared when entering `RL_JOINT_PD`.

## CPU-only LibTorch

The exported actor has been verified with non-cxx11-abi CPU-only LibTorch
1.10.1 and `_GLIBCXX_USE_CXX11_ABI=0`.

From the source root, the recommended machine-independent install location is
`.deps/libtorch-1.10.1-cpu`:

```bash
mkdir -p .deps
curl -L --fail -o .deps/libtorch-shared-with-deps-1.10.1+cpu.zip \
  https://download.pytorch.org/libtorch/cpu/libtorch-shared-with-deps-1.10.1%2Bcpu.zip
unzip .deps/libtorch-shared-with-deps-1.10.1+cpu.zip -d .deps
mv .deps/libtorch .deps/libtorch-1.10.1-cpu
```

Some LibTorch 1.10.1 CPU packages require MKL shared libraries at link/runtime.
CMake automatically tries a valid cache, `$LIBTORCH_MKL_ROOT`,
`$CONDA_PREFIX/lib`, and source-relative `.deps/mkl/lib`. For an unusual location, copy
`cmake/RapidRLLocalConfig.cmake.example` to `cmake/RapidRLLocalConfig.cmake`
and set `RAPID_RL_LOCAL_MKL_ROOT` once; the local file is ignored by Git.

## Build from `build` or `build-cpu`

The dependency resolver accepts a valid explicit/cache path, then the local
config, `$LIBTORCH_ROOT`, source-root `.deps`, and parent `.deps`. This makes
the build independent of the enclosing workspace name and repairs stale cached
absolute paths automatically.

From the source root, create the initial no-GUI controller build:

```bash
mkdir -p build-cpu
cd build-cpu

cmake .. -DNO_SIM=ON

cmake --build . \
  --target mit_ctrl rapid_rl_policy_benchmark rapid_rl_policy_config_test -j
```

After the initial configure, the second command is the normal incremental
build command. No workspace path, `LIBTORCH_ROOT`, MKL path, CPU-only flag, or
CMake policy flag needs to be repeated.

## Simulator GUI Build (runtime RL PD tuning)

The same `build-cpu` directory may also generate the Qt `sim` target. To adjust
`rl_kp_joint` and `rl_kd_joint` at runtime, reconfigure that directory once:

```bash
cd build-cpu

cmake .. -DNO_SIM=OFF

cmake --build . \
  --target sim mit_ctrl rapid_rl_policy_benchmark rapid_rl_policy_config_test -j

ctest --output-on-failure
```

`NO_SIM=OFF` adds only the GUI target; it does not change the CPU-only LibTorch
controller or the real-robot PD profile. Reconfigure with `NO_SIM=ON` later if
the GUI target is no longer wanted. Qt5 Core/Widgets/Gui/Gamepad and OpenGL are
required for this mode; set `CMAKE_PREFIX_PATH` in the local config only for a
nonstandard Qt installation.

The GUI resolves `../config` and `../resources` from its working directory, so
`build` or `build-cpu` must be immediately below the source root. Start both
programs from that directory:

```bash
# Terminal A
cd build-cpu
./sim/sim

# Terminal B, using the same working directory
cd build-cpu
./user/MIT_Controller/mit_ctrl m s
```

After starting Mini Cheetah / Simulator and selecting `RL_JOINT_PD` (mode 53),
the User Parameters table can edit `rl_kp_joint` and `rl_kd_joint` in
`[abad, hip/thigh, knee/calf]` order. The three values are shared by all four
legs, take effect in the simulator on the next low-level control cycle, and
are ignored by the `mit_ctrl m r` real-robot profile.

## Verify

From the current `build` or `build-cpu` directory, run the focused observation,
mapping, default-angle, action-scale, and PD-gain test:

```bash
ctest --output-on-failure
```

The PD test keeps three separately named profiles.  Training gains are policy
metadata only; `mit_ctrl m s` selects the simulator profile and `mit_ctrl m r`
selects the validated real-robot profile.  The latter must remain
`Kp=[20,20,20]`, `Kd=[0.5,0.5,0.5]` unless a separately reviewed hardware
change is intended.  Controller startup prints the selected profile and gains.

The CPU-only build must not depend on CUDA and must resolve all shared
libraries:

```bash
ldd user/MIT_Controller/mit_ctrl | grep -E "cuda|not found"
```

The command above must print nothing.

Benchmark the exact checkpoint and LibTorch build:

```bash
./user/MIT_Controller/rapid_rl_policy_benchmark 1000
```

Expected shape output:

```text
policy=.../rl-checkpoints/model_1000.jit
input_shape=[1, 270] action_shape=[1, 12]
```

Target latency:

```text
p95 < 18 ms
```

All policy outputs must remain finite. Controller startup must report the same
policy path and `[1,270] -> [1,12]` shapes.
