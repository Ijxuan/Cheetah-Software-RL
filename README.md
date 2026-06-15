## Concurrent Training of a Control Policy and a State Estimator for Dynamic and Robust Legged Locomotion
This repository is for sharing our implementation code on the Mini cheetah.
This software is based on the work of MIT-biomimetics Lab.
For the instruction about build, use of simulation, and deployment on the robot, please visit the original repository (https://github.com/mit-biomimetics/Cheetah-Software).

We inserted our controller, RLJointPD, inside the MIT_Controller.

To run our controller in simulation, first change the vairable 'isMinicheetah' to false.
'isMinicheetah' is located in 'user/MIT_Controller/FSM_States/FSM_State_RLJointPD.cpp'.
Then, follow the below code to run mit_ctrl.
```
cd build
make -j4
./user/MIT_Controller/mit_ctrl m s
```
After running mit_ctrl, change 'use_rc' to 0 and 'control_mode' to 53 in SimControlPanel.


To run our controller on the real robot, change the variable 'isMinicheetah' to true.
If 'isMinicheetah' is set to be false, then the real robot cannot walk well because of wrong PD gains.

Concerning the RC controller, we removed all the other locomotion mode and replaced them with our controller.
By choosing the 'run' mode on the RC controller, RLJointPD mode is executed.

## Real Robot CPU-only LibTorch Build

`RL_JOINT_PD` now uses in-process LibTorch inference only. Build the real-robot
controller against a CPU-only LibTorch package; do not use the CUDA PyTorch
package on the onboard computer.

Install or copy the CPU-only LibTorch package first. The expected ABI for the
current exported policy is `_GLIBCXX_USE_CXX11_ABI=0`, so use the non-cxx11-abi
LibTorch 1.10.1 CPU package. The examples below assume it is available at
`/opt/libtorch-1.10.1-cpu`.

From the deployment workspace root:

```bash
cmake -S Cheetah-Software-RL -B Cheetah-Software-RL/build-cpu \
  -DNO_SIM=ON \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DLIBTORCH_ROOT=/opt/libtorch-1.10.1-cpu \
  -DLIBTORCH_CPU_ONLY=ON

cmake --build Cheetah-Software-RL/build-cpu \
  --target mit_ctrl rapid_rl_policy_benchmark -j
```

If linking reports missing `mkl_*` symbols, pass the directory containing MKL
shared libraries:

```bash
cmake -S Cheetah-Software-RL -B Cheetah-Software-RL/build-cpu \
  -DNO_SIM=ON \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DLIBTORCH_ROOT=/opt/libtorch-1.10.1-cpu \
  -DLIBTORCH_CPU_ONLY=ON \
  -DLIBTORCH_MKL_ROOT=/path/to/mkl/lib
```

Verify that the real-robot binary has no CUDA or unresolved dependencies:

```bash
ldd Cheetah-Software-RL/build-cpu/user/MIT_Controller/mit_ctrl | grep -E "cuda|not found"
```

The command above must print nothing. Then benchmark the exact policy loaded by
the controller:

```bash
./Cheetah-Software-RL/build-cpu/user/MIT_Controller/rapid_rl_policy_benchmark 1000
```

Expected output includes `latent_shape=[1, 18] action_shape=[1, 12]`. The p95
latency must be below 20 ms, preferably below 18 ms. For the full deployment
notes and checkpoint replacement flow, see
`docs/rapid_rl_libtorch_deploy.md`.
