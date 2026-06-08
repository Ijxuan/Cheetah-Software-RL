#ifndef RAPID_RL_BUILD_CONFIG_H
#define RAPID_RL_BUILD_CONFIG_H

namespace rapid_rl {
namespace build_config {

constexpr bool kUseLibTorch = @RAPID_RL_USE_LIBTORCH_VALUE@;
constexpr bool kLibTorchCpuOnly = @RAPID_RL_LIBTORCH_CPU_ONLY_VALUE@;
constexpr int kLibTorchCxx11Abi = @RAPID_RL_LIBTORCH_CXX11_ABI@;
constexpr const char* kCheckpointDir = "@RAPID_RL_CHECKPOINT_DIR@";
constexpr int kWarmupIters = @RAPID_RL_WARMUP_ITERS@;
constexpr float kMaxInferenceMs =
    static_cast<float>(@RAPID_RL_MAX_INFERENCE_MS@);
constexpr int kTorchNumThreads = @RAPID_RL_TORCH_NUM_THREADS@;
constexpr bool kDebugLcmState = @RAPID_RL_DEBUG_LCM_STATE_VALUE@;

}  // namespace build_config
}  // namespace rapid_rl

#endif  // RAPID_RL_BUILD_CONFIG_H
