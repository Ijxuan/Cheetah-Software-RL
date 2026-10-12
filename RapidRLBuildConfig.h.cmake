#ifndef RAPID_RL_BUILD_CONFIG_H
#define RAPID_RL_BUILD_CONFIG_H

namespace rapid_rl {
namespace build_config {

constexpr const char* kCheckpointDir = "@RAPID_RL_CHECKPOINT_DIR@";
constexpr const char* kCheckpointFile = "@RAPID_RL_CHECKPOINT_FILE@";
constexpr int kWarmupIters = @RAPID_RL_WARMUP_ITERS@;
constexpr float kMaxInferenceMs =
    static_cast<float>(@RAPID_RL_MAX_INFERENCE_MS@);
constexpr int kOnnxNumThreads = @RAPID_RL_ONNX_NUM_THREADS@;
constexpr bool kDebugLcmState = @RAPID_RL_DEBUG_LCM_STATE_VALUE@;
constexpr int kEthercatCpuCore = @RAPID_RL_ETHERCAT_CPU_CORE@;
constexpr int kEthercatRtPriority = @RAPID_RL_ETHERCAT_RT_PRIORITY@;

}  // namespace build_config
}  // namespace rapid_rl

#endif  // RAPID_RL_BUILD_CONFIG_H
