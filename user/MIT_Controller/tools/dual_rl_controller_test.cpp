#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <time.h>

#include "FSM_States/ControlFSM.h"
#include "Dynamics/MiniCheetah.h"
#include "main_helper.h"

static void check(bool ok, const char* message) {
  if (!ok) { std::cerr << message << '\n'; std::exit(1); }
}
static int estimator_runs = 0;
class CountingEstimator : public GenericEstimator<float> {
 public:
  void setup() override {}
  void run() override { ++estimator_runs; }
};

int main() {
  using namespace rapid_rl;
  using Phase = RLPolicySwitch::Phase;
  gMasterConfig.simulated = false;
  auto quad = buildMiniCheetah<float>();
  LegController<float> legs(quad);
  RobotControlParameters params;
  params.use_rc = 1; params.control_mode = K_PASSIVE; params.controller_dt = .002;
  StateEstimate<float> estimate{};
  estimate.rBody.setIdentity(); estimate.rpy.setZero();
  estimate.vBody.setZero(); estimate.omegaBody.setZero();
  StateEstimatorContainer<float> estimator(nullptr, nullptr, legs.datas, &estimate, &params);
  estimator.addEstimator<CountingEstimator>();
  rc_control_settings rc{};
  rc.mode = RC_mode::OFF;
  rc.rl_selection_sequence = 1;
  rc.v_des[0] = .3;
  GamepadCommand gamepad{};
  DesiredStateCommand<float> desired(&gamepad, &rc, &params, &estimate, .002f);
  ControlFSMData<float> data{};
  data._quadruped = &quad; data._legController = &legs;
  data._stateEstimator = &estimator; data._desiredStateCommand = &desired;
  data.controlParameters = &params;
  const JointVector initial = FlatPolicyRunner::defaultQ();
  for (int leg = 0; leg < 4; ++leg) {
    legs.datas[leg].q = initial.segment<3>(3 * leg);
    legs.datas[leg].qd.setZero();
    legs.datas[leg].p << 0.f, 0.f, -.28f;
  }
  int64_t time = 1000000;
  auto context = std::make_shared<RLControlContext<float>>(&data, false, [&] { return time; });
  auto& state = *context;
  FSM_State_RLFlat<float> flat(&data, context);
  FSM_State_RLStairs<float> stairs(&data, context);
  FSM_State_Passive<float> passive(&data);
  FSM_State_RecoveryStand<float> recovery(&data);
  FSM_StatesList<float> registry{};
  registry.rlFlat = &flat; registry.rlStairs = &stairs;
  registry.passive = &passive; registry.recoveryStand = &recovery;
  ControlFSM<float> fsm(data, registry, context);
  check(fsm.getNextState(FSM_StateName::RL_FLAT) == &flat &&
        fsm.getNextState(FSM_StateName::RL_STAIRS) == &stairs,
        "ControlFSM did not register two independent state objects");
  auto recoverInto = [&](int mode) {
    rc.mode = RC_mode::OFF;
    fsm.initialize(); // same reset invoked by RobotRunner on OFF
    check(fsm.currentState == &passive && !state.flatHistory().ready() &&
          !state.stairsRunner().historyReady(), "OFF left stale state/history");
    rc.mode = RC_mode::RECOVERY_STAND; time += 2000; fsm.runFSM();
    check(fsm.currentState == &recovery && recovery.stateName == FSM_StateName::RECOVERY_STAND,
          "recovery state identity incorrect");
    rc.mode = mode; ++rc.rl_selection_sequence; time += 2000; fsm.runFSM();
    check(fsm.currentState == (mode == RC_mode::RL_FLAT ?
          static_cast<FSM_State<float>*>(&flat) : static_cast<FSM_State<float>*>(&stairs)),
          "recovery did not enter requested RL state");
  };
  recoverInto(RC_mode::RL_FLAT);
  auto target = [&] {
    JointVector q;
    for (int leg = 0; leg < 4; ++leg) q.segment<3>(leg * 3) = legs.commands[leg].qDes;
    return q;
  };
  float max_blend_delta = 0.f;
  auto step = [&](int n) {
    for (int i = 0; i < n; ++i) {
      time += 2000;
      const auto before_phase = state.policySwitch().phase();
      const auto before_controller = state.policySwitch().active();
      const JointVector previous = target();
      const int64_t sample_time = state.flatHistory().latest().time_us;
      const auto him_history = state.stairsRunner().observationHistory();
      fsm.runFSM();
      check(fsm.currentState == (state.policySwitch().active() == RLController::Flat ?
            static_cast<FSM_State<float>*>(&flat) : static_cast<FSM_State<float>*>(&stairs)),
            "policy owner diverged from ControlFSM.currentState");
      check(state.activeMode() == (fsm.currentState == &flat ? 54 : 53),
            "debug active mode reports the request instead of actual state");
      if (before_controller != state.policySwitch().active()) {
        check(sample_time == state.flatHistory().latest().time_us &&
              (him_history - state.stairsRunner().observationHistory()).norm() == 0.f &&
              state.flatHistory().ready() && state.stairsRunner().historyReady(),
              "FSM onExit/onEnter reset or rolled shared history");
      }
      check(legs._legsEnabled, "unexpected controller disable");
      const auto after_phase = state.policySwitch().phase();
      if (before_phase == Phase::Blending || after_phase == Phase::Blending ||
          before_controller != state.policySwitch().active()) {
        const float delta = (target() - previous).cwiseAbs().maxCoeff();
        max_blend_delta = std::max(max_blend_delta, delta);
        check(delta <= .05001f, "handover target step exceeded .05 rad");
      }
      check(target().allFinite(), "non-finite joint target");
      // A held nonzero RC command must return after every completed handover.
      if (params.use_rc && before_phase == Phase::Running && after_phase == Phase::Running &&
          state.flatHistory().ready() && state.flatHistory().latest().time_us == time)
        check(std::fabs(state.flatHistory().latest().command[0] - float(rc.v_des[0])) < 1e-6f,
              "forward command remained zero after handover");
      if (state.policySwitch().zeroCommand() && state.flatHistory().ready() &&
          state.flatHistory().latest().time_us == time)
        check(state.flatHistory().latest().command.norm() == 0.f, "switch failed to command zero");
    }
  };
  step(150);
  check(state.policySwitch().active() == RLController::Flat &&
        state.policySwitch().phase() == Phase::Running, "flat startup failed");
  check(state.flatHistory().ready() && state.stairsRunner().historyReady(),
        "inactive policy history not maintained");
  check(std::fabs(legs.commands[0].kpJoint(0, 0) - 17.f) < 1e-6f, "legacy flat gains changed");
  InferenceTimingSummary timing;
  check(!state.stairsRunner().timingSummary(&timing), "inactive HIM policy ran inference");
  rc.mode = RC_mode::RL_STAIRS; ++rc.rl_selection_sequence;
  estimate.omegaBody[0] = .2f;
  step(150);
  check(state.policySwitch().active() == RLController::Flat &&
        state.policySwitch().phase() == Phase::Stopping, "switched at angular-speed limit");
  estimate.omegaBody[0] = .19f;
  estimate.vBody[0] = .3f;
  for (int leg = 0; leg < 4; ++leg) legs.datas[leg].qd.setConstant(.8f);
  step(450);
  check(state.policySwitch().active() == RLController::Stairs &&
        state.policySwitch().phase() == Phase::Running, "flat -> stairs failed");
  check(state.stairsRunner().timingSummary(&timing) && timing.count > 0,
        "stairs never ran inference");
  step(1200); // Exercise the full two-second post-handover diagnostic window.

  check(std::fabs(legs.commands[0].kpJoint(0, 0) - 20.f) < 1e-6f, "HIM real robot gains changed");
  estimate.vBody.setZero(); estimate.omegaBody.setZero();
  for (int leg = 0; leg < 4; ++leg) legs.datas[leg].qd.setZero();
  rc.mode = RC_mode::RL_FLAT; ++rc.rl_selection_sequence;
  estimate.rpy[0] = 31.f * float(M_PI) / 180.f;
  estimate.rBody = Eigen::AngleAxisf(estimate.rpy[0], Eigen::Vector3f::UnitX()).toRotationMatrix();
  step(150);
  check(fsm.currentState == &stairs && state.policySwitch().phase() == Phase::Stopping,
        "entered flat above 30 degree tilt limit");
  estimate.rpy.setZero(); estimate.rBody.setIdentity();
  step(450);
  check(state.policySwitch().active() == RLController::Flat &&
        state.policySwitch().phase() == Phase::Running, "stairs -> flat failed");
  const auto count_before = timing.count;
  state.stairsRunner().timingSummary(&timing);
  check(timing.count >= count_before, "invalid inference counter");
  const auto count_flat = timing.count;
  step(50);
  state.stairsRunner().timingSummary(&timing);
  check(timing.count == count_flat, "inactive HIM inference kept running");
  rc.mode = RC_mode::RL_STAIRS; ++rc.rl_selection_sequence;
  estimate.omegaBody[0] = .2f;
  step(2600);
  check(state.policySwitch().phase() == Phase::Blocked &&
        state.policySwitch().active() == RLController::Flat, "handover timeout failed");
  estimate.omegaBody.setZero(); step(200);
  check(state.policySwitch().phase() == Phase::Blocked, "timeout resumed automatically");
  ++rc.rl_selection_sequence; step(450);
  check(state.policySwitch().active() == RLController::Stairs &&
        state.policySwitch().phase() == Phase::Running, "fresh selection did not retry");
  check(estimator_runs == 0, "policy ran state estimator a second time");
  estimate.rpy[0] = .8f; time += 2000; fsm.runFSM();
  rc_control_settings shared{}; get_rc_control_settings(&shared);
  check(!legs._legsEnabled && shared.mode == RC_mode::OFF && params.control_mode == K_PASSIVE,
        "orientation failure did not disable and latch OFF");
  estimate.rpy.setZero(); time += 2000; fsm.runFSM();
  check(!legs._legsEnabled, "safety latch cleared without reentry");
  recoverInto(RC_mode::RL_STAIRS);
  check(!state.flatHistory().ready() && !state.stairsRunner().historyReady(),
        "reentry reused stale policy history");
  time += 10000;
  legs.datas[0].q[0] = std::numeric_limits<float>::quiet_NaN();
  fsm.runFSM(); get_rc_control_settings(&shared);
  check(!legs._legsEnabled && shared.mode == RC_mode::OFF, "NaN state did not request OFF");
  legs.datas[0].q[0] = initial[0];
  recoverInto(RC_mode::RL_FLAT);
  estimate.rpy[0] = 31.f * float(M_PI) / 180.f;
  estimate.rBody = Eigen::AngleAxisf(estimate.rpy[0], Eigen::Vector3f::UnitX()).toRotationMatrix();
  time += 10000; fsm.runFSM(); get_rc_control_settings(&shared);
  check(!legs._legsEnabled && shared.mode == RC_mode::OFF,
        "legacy flat 30 degree total-tilt protection lost");
  estimate.rpy.setZero(); estimate.rBody.setIdentity();
  recoverInto(RC_mode::RL_FLAT); step(150);
  rc.mode = RC_mode::RL_STAIRS; ++rc.rl_selection_sequence; step(170);
  check(fsm.currentState == &stairs && state.policySwitch().phase() == Phase::Blending,
        "new state did not own the blending phase");
  rc.mode = RC_mode::RECOVERY_STAND; time += 2000; fsm.runFSM();
  check(fsm.currentState == &recovery && !state.flatHistory().ready() &&
        !state.stairsRunner().historyReady(), "LT+A did not interrupt blend and clear history");
  recoverInto(RC_mode::RL_FLAT); step(150);
  rc.mode = RC_mode::RL_STAIRS; ++rc.rl_selection_sequence; step(170);
  rc.mode = RC_mode::OFF; time += 2000; fsm.runFSM();
  check(fsm.currentState == &passive && !legs._legsEnabled && !state.flatHistory().ready(),
        "RC OFF did not interrupt blend through the top-level FSM");
  recoverInto(RC_mode::RL_FLAT); step(150);
  params.use_rc = 0; params.control_mode = K_RL_STAIRS; step(460);
  check(fsm.currentState == &stairs, "GUI mode 53 did not transition to stairs");
  params.control_mode = K_RL_FLAT; step(460);
  check(fsm.currentState == &flat, "GUI mode 54 did not transition to flat");
  params.control_mode = K_PASSIVE; time += 2000; fsm.runFSM();
  check(fsm.currentState == &passive && !state.flatHistory().ready(), "GUI passive left stale histories");
  std::cout << "PASS: ControlFSM real state identities, persistent histories, both handovers, OFF/recovery/GUI; max blend delta="
            << max_blend_delta << " rad; no motor interface started\n";
}
