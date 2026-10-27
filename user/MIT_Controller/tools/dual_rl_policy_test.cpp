#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <unistd.h>

#include "FSM_States/FlatPolicyRunner.h"
#include "FSM_States/RapidRLPolicyRunner.h"
#include "FSM_States/RLPolicySwitch.h"
#include "RapidRLBuildConfig.h"

static void check(bool ok, const char* message) {
  if (!ok) { std::cerr << message << '\n'; std::exit(1); }
}

int main() {
  using namespace rapid_rl;
  FlatPolicyHistory history;
  Eigen::Matrix<float, 72, 1> errors = Eigen::Matrix<float, 72, 1>::Zero();
  Eigen::Matrix<float, 72, 1> velocities = errors;
  JointVector previous = JointVector::Zero();
  float max_observation_error = 0.f;
  for (int t = 0; t < 24; ++t) {
    RLStateSample s;
    s.time_us = 1000000 + t * 10000;
    s.r_body = Eigen::AngleAxisf(.01f * t, Eigen::Vector3f::UnitX()).toRotationMatrix();
    s.omega << .1f, .2f, .3f;
    s.command << .2f, -.1f, .4f;
    for (int j = 0; j < 12; ++j) {
      s.q[j] = .1f * t + j;
      s.qd[j] = .2f * t - j;
      s.applied_target[j] = .3f * t + .01f * j;
      s.feet[j] = -.1f * j;
    }
    errors.head<60>() = errors.tail<60>().eval();
    errors.tail<12>() = s.applied_target - s.q;
    velocities.head<60>() = velocities.tail<60>().eval();
    velocities.tail<12>() = s.qd;
    history.append(s);
    if (t >= 5) {
      FlatObservation legacy;
      legacy << s.r_body.transpose().row(2).transpose(), s.q, s.omega, s.qd,
          s.applied_target, previous, errors.segment<12>(0), velocities.segment<12>(0),
          errors.segment<12>(24), velocities.segment<12>(24),
          errors.segment<12>(48), velocities.segment<12>(48), s.feet, s.command;
      max_observation_error = std::max(max_observation_error,
          (legacy - history.observation()).cwiseAbs().maxCoeff());
    }
    previous = s.applied_target;
  }
  check(history.ready() && max_observation_error == 0.f, "flat history differs from legacy layout");
  auto gap = history.latest(); gap.time_us += 40000;
  history.append(gap);
  check(!history.ready(), "flat history reused samples across a gap");

  FlatPolicyRunner flat;
  check(flat.load(build_config::kFlatModelDir), flat.error().c_str());
  // Independent legacy CPU network invocation on the same recorded observation.
  rai::FuncApprox::MLP_fullyconnected<float, 152, 12,
      rai::FuncApprox::ActivationType::leakyrelu> actor({512, 256, 64});
  rai::FuncApprox::MLP_fullyconnected<float, 141, 11,
      rai::FuncApprox::ActivationType::leakyrelu> estimator({256, 128});
  const std::string base = std::string(build_config::kFlatModelDir) + "/";
  actor.updateParamFromTxt(base + "actor_5000.txt");
  estimator.updateParamFromTxt(base + "estimator_5000.txt");
  FlatObservation raw, normalized, means, variances;
  std::ifstream mean_file(base + "mean5000.csv"), var_file(base + "var5000.csv");
  for (int i = 0; i < 141; ++i) { mean_file >> means[i]; var_file >> variances[i]; }
  float max_flat_error = 0.f, flat_max_ms = 0.f;
  for (int t = 0; t < 24; ++t) {
    for (int j = 0; j < 141; ++j) raw[j] = .01f * std::sin(float(t + j));
    raw.segment<3>(0) << 0, 0, 1;
    raw.segment<12>(3) = FlatPolicyRunner::defaultQ();
    raw.segment<12>(30) = FlatPolicyRunner::defaultQ();
    raw.segment<12>(42) = FlatPolicyRunner::defaultQ();
    for (int j = 0; j < 141; ++j)
      normalized[j] = std::max(-10.f, std::min(10.f,
          (raw[j] - means[j]) / std::sqrt(variances[j] + 1e-8f)));
    Eigen::Matrix<float, 152, 1> actor_obs;
    actor_obs << normalized, estimator.forward(normalized);
    const JointVector expected = actor.forward(actor_obs) * .1f + FlatPolicyRunner::defaultQ();
    JointVector actual;
    float ms; std::string error;
    check(flat.infer(raw, &actual, &ms, &error), error.c_str());
    flat_max_ms = std::max(flat_max_ms, ms);
    max_flat_error = std::max(max_flat_error, (expected - actual).cwiseAbs().maxCoeff());
  }
  check(max_flat_error < 1e-5f, "flat model differs from legacy inference");
  raw[0] = std::numeric_limits<float>::quiet_NaN();
  JointVector out; float ms; std::string error;
  check(!flat.infer(raw, &out, &ms, &error), "flat accepted NaN");
  FlatPolicyRunner bad;
  check(!bad.load("/nonexistent-dual-rl-model"), "missing flat model accepted");
  char temp[] = "/tmp/dual-rl-model-XXXXXX";
  const char* dir = mkdtemp(temp); check(dir, "mkdtemp failed");
  { std::ofstream file(std::string(dir) + "/actor_5000.txt"); file << "1,2,3\n"; }
  check(!bad.load(dir), "truncated flat weights accepted");
  // Use read-only links for the large weights; normalization failures must
  // still be rejected after valid weight validation, before any inference.
  std::filesystem::remove(std::string(dir) + "/actor_5000.txt");
  for (const auto* name : {"actor_5000.txt", "estimator_5000.txt", "mean5000.csv"})
    std::filesystem::create_symlink(base + name, std::string(dir) + "/" + name);
  check(!bad.load(dir), "missing variance file accepted");
  { std::ofstream file(std::string(dir) + "/var5000.csv"); file << "-1\n"; }
  check(!bad.load(dir), "negative variance accepted");
  std::filesystem::remove_all(dir);

  OnnxPolicyRunner combined, separated;
  check(combined.load() && separated.load(), "HIM model load failed");
  float max_him_error = 0.f;
  JointVector previous_action = JointVector::Zero();
  for (int t = 0; t < 24; ++t) {
    if (t == 12) {
      combined.resetObservationHistory(); separated.resetObservationHistory();
      check(!separated.historyReady(), "HIM reset retained readiness");
    }
    const Eigen::Vector3f omega(.01f * t, -.02f, .03f), gravity(0, 0, -1), command(.2f, 0, .1f);
    const JointVector q = DefaultJointPositionPolicyOrder();
    JointVector a, b, qa, qb;
    check(combined.infer(omega, gravity, command, q, JointVector::Zero(), previous_action,
          &a, &qa, &ms, &error), error.c_str());
    check(separated.observe(omega, gravity, command, q, JointVector::Zero(), previous_action,
          &error), error.c_str());
    const auto before = separated.observationHistory();
    check(separated.inferFromHistory(&b, &qb, &ms, &error), error.c_str());
    check((before - separated.observationHistory()).norm() == 0.f,
          "inference rolled HIM history a second time");
    max_him_error = std::max(max_him_error, (a - b).cwiseAbs().maxCoeff());
    previous_action = a;
  }
  check(max_him_error < 1e-6f, "separated HIM history changed inference");
  JointVector applied;
  for (int i = 0; i < 12; ++i) applied[i] = .01f * i;
  check((ActionToTargetQPolicyOrder(AppliedTargetToHimAction(applied)) -
         RobotToPolicyOrder(applied)).cwiseAbs().maxCoeff() < 1e-6f,
        "applied target/action conversion changed order or scale");

  RLPolicySwitch sw;
  using Phase = RLPolicySwitch::Phase;
  sw.reset(RLController::Flat, 1, 0);
  sw.tick(120000, true, false);
  check(sw.phase() == Phase::Preparing, "startup ignored history readiness");
  sw.tick(120001, true, true);
  check(sw.phase() == Phase::Running, "startup did not finish");
  sw.request(RLController::Stairs, 2, 200000, true);
  sw.tick(210000, true, true); sw.tick(400000, false, true);
  sw.tick(500000, true, true); sw.tick(699999, true, true);
  check(sw.active() == RLController::Flat && sw.phase() == Phase::Stopping, "ready before continuous 200 ms settling");
  sw.tick(700000, true, true);
  check(sw.active() == RLController::Flat && sw.phase() == Phase::Ready,
        "preparation changed policy owner before FSM onEnter");
  check(!sw.activate(RLController::Flat, 800000), "wrong destination activated");
  check(sw.activate(RLController::Stairs, 800000), "destination onEnter could not activate");
  check(sw.active() == RLController::Stairs && sw.phase() == Phase::Blending &&
        sw.blend(800000) == 0.f, "handover entry incorrect");
  sw.finishBlend(1299999, true);
  check(sw.phase() == Phase::Blending, "blend completed too soon");
  sw.finishBlend(1300000, false);
  check(sw.phase() == Phase::Blending, "blend ignored target rate limit");
  sw.finishBlend(1300001, true);
  check(sw.phase() == Phase::Running, "blend did not finish");
  sw.request(RLController::Flat, 3, 1400000, true);
  sw.tick(6400000, false, true);
  check(sw.phase() == Phase::Blocked && sw.zeroCommand(), "stop timeout resumed motion");
  sw.tick(7000000, true, true);
  check(sw.phase() == Phase::Blocked, "timeout retried without fresh button");
  sw.request(RLController::Flat, 4, 7100000, false);
  check(sw.phase() == Phase::Blocked, "unavailable model switched");
  sw.request(RLController::Stairs, 5, 7200000, true);
  check(sw.phase() == Phase::Running, "selecting active controller did not cancel pending switch");
  sw.request(RLController::Flat, 6, 7300000, true);
  sw.tick(7300000, true, true); sw.tick(7600000, true, true);
  check(sw.phase() == Phase::Ready && sw.active() == RLController::Stairs,
        "second handover bypassed FSM");
  check(sw.activate(RLController::Flat, 7600000), "second destination activation failed");
  sw.request(RLController::Stairs, 7, 7700000, true);
  check(sw.active() == RLController::Flat && sw.phase() == Phase::Blending,
        "queued request interrupted blend");
  sw.finishBlend(8100000, true);
  check(sw.active() == RLController::Flat && sw.phase() == Phase::Stopping,
        "queued request was not processed after blend");
  sw.tick(8100000, true, false); sw.tick(8500000, true, false);
  check(sw.phase() == Phase::Stopping, "handover ignored missing history");
  std::cout << "PASS: legacy observation max_error=" << max_observation_error
            << ", flat target max_error=" << max_flat_error
            << ", HIM action max_error=" << max_him_error
            << ", flat max_ms=" << flat_max_ms << '\n';
}
