#include "FlatPolicyRunner.h"

#include <chrono>
#include <stdexcept>

namespace rapid_rl {
namespace {
// Validate before calling the legacy loader, which otherwise accepts truncated
// files and leaves weights uninitialized. Legacy weights are a single CSV line.
void validateWeights(const std::string& path, const std::vector<int>& layers) {
  std::ifstream file(path);
  if (!file) throw std::runtime_error("cannot open " + path);
  std::string line, cell;
  if (!std::getline(file, line)) throw std::runtime_error("empty " + path);
  file >> std::ws;
  if (!file.eof()) throw std::runtime_error("expected single CSV line: " + path);
  size_t expected = 0, count = 0;
  for (size_t i = 1; i < layers.size(); ++i)
    expected += (layers[i - 1] + 1) * layers[i];
  std::istringstream csv(line);
  while (std::getline(csv, cell, ',')) {
    size_t used = 0;
    const float value = static_cast<float>(std::stod(cell, &used));
    if (!std::isfinite(value) || cell.find_first_not_of(" \t\r", used) != std::string::npos)
      throw std::runtime_error("invalid weight: " + path);
    ++count;
  }
  if (count != expected) throw std::runtime_error("wrong weight count: " + path);
}

void readNormalization(const std::string& path, FlatObservation* values, bool variance) {
  std::ifstream file(path);
  if (!file) throw std::runtime_error("cannot open " + path);
  for (int i = 0; i < 141; ++i) {
    float value;
    if (!(file >> value) || !std::isfinite(value) || (variance && value < 0))
      throw std::runtime_error("invalid normalization: " + path);
    (*values)[i] = value;
  }
  file >> std::ws;
  if (!file.eof()) throw std::runtime_error("wrong normalization count: " + path);
}
}  // namespace

bool FlatPolicyRunner::load(const std::string& directory) {
  ready_ = false;
  error_.clear();
  try {
    const std::string base = directory + "/";
    validateWeights(base + "actor_5000.txt", {152, 512, 256, 64, 12});
    validateWeights(base + "estimator_5000.txt", {141, 256, 128, 11});
    readNormalization(base + "mean5000.csv", &mean_, false);
    readNormalization(base + "var5000.csv", &variance_, true);
    actor_.updateParamFromTxt(base + "actor_5000.txt");
    estimator_.updateParamFromTxt(base + "estimator_5000.txt");
    ready_ = true;
    JointVector target;
    float ms;
    if (!infer(FlatObservation::Zero(), &target, &ms, &error_)) ready_ = false;
  } catch (const std::exception& e) {
    error_ = e.what();
    ready_ = false;
  }
  return ready_;
}

bool FlatPolicyRunner::infer(const FlatObservation& raw, JointVector* target,
                            float* elapsed_ms, std::string* error) {
  if (error) error->clear();
  if (!ready_ || !target || !raw.allFinite()) {
    if (error) *error = "flat policy not ready or invalid input/output";
    return false;
  }
  const auto start = std::chrono::steady_clock::now();
  FlatObservation obs = ((raw - mean_).array() / (variance_.array() + 1e-8f).sqrt())
      .max(-10.f).min(10.f).matrix();
  if (!obs.allFinite()) {
    if (error) *error = "non-finite normalized flat observation";
    return false;
  }
  const Eigen::Matrix<float, 11, 1> estimate = estimator_.forward(obs);
  Eigen::Matrix<float, 152, 1> actor_obs;
  actor_obs << obs, estimate;
  *target = actor_.forward(actor_obs) * .1f + defaultQ();
  if (elapsed_ms) *elapsed_ms = std::chrono::duration<float, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  if (!estimate.allFinite() || !target->allFinite()) {
    if (error) *error = "non-finite flat estimate or target";
    return false;
  }
  return true;
}
}  // namespace rapid_rl
