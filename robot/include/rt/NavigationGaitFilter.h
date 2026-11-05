#pragma once

// Serialized by the RC mutex. Monotonic seconds are injected for offline tests.
class NavigationGaitFilter {
 public:
  enum class Result { Confirming, Ready, CoolingDown, Switching };
  static constexpr int kConfirmations = 5;
  static constexpr double kCooldownSeconds = 2.;
  static constexpr double kMaxMessageGapSeconds = 1.;

  void resetConfirmation() { candidate_ = -1; count_ = 0; }
  void requestSubmitted() { pending_ = true; resetConfirmation(); }
  void manualSelection() { pending_ = false; resetConfirmation(); }
  void handover(bool switching, bool completed, double now) {
    pending_ = false;
    switching_ = switching;
    resetConfirmation();
    if (completed) cooldown_until_ = now + kCooldownSeconds;
  }
  Result receive(int target, double now) {
    if (switching_ || pending_) { resetConfirmation(); return Result::Switching; }
    if (remainingCooldown(now) > 0.) {
      resetConfirmation();
      return Result::CoolingDown;
    }
    if (target != candidate_ || now - last_message_ >= kMaxMessageGapSeconds)
      resetConfirmation();
    candidate_ = target;
    last_message_ = now;
    if (count_ < kConfirmations) ++count_;
    return count_ == kConfirmations ? Result::Ready : Result::Confirming;
  }
  int confirmations() const { return count_; }
  bool switching() const { return switching_ || pending_; }
  double remainingCooldown(double now) const {
    return cooldown_until_ > now ? cooldown_until_ - now : 0.;
  }

 private:
  int candidate_ = -1, count_ = 0;
  bool switching_ = false, pending_ = false;
  double last_message_ = 0., cooldown_until_ = 0.;
};
