// Conan::ImportStart
#include <foc/core/startup.hpp>

#include <cmath>
// Conan::ImportEnd

namespace foc {

namespace core {

namespace {

constexpr float kTwoPi = 6.28318530717958647692F;

auto wrap_two_pi(float angle) noexcept -> float {
    while (angle >= kTwoPi) {
        angle -= kTwoPi;
    }
    while (angle < 0.0F) {
        angle += kTwoPi;
    }
    return angle;
}

} // namespace


StartupRoutine::StartupRoutine(StartupConfig config) noexcept : config_(config) {}


void StartupRoutine::set_config(StartupConfig config) noexcept { config_ = config; }


auto StartupRoutine::validate() const noexcept -> ErrorCode {
    const bool untouched = (config_.align_current_a == 0.0F) && (config_.align_hold_s == 0.0F) &&
                           (config_.pull_current_a == 0.0F) && (config_.pull_accel_rad_s2 == 0.0F) &&
                           (config_.handover_speed_rad_s == 0.0F);
    if (untouched) {
        // No recipe was requested. `NotImpl` is the third answer and means "this build does not start this way", which
        // is what `Motor` needs in order to fall back to its plain Align -> Run behaviour.
        // 没有请求任何配方。`NotImpl` 是第三种答案，意为"这次构建不以这种方式起动"，而这正是 `Motor` 退回其原本
        // Align -> Run 行为所需要的信息。
        return ErrorCode::NotImpl;
    }
    const bool usable = std::isfinite(config_.align_current_a) && (config_.align_current_a > 0.0F) &&
                        std::isfinite(config_.align_hold_s) && (config_.align_hold_s > 0.0F) &&
                        std::isfinite(config_.pull_current_a) && (config_.pull_current_a > 0.0F) &&
                        std::isfinite(config_.pull_accel_rad_s2) && (config_.pull_accel_rad_s2 > 0.0F) &&
                        std::isfinite(config_.handover_speed_rad_s) && (config_.handover_speed_rad_s > 0.0F);
    return usable ? ErrorCode::Ok : ErrorCode::Param;
}


auto StartupRoutine::begin(std::uint32_t pole_pairs) noexcept -> bool {
    if (pole_pairs == 0U) {
        state_ = StartupState::Failed;
        return false;
    }
    if (validate() != ErrorCode::Ok) {
        state_ = StartupState::Failed;
        return false;
    }
    pole_pairs_ = pole_pairs;
    state_ = StartupState::Align;
    elapsed_s_ = 0.0F;
    theta_e_ = 0.0F;
    omega_e_ = 0.0F;
    offset_mech_ = 0.0F;
    return true;
}


auto StartupRoutine::step(float dt_s, float angle_mech_rad) noexcept -> AlignOutput {
    AlignOutput out{};
    if (!std::isfinite(dt_s) || !(dt_s > 0.0F)) {
        return out;
    }

    switch (state_) {
    case StartupState::Align:
        // The vector sits still in the stator frame and the rotor travels to it. That is the only part of the sequence
        // that has a known answer, which is why the offset is captured here and nowhere else.
        // 矢量在定子坐标系里静止，转子走过去。这是整条序列里唯一有已知答案的部分，所以偏置只在这里捕获。
        out.id_a = config_.align_current_a;
        out.theta_e_rad = theta_e_;
        elapsed_s_ += dt_s;
        if (elapsed_s_ >= config_.align_hold_s) {
            offset_mech_ = std::isfinite(angle_mech_rad) ? angle_mech_rad : 0.0F;
            elapsed_s_ = 0.0F;
            omega_e_ = 0.0F;
            state_ = StartupState::Pull;
        }
        return out;

    case StartupState::Pull: {
        // A rotating stator vector drags the rotor along at a slip the load sets. Nothing measures the rotor here; the
        // speed below is the vector's, not the machine's, and the difference is exactly what the hand-over has to absorb.
        // 旋转的定子矢量以负载决定的转差拖着转子。这里没有任何东西在测转子；下面的速度是矢量的，不是机器的，而这个差别
        // 正是交接时必须吸收的东西。
        omega_e_ += config_.pull_accel_rad_s2 * dt_s;
        theta_e_ = wrap_two_pi(theta_e_ + (omega_e_ * dt_s));
        out.id_a = config_.pull_current_a;
        out.theta_e_rad = theta_e_;
        if (omega_e_ >= config_.handover_speed_rad_s) {
            state_ = StartupState::Done;
            out.finished = true;
        }
        return out;
    }

    case StartupState::Done:
        // `Motor` has left the Align state by now; answering the last vector keeps the output well defined for a caller
        // that ticks once more.
        // `Motor` 此时已离开 Align 状态；仍回答最后一个矢量，使多 tick 一次的调用方也能拿到确定的值。
        out.id_a = config_.pull_current_a;
        out.theta_e_rad = theta_e_;
        return out;

    case StartupState::Failed:
    case StartupState::Idle:
    default:
        out.failed = (state_ == StartupState::Failed);
        return out;
    }
}


void StartupRoutine::reset() noexcept {
    state_ = StartupState::Idle;
    elapsed_s_ = 0.0F;
    theta_e_ = 0.0F;
    omega_e_ = 0.0F;
    offset_mech_ = 0.0F;
}


} // namespace core

} // namespace foc
