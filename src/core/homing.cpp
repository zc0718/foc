// Conan::ImportStart
#include <foc/core/homing.hpp>
// Conan::ImportEnd

#include <cmath>

namespace foc {

namespace core {

namespace {

/**
 * @brief [en] Whether a recipe describes a search that can be run.
 * @brief [zh] 这个配方描述的是否是一次可执行的搜索。
 * @note [en] A free function rather than a member call from the constructor: `validate()` needs `config_`, and calling
 *            a member function on a half-constructed object is the kind of thing that works until someone reorders the
 *            member list.
 * @note [zh] 用自由函数而不是在构造函数里调用成员：`validate()` 需要 `config_`，而在半构造的对象上调用成员函数，属于那
 *            种"直到有人重排了成员列表才出问题"的做法。
 */
auto usable(const HomingConfig& config) noexcept -> bool {
    const bool all_zero = (config.creep_speed_rad_s == 0.0F) && (config.timeout_s == 0.0F) && (config.direction == 0);
    return !all_zero && std::isfinite(config.creep_speed_rad_s) && (config.creep_speed_rad_s > 0.0F) &&
           std::isfinite(config.timeout_s) && (config.timeout_s > 0.0F) &&
           ((config.direction == 1) || (config.direction == -1));
}

} // namespace

HomingRoutine::HomingRoutine(HomingConfig config) noexcept
    : config_(config), state_(usable(config) ? HomingState::Idle : HomingState::Failed),
      failure_(usable(config) ? HomingFailure::None : HomingFailure::NotConfigured), elapsed_s_(0.0F) {}

auto HomingRoutine::validate() const noexcept -> ErrorCode {
    const bool all_zero =
        (config_.creep_speed_rad_s == 0.0F) && (config_.timeout_s == 0.0F) && (config_.direction == 0);
    if (all_zero) {
        // The documented way of saying "no homing was asked for", matching `StartupRoutine`: a zero recipe is not a
        // malformed one, it is the absence of a request, and answering `Param` would blame the caller for something
        // they did not ask for.
        // "没有要求回零"的书面写法，与 `StartupRoutine` 一致：全零配方不是写错了，而是没有请求；答 `Param` 等于为
        // 一件调用方根本没要求的事责怪它。
        return ErrorCode::NotImpl;
    }
    if (!std::isfinite(config_.creep_speed_rad_s) || !(config_.creep_speed_rad_s > 0.0F) ||
        !std::isfinite(config_.timeout_s) || !(config_.timeout_s > 0.0F) ||
        ((config_.direction != 1) && (config_.direction != -1))) {
        return ErrorCode::Param;
    }
    return ErrorCode::Ok;
}

void HomingRoutine::reset() noexcept {
    state_ = usable(config_) ? HomingState::Idle : HomingState::Failed;
    failure_ = usable(config_) ? HomingFailure::None : HomingFailure::NotConfigured;
    elapsed_s_ = 0.0F;
}

auto HomingRoutine::begin(const PositionReference& reference) noexcept -> bool {
    reset();
    if (state_ == HomingState::Failed) {
        return true; // a rejected recipe is already over
    }
    if (reference.homed()) {
        state_ = HomingState::Done;
        return true;
    }
    state_ = HomingState::Creep;
    return false;
}

auto HomingRoutine::step(const PositionReference& reference, float dt_s) noexcept -> Output {
    Output out{};
    out.state = state_;
    out.finished = (state_ == HomingState::Done) || (state_ == HomingState::Failed);
    if (!(dt_s > 0.0F) || out.finished || (state_ == HomingState::Idle)) {
        return out;
    }

    elapsed_s_ += dt_s;
    if (reference.homed()) {
        state_ = HomingState::Done;
        out.speed_ref_rad_s = 0.0F;
        out.state = state_;
        out.finished = true;
        return out;
    }
    if (elapsed_s_ >= config_.timeout_s) {
        // Giving up rather than turning forever: an axis that never sees its mark is a machine with a broken sensor or a
        // mark out of reach, and both need a human rather than more current.
        // 放弃而不是一直转：一个永远见不到标记的轴，要么传感器坏了，要么标记在行程之外，两者都需要人，而不是更大的电流。
        state_ = HomingState::Failed;
        failure_ = HomingFailure::TimedOut;
        out.speed_ref_rad_s = 0.0F;
        out.state = state_;
        out.finished = true;
        return out;
    }

    out.speed_ref_rad_s = static_cast<float>(config_.direction) * config_.creep_speed_rad_s;
    return out;
}

} // namespace core

} // namespace foc
