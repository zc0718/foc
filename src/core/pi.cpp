// Conan::ImportStart
#include <foc/core/pi.hpp>

#include <foc/core/limits.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

namespace {


/**
 * @brief [en] Implements valid_config(); the contract is documented on its declaration.
 * @brief [zh] 实现 valid_config()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
constexpr auto valid_config(const Pi::Config& config) noexcept -> bool {
    return (config.kp >= 0.0F) && (config.ki >= 0.0F) && (config.out_min <= config.out_max) &&
           (config.integral_min <= config.integral_max);
}

} // namespace

Pi::Pi(Config config) noexcept
    : config_(config), integral_(0.0F), output_(0.0F),
      status_(valid_config(config) ? ErrorCode::Ok : ErrorCode::Param) {}

auto Pi::step(float setpoint, float feedback, float dt_s) noexcept -> float {
    return run(setpoint, feedback, dt_s, config_.kp, config_.integral_min, config_.integral_max);
}


/**
 * @brief [en] Implements step(const SpeedInput&); the contract is documented on its declaration.
 * @brief [zh] 实现 step(const SpeedInput&)；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
auto Pi::step(const SpeedInput& request, float feedback, float dt_s) noexcept -> float {
    // 0 means "no override". Anything else is a multiplier, and it is applied to the *configuration*
    // rather than accumulated into it, so a compliant call never leaves the controller detuned.
    // 0 表示“不覆盖”。其余取值是乘数，并且是作用于**配置**而不是累加进配置，因此一次柔顺调用绝不会
    // 把控制器留在失调状态。
    const float kp_scale = (request.kp_scale > 0.0F) ? request.kp_scale : 1.0F;
    const float ilimit_scale = (request.ilimit_scale > 0.0F) ? request.ilimit_scale : 1.0F;
    return run(request.target_rad_s, feedback, dt_s, config_.kp * kp_scale, config_.integral_min * ilimit_scale,
               config_.integral_max * ilimit_scale);
}


/**
 * @brief [en] The step itself, parameterised by the gains in force for this call.
 * @brief [zh] 步骤本体，其参数是本次调用生效的增益。
 * @since 0.1.0
 */
auto Pi::run(float setpoint, float feedback, float dt_s, float kp, float integral_min, float integral_max) noexcept
    -> float {
    if (!(dt_s > 0.0F)) {
        status_ = ErrorCode::Param;
        return output_;
    }

    if (status_ == ErrorCode::Param) {
        return output_;
    }

    const float error = setpoint - feedback;
    integral_ += static_cast<float>(config_.ki * error) * dt_s;
    // Re-clamped every step against the *current* window: when `ilimit_scale` shrinks it, the torque
    // already stored is released at once instead of leaking out over many steps.
    // 每一步都按**当前**窗口重新夹紧：`ilimit_scale` 把窗口收窄时，已存的转矩会立刻释放，而不是花
    // 很多步慢慢漏掉。
    integral_ = clamp(integral_, integral_min, integral_max);

    const float raw = static_cast<float>(kp * error) + integral_;
    output_ = clamp(raw, config_.out_min, config_.out_max);
    return output_;
}

void Pi::reset() noexcept {
    integral_ = 0.0F;
    output_ = 0.0F;
    status_ = valid_config(config_) ? ErrorCode::Ok : ErrorCode::Param;
}

auto Pi::integral() const noexcept -> float { return integral_; }

auto Pi::output() const noexcept -> float { return output_; }

auto Pi::status() const noexcept -> ErrorCode { return status_; }

} // namespace core

} // namespace foc
