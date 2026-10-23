// Conan::ImportStart
#include <foc/core/pi.hpp>

#include <foc/core/limits.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

namespace {


/**
 * @brief [en] Implements valid_config(); the contract is documented on its declaration.
 *        [zh] 实现 valid_config()；契约文档在对应的声明处。
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
    if (!(dt_s > 0.0F)) {
        status_ = ErrorCode::Param;
        return output_;
    }

    if (status_ == ErrorCode::Param) {
        return output_;
    }

    const float error = setpoint - feedback;
    integral_ += static_cast<float>(config_.ki * error) * dt_s;
    integral_ = clamp(integral_, config_.integral_min, config_.integral_max);

    const float raw = static_cast<float>(config_.kp * error) + integral_;
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
