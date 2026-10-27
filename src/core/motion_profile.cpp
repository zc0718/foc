// Conan::ImportStart
#include <foc/core/motion_profile.hpp>

#include <algorithm>
#include <cmath>
// Conan::ImportEnd

namespace foc {

namespace core {

namespace {


/**
 * @brief [en] Whether the configuration describes limits the controller can honour.
 *        [zh] 该配置是否描述了控制器能遵守的限值。
 * @attacher
 * @since 0.1.0
 */
constexpr auto valid_config(const MotionProfile::Config& config) noexcept -> bool {
    return (config.shaping_gain_rad_s > 0.0F) && (config.velocity_limit_rad_s > 0.0F) &&
           (config.accel_limit_rad_s2 > 0.0F);
}

} // namespace

MotionProfile::MotionProfile(const Config& config) noexcept : config_(config) {}


/**
 * @brief [en] Implements step(); the contract is documented on its declaration.
 *        [zh] 实现 step()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
auto MotionProfile::step(float setpoint, float feedback, float dt_s) noexcept -> float {
    return shape(setpoint, feedback, dt_s, config_.velocity_limit_rad_s, config_.accel_limit_rad_s2,
                 config_.shaping_gain_rad_s);
}


/**
 * @brief [en] Implements step(const PositionInput&); the contract is documented on its declaration.
 *        [zh] 实现 step(const PositionInput&)；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
auto MotionProfile::step(const PositionInput& request, float feedback, float dt_s) noexcept -> float {
    // 0 means "no override", the project-wide rule for per-call values: it is only ever a default
    // where 0 is not a legal limit anyway. A negative limit is clamped to 0 (i.e. ignored) instead
    // of being read as "go the other way".
    // 0 表示“不覆盖”，这是本项目对按次传值的统一规则：只有在 0 本身不构成合法限值的地方它才是默认。
    // 负的限制按 0 处理（即忽略），而不是被理解成“反向运动”。
    const float velocity_limit =
        (request.velocity_limit_rad_s > 0.0F) ? request.velocity_limit_rad_s : config_.velocity_limit_rad_s;
    const float accel_limit =
        (request.accel_limit_rad_s2 > 0.0F) ? request.accel_limit_rad_s2 : config_.accel_limit_rad_s2;
    const float shaping_gain = config_.shaping_gain_rad_s * ((request.kp_scale > 0.0F) ? request.kp_scale : 1.0F);
    return shape(request.target_rad, feedback, dt_s, velocity_limit, accel_limit, shaping_gain);
}


/**
 * @brief [en] The shaping law itself, parameterised by the limits in force for this call.
 *        [zh] 塑造律本体，其参数是本次调用生效的限值。
 * @since 0.1.0
 */
auto MotionProfile::shape(float setpoint, float feedback, float dt_s, float velocity_limit_rad_s,
                          float accel_limit_rad_s2, float shaping_gain_rad_s) noexcept -> float {
    const Config effective{shaping_gain_rad_s, velocity_limit_rad_s, accel_limit_rad_s2};
    if (!(dt_s > 0.0F) || !std::isfinite(setpoint) || !std::isfinite(feedback) || !valid_config(effective)) {
        // Hold what we had: refusing to move is the only safe answer to an input we cannot use.
        // 保持原样：对无法使用的输入，“不动”是唯一安全的答案。
        return valid_config(effective) ? velocity_rad_s_ : 0.0F;
    }

    // Three ceilings on the demanded speed, and the smallest wins:
    //   1. the proportional term          gain * |error|      -- chase the error
    //   2. the physical braking curve     sqrt(2*a*|error|)   -- be able to stop in what is left
    //   3. the configured velocity limit  v_limit
    // Without (2) a velocity-limited move sails past the target by the braking distance: the P term
    // only starts easing off once the error is already small.
    // 需求速度上有三个上限，取最小：比例项、物理制动曲线、速度上限。少了第二条，速度限幅生效的运动
    // 会冲过目标整整一个制动距离——因为比例项要等误差已经很小才开始回落。
    const float two = 2.0F;
    const float error = setpoint - feedback;
    const float magnitude_error = std::fabs(error);
    const float proportional = effective.shaping_gain_rad_s * magnitude_error;
    const float braking = std::sqrt(two * effective.accel_limit_rad_s2 * magnitude_error);
    float magnitude = std::min(proportional, braking);
    magnitude = std::min(magnitude, effective.velocity_limit_rad_s);
    const float demanded = (error >= 0.0F) ? magnitude : -magnitude;

    const float max_change = effective.accel_limit_rad_s2 * dt_s;

    if (demanded > (velocity_rad_s_ + max_change)) {
        velocity_rad_s_ += max_change;
    } else if (demanded < (velocity_rad_s_ - max_change)) {
        velocity_rad_s_ -= max_change;
    } else {
        velocity_rad_s_ = demanded;
    }

    return velocity_rad_s_;
}


/**
 * @brief [en] Implements reset(); the contract is documented on its declaration.
 *        [zh] 实现 reset()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
void MotionProfile::reset() noexcept { velocity_rad_s_ = 0.0F; }


/**
 * @brief [en] Implements config(); the contract is documented on its declaration.
 *        [zh] 实现 config()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
auto MotionProfile::config() const noexcept -> const Config& { return config_; }


/**
 * @brief [en] Implements velocity_command_rad_s(); the contract is documented on its declaration.
 *        [zh] 实现 velocity_command_rad_s()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
auto MotionProfile::velocity_command_rad_s() const noexcept -> float { return velocity_rad_s_; }

} // namespace core

} // namespace foc
