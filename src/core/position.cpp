// Conan::ImportStart
#include <foc/core/position.hpp>

#include <algorithm>
#include <cmath>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] Implements PositionP(const Config&); the contract is documented on its declaration.
 *        [zh] 实现 PositionP(const Config&)；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
PositionP::PositionP(const Config& config) noexcept : config_(config) {}


/**
 * @brief [en] Implements PositionP::step(); the contract is documented on its declaration.
 *        [zh] 实现 PositionP::step()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
auto PositionP::step(const PositionInput& request, float feedback, float dt_s) noexcept -> float {
    // 0 in a per-move field means "no override"; the configured value then applies. Negative
    // per-move limits are the caller's bug and are read as 0 rather than as "go the other way".
    // 按次字段里的 0 表示"不覆盖"，此时用配置里的值。负的按次限制是调用方的 bug，按 0 处理，
    // 而不是理解成"反向运动"。
    const float velocity_limit =
        (request.velocity_limit_rad_s > 0.0F) ? request.velocity_limit_rad_s : config_.velocity_limit_rad_s;
    const float accel_limit =
        (request.accel_limit_rad_s2 > 0.0F) ? request.accel_limit_rad_s2 : config_.accel_limit_rad_s2;

    if (!(dt_s > 0.0F) || !std::isfinite(request.target_rad) || !std::isfinite(feedback) ||
        !(config_.gain_1_s > 0.0F) || !(velocity_limit > 0.0F)) {
        // An unusable input holds the previous command: refusing to move is the safe answer, and a
        // silent jump to zero would be a step change the mechanism did not ask for.
        // 无法使用的输入保持上一条指令：不动是安全的答案，而静默归零会是机构并没有要求的阶跃。
        return velocity_rad_s_;
    }

    const float kp_scale = (request.kp_scale > 0.0F) ? request.kp_scale : 1.0F;
    const float error = request.target_rad - feedback;
    const float demanded = std::clamp(config_.gain_1_s * kp_scale * error, -velocity_limit, velocity_limit);

    if (accel_limit > 0.0F) {
        const float max_change = accel_limit * dt_s;
        if (demanded > (velocity_rad_s_ + max_change)) {
            velocity_rad_s_ += max_change;
        } else if (demanded < (velocity_rad_s_ - max_change)) {
            velocity_rad_s_ -= max_change;
        } else {
            velocity_rad_s_ = demanded;
        }
    } else {
        velocity_rad_s_ = demanded;
    }
    return velocity_rad_s_;
}


/**
 * @brief [en] Implements PositionP::reset(); the contract is documented on its declaration.
 *        [zh] 实现 PositionP::reset()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
void PositionP::reset() noexcept { velocity_rad_s_ = 0.0F; }


/**
 * @brief [en] Implements PositionP::config(); the contract is documented on its declaration.
 *        [zh] 实现 PositionP::config()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
auto PositionP::config() const noexcept -> const Config& { return config_; }

} // namespace core

} // namespace foc
