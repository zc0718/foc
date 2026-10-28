// Conan::ImportStart
#include <foc/core/position_reference.hpp>
// Conan::ImportEnd

#include <cmath>

namespace foc {

namespace core {

namespace {

constexpr float kPi = 3.14159265358979F;
constexpr float kTwoPi = 6.28318530717959F;

/**
 * @brief [en] Wrap an angle into (-pi, pi].
 * @brief [zh] 把角度折到 (-pi, pi]。
 */
auto wrap_pi(float angle) noexcept -> float {
    while (angle <= -kPi) {
        angle += kTwoPi;
    }
    while (angle > kPi) {
        angle -= kTwoPi;
    }
    return angle;
}

} // namespace

PositionReference::PositionReference(Config config) noexcept
    : config_(config), index_offset_rad_(0.0F), previous_theta_e_rad_(0.0F), unwrapped_mech_rad_(0.0F),
      referenced_mech_rad_(0.0F), primed_(false), homed_(false), lost_count_(false) {
    if (config_.pole_pairs == 0U) {
        config_.pole_pairs = 1U;
    }
}

void PositionReference::reset() noexcept {
    index_offset_rad_ = 0.0F;
    previous_theta_e_rad_ = 0.0F;
    unwrapped_mech_rad_ = 0.0F;
    referenced_mech_rad_ = 0.0F;
    primed_ = false;
    homed_ = false;
    lost_count_ = false;
}

auto PositionReference::update(float theta_e_rad, bool index_pulse) noexcept -> bool {
    const float poles = static_cast<float>(config_.pole_pairs);

    if (!primed_) {
        previous_theta_e_rad_ = theta_e_rad;
        unwrapped_mech_rad_ = theta_e_rad / poles;
        primed_ = true;
    } else {
        const float delta_e = wrap_pi(theta_e_rad - previous_theta_e_rad_);
        previous_theta_e_rad_ = theta_e_rad;

        // A step at or beyond half an electrical turn cannot be distinguished from a wrap, so it is reported instead of
        // guessed at: picking a direction here would produce an angle that looks fine and is a whole turn out.
        // 半个电角度以上的跳变无法与一次折返区分，因此**报告**而不是猜：在这里选一个方向会给出一个看起来没问题、却差了
        // 整整一圈的角度。
        if (std::fabs(delta_e) >= (0.9F * kPi)) {
            lost_count_ = true;
        }
        unwrapped_mech_rad_ += delta_e / poles;
    }

    // Bring the referenced angle up to date *before* the pulse is handled, so a repeat pass snaps from this tick's
    // value rather than from the previous one -- otherwise the correction carries one tick of travel as an error.
    // 在处理脉冲**之前**先把基准角更新到本 tick，于是重复经过是从本 tick 的值吸附、而不是从上一 tick，否则这次修正会
    // 把一 tick 的行程当成误差带进来。
    if (homed_) {
        referenced_mech_rad_ = unwrapped_mech_rad_ - index_offset_rad_;
    }
    if (!index_pulse) {
        return false;
    }

    if (!homed_) {
        // First index: the only moment where the origin is *defined* rather than corrected.
        // 第一次索引：原点被**定义**而不是被修正的唯一时刻。
        index_offset_rad_ = unwrapped_mech_rad_ - config_.index_mech_rad;
        homed_ = true;
    } else {
        // A later pass: snap to the nearest index position instead of redefining zero. Redefining would subtract a whole
        // turn and make the reported angle go backwards; on a repeat pass the mark's job is to remove accumulated drift.
        // 之后的经过：吸附到**最近的**索引位置，而不是重新定义零点。重新定义会减掉整整一圈、让报出的角度倒退；重复经过
        // 时标记的职责是消除累积漂移。
        index_offset_rad_ += wrap_pi(referenced_mech_rad_ - config_.index_mech_rad);
    }
    // Recomputed rather than set to `index_mech_rad`: the mark says *which turn* this is, and the residual above already
    // moved the origin onto it. Assigning the bare index position would rewind the count by a whole turn on every pass,
    // which is the difference this case was written to catch -- 6.26 rad reading back as 0.0.
    // 重新计算而不是赋成 `index_mech_rad`：标记说明的是**第几圈**，上面的残差已经把原点移到了它上面。直接赋索引位置
    // 会在每次经过时把计数倒退整整一圈——这正是本用例要抓的区别：6.26 rad 读回成 0.0。
    referenced_mech_rad_ = unwrapped_mech_rad_ - index_offset_rad_;

    // The mark is precisely the information that was lost, so it restores the count as well.
    // 标记正是丢失掉的那个信息，因此它也恢复了计数。
    lost_count_ = false;
    return true;
}

} // namespace core

} // namespace foc
