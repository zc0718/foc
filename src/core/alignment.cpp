// Conan::ImportStart
#include <foc/core/alignment.hpp>

#include <cmath>
// Conan::ImportEnd

namespace foc {

namespace core {

namespace {

constexpr float kPi = 3.14159265358979323846F;
constexpr float kTwoPi = 6.28318530717958647692F;
constexpr float kHalfPi = 1.57079632679489661923F;

/**
 * @brief Wrap into [0, 2*pi).
 */
auto wrap_two_pi(float angle_rad) noexcept -> float {
    float wrapped = std::fmod(angle_rad, kTwoPi);
    if (wrapped < 0.0F) {
        wrapped += kTwoPi;
    }
    return wrapped;
}

/**
 * @brief Wrap into (-pi, pi].
 */
auto wrap_pi(float angle_rad) noexcept -> float { return wrap_two_pi(angle_rad + kPi) - kPi; }

} // namespace


/**
 * @brief [en] Implements AlignRoutine(AlignConfig); the contract is documented on its declaration.
 * @brief [zh] 实现 AlignRoutine(AlignConfig)；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
AlignRoutine::AlignRoutine(AlignConfig config) noexcept : config_(config) {}


/**
 * @brief [en] Implements AlignRoutine::set_config(); the contract is documented on its declaration.
 * @brief [zh] 实现 AlignRoutine::set_config()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
void AlignRoutine::set_config(AlignConfig config) noexcept { config_ = config; }


/**
 * @brief [en] Implements AlignRoutine::config(); the contract is documented on its declaration.
 * @brief [zh] 实现 AlignRoutine::config()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
auto AlignRoutine::config() const noexcept -> const AlignConfig& { return config_; }


/**
 * @brief [en] Implements AlignRoutine::validate(); the contract is documented on its declaration.
 * @brief [zh] 实现 AlignRoutine::validate()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
auto AlignRoutine::validate() const noexcept -> ErrorCode {
    // A default-constructed recipe is the documented "nobody asked for alignment" state. It is
    // deliberately distinct from a *broken* recipe: answering Param here would make every motor
    // that merely carries an aligner refuse to start.
    // 默认构造的配方就是文档化的“没人请求对齐”状态。它故意与**坏掉**的配方区分开：这里若答 Param，
    // 会让每个只不过带了个对齐器的电机都拒绝启动。
    const bool unset = (config_.current_a == 0.0F) && (config_.ramp_s == 0.0F) && (config_.hold_s == 0.0F) &&
                       (config_.settle_tolerance_rad == 0.0F) && (config_.verify_s == 0.0F) &&
                       (config_.align_angle_e_rad == 0.0F);
    if (unset) {
        return ErrorCode::NotImpl;
    }

    // Three fields must be positive because zero is not a meaningful value for them; the other
    // three accept zero as "not used". A recipe that leaves the first group at zero is a
    // configuration error -- substituting a plausible-looking constant would hide it until the
    // machine misbehaved on a real shaft.
    // 三个字段必须为正，因为 0 对它们没有意义；另外三个允许取 0 表示“不用”。第一组留成 0 属于配置
    // 错误——换成一个看起来合理的常数只会把它藏到真机出错的那天才暴露。
    if (!(config_.current_a > 0.0F)) {
        return ErrorCode::Param;
    }
    if (!(config_.hold_s > 0.0F)) {
        return ErrorCode::Param;
    }
    if (!(config_.settle_tolerance_rad > 0.0F)) {
        return ErrorCode::Param;
    }
    if (!(config_.ramp_s >= 0.0F)) {
        return ErrorCode::Param;
    }
    if (!(config_.verify_s >= 0.0F)) {
        return ErrorCode::Param;
    }
    if (!std::isfinite(config_.align_angle_e_rad)) {
        return ErrorCode::Param;
    }
    return ErrorCode::Ok;
}


/**
 * @brief [en] Implements AlignRoutine::begin(); the contract is documented on its declaration.
 * @brief [zh] 实现 AlignRoutine::begin()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
auto AlignRoutine::begin(std::uint32_t pole_pairs) noexcept -> bool {
    reset();
    const ErrorCode recipe = validate();
    if (recipe == ErrorCode::NotImpl) {
        return false; // nothing was requested: Idle, not Failed
    }
    if ((recipe != ErrorCode::Ok) || (pole_pairs == 0U)) {
        state_ = AlignState::Failed;
        failure_ = AlignFailure::NotConfigured;
        return false;
    }
    pole_pairs_ = pole_pairs;
    // A zero ramp is legal and means "apply the full current at once"; skipping straight to the
    // hold keeps that case from needing a special branch inside the ramp phase -- but the current
    // has to be raised there, because the ramp phase is the only other place that sets it.
    // 零爬升合法，含义是「一次给足电流」；直接进入保持阶段，省掉爬升里的特例分支——但电流必须在这里
    // 给足，因为爬升阶段是另一处设置它的地方。
    if (config_.ramp_s > 0.0F) {
        enter(AlignState::Ramp);
    } else {
        id_a_ = config_.current_a;
        enter(AlignState::Settle);
    }
    return true;
}


/**
 * @brief [en] Implements AlignRoutine::step(); the contract is documented on its declaration.
 * @brief [zh] 实现 AlignRoutine::step()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
auto AlignRoutine::step(float dt_s, float angle_mech_rad) noexcept -> AlignOutput {
    AlignOutput out{0.0F, config_.align_angle_e_rad, false, false};

    if ((state_ == AlignState::Idle) || (state_ == AlignState::Done)) {
        return out;
    }
    if (state_ == AlignState::Failed) {
        out.failed = true;
        return out;
    }
    // A non-positive step cannot advance a time-based sequence. The motor layer already rejects it,
    // so this is a guard, not a policy: hold the phase where it is rather than stretching it.
    // 非正步长无法推进基于时间的流程。电机层已经会拒绝它，所以这里只是护栏而非策略：停在原阶段，
    // 而不是把阶段撑长。
    if (!(dt_s > 0.0F)) {
        return out;
    }

    elapsed_s_ += dt_s;

    if (state_ == AlignState::Ramp) {
        const float fraction = (config_.ramp_s > 0.0F) ? (elapsed_s_ / config_.ramp_s) : 1.0F;
        if (fraction < 1.0F) {
            id_a_ = config_.current_a * fraction;
            out.id_a = id_a_;
            return out;
        }
        id_a_ = config_.current_a;
        enter(AlignState::Settle);
    }

    if (state_ == AlignState::Settle) {
        out.id_a = id_a_;
        accumulate(angle_mech_rad);
        if (elapsed_s_ >= config_.hold_s) {
            first_mean_rad_ = capture_mean();
            if (!(last_spread_rad_ <= config_.settle_tolerance_rad)) {
                // Reject as "did not settle" rather than accept the mean of a moving rotor: a mean
                // over a swing is a number, not an angle the rotor is actually at.
                // 以“未收敛”拒绝，而不是接受一个还在晃的转子的均值：在摆动上求平均得到的是一个数字，
                // 不是转子真正所在的角度。
                return fail(AlignFailure::NotSettled);
            }
            // Only an accepted capture is published: the verify window's spread measures travel, so
            // overwriting spread_rad_ with it would report a number that means something else.
            // 只公布被接受的捕获：校验窗口的离散度度量的是行程，用它覆盖 spread_rad_ 会报出一个含义
            // 不同的数字。
            spread_rad_ = last_spread_rad_;
            if (config_.verify_s > 0.0F) {
                enter(AlignState::Verify);
            } else {
                finish();
                out.id_a = 0.0F;
                out.finished = true;
            }
        }
        return out;
    }

    // Verify: the field advances a quarter electrical turn. A rotor that follows keeps the same
    // sign of movement; an inverted sensor reports the opposite sign, which is the one failure a
    // captured offset cannot repair by itself.
    // 校验：磁场前进四分之一电周期。跟随的转子位移符号与磁场一致；传感器符号接反会报出相反的符号，
    // 而这恰恰是捕获到的偏置本身修不好的那种错误。
    out.theta_e_rad = config_.align_angle_e_rad + kHalfPi;
    out.id_a = id_a_;
    accumulate(angle_mech_rad);
    if (elapsed_s_ >= config_.verify_s) {
        const float second_mean_rad = capture_mean();
        const float delta_rad = wrap_pi(second_mean_rad - first_mean_rad_);
        // No spread test here, and that is not an oversight: the rotor is *supposed* to traverse a
        // quarter pole pitch during this window, so the spread measures travel, not jitter. What
        // must be tested is whether it went anywhere at all, and in which direction.
        // 这里不查离散度，这不是疏漏：转子在该窗口内本来就该走过四分之一个极距，所以离散度度量的
        // 是行程而不是抖动。要检验的是它到底有没有动、朝哪个方向动。
        //
        // A quarter of the commanded travel is the derived acceptance threshold: it is the commanded
        // move divided by four, chosen to sit far above sensor noise and far below the full move, so
        // a sticky shaft still counts as having followed. Requiring the full quarter turn would
        // reject a shaft with ordinary friction; requiring merely "more than noise" would accept a
        // jammed one. A machine that cannot make even a quarter of the way needs more alignment
        // current, which is exactly the diagnosis `NotSettled` points at.
        // 验收门限取被指令行程的四分之一（由指令量导出）：远高于传感器噪声、又远低于整个行程，因此
        // 略有摩擦的轴仍算“跟上了”。要求走满四分之一圈会把正常摩擦的轴判死；只要求“大于噪声”又会让
        // 卡死的轴通过。连四分之一都走不到的机器需要加大对齐电流——这正是 NotSettled 指向的诊断。
        const float commanded_mech_rad = kHalfPi / static_cast<float>(pole_pairs_);
        if (!(std::fabs(delta_rad) >= (commanded_mech_rad / 4.0F))) {
            return fail(AlignFailure::NotSettled);
        }
        if (!(delta_rad > 0.0F)) {
            return fail(AlignFailure::DirectionWrong);
        }
        finish();
        out.id_a = 0.0F;
        out.theta_e_rad = config_.align_angle_e_rad;
        out.finished = true;
    }
    return out;
}


/**
 * @brief [en] Implements AlignRoutine::reset(); the contract is documented on its declaration.
 * @brief [zh] 实现 AlignRoutine::reset()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
void AlignRoutine::reset() noexcept {
    state_ = AlignState::Idle;
    failure_ = AlignFailure::None;
    pole_pairs_ = 0U;
    elapsed_s_ = 0.0F;
    id_a_ = 0.0F;
    offset_mech_rad_ = 0.0F;
    spread_rad_ = 0.0F;
    last_spread_rad_ = 0.0F;
    first_mean_rad_ = 0.0F;
    sum_sin_ = 0.0F;
    sum_cos_ = 0.0F;
    samples_ = 0U;
}


/**
 * @brief [en] Implements AlignRoutine::state(); the contract is documented on its declaration.
 * @brief [zh] 实现 AlignRoutine::state()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
auto AlignRoutine::state() const noexcept -> AlignState { return state_; }


/**
 * @brief [en] Implements AlignRoutine::failure(); the contract is documented on its declaration.
 * @brief [zh] 实现 AlignRoutine::failure()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
auto AlignRoutine::failure() const noexcept -> AlignFailure { return failure_; }


/**
 * @brief [en] Implements AlignRoutine::offset_mech_rad(); the contract is documented on its declaration.
 * @brief [zh] 实现 AlignRoutine::offset_mech_rad()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
auto AlignRoutine::offset_mech_rad() const noexcept -> float { return offset_mech_rad_; }


/**
 * @brief [en] Implements AlignRoutine::spread_rad(); the contract is documented on its declaration.
 * @brief [zh] 实现 AlignRoutine::spread_rad()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
auto AlignRoutine::spread_rad() const noexcept -> float { return spread_rad_; }


/**
 * @brief [en] Reject the capture and stop: zero current from this tick on.
 * @brief [zh] 拒绝捕获并停止：从本 tick 起电流归零。
 */
auto AlignRoutine::fail(AlignFailure reason) noexcept -> AlignOutput {
    state_ = AlignState::Failed;
    failure_ = reason;
    id_a_ = 0.0F;
    return AlignOutput{0.0F, config_.align_angle_e_rad, false, true};
}


/**
 * @brief [en] Accept the capture and derive the mechanical offset from it.
 * @brief [zh] 接受捕获并由它导出机械偏置。
 */
void AlignRoutine::finish() noexcept {
    const float forced_mech_rad = config_.align_angle_e_rad / static_cast<float>(pole_pairs_);
    offset_mech_rad_ = wrap_two_pi(first_mean_rad_ - forced_mech_rad);
    state_ = AlignState::Done;
    id_a_ = 0.0F;
}


/**
 * @brief [en] Switch phase and restart the phase timer plus the capture accumulator.
 * @brief [zh] 切换阶段，同时重启阶段计时与捕获累加器。
 */
void AlignRoutine::enter(AlignState next) noexcept {
    state_ = next;
    elapsed_s_ = 0.0F;
    sum_sin_ = 0.0F;
    sum_cos_ = 0.0F;
    samples_ = 0U;
}


/**
 * @brief [en] Fold one sensor reading into the circular accumulator.
 * @brief [zh] 把一个传感器读数并入圆统计累加器。
 * @note [en] Circular, not arithmetic: a sensor sitting at 6.28 rad and another at 0.01 rad are 0.01 rad apart, but
 *            their arithmetic mean is 3.14 -- a half turn of pure fiction. The sine/cosine sums give the correct mean
 *            and make the spread meaningful across the wrap.
 * @note [zh] 用圆统计而非算术平均：读数 6.28 rad 与 0.01 rad 相距 0.01 rad，但算术均值是 3.14—— 凭空多出半圈。正弦/余弦
 *            累加给出正确的均值，并让离散度跨越 2π 仍然有意义。
 */
void AlignRoutine::accumulate(float angle_mech_rad) noexcept {
    sum_sin_ += static_cast<double>(std::sin(angle_mech_rad));
    sum_cos_ += static_cast<double>(std::cos(angle_mech_rad));
    ++samples_;
}


/**
 * @brief [en] Mean of the accumulated readings, plus the spread statistics in spread_rad_.
 * @brief [zh] 累加读数的均值，并把离散度写入 spread_rad_。
 */
auto AlignRoutine::capture_mean() noexcept -> float {
    if (samples_ == 0U) {
        last_spread_rad_ = kPi;
        return 0.0F;
    }
    const float mean_rad = std::atan2(static_cast<float>(sum_sin_), static_cast<float>(sum_cos_));
    // Mean resultant length: 1 when every sample agrees, 0 when they cancel. Clamped because
    // rounding can push it a hair above 1, and log() would then go positive.
    // 平均合成长度：全部一致时为 1，互相抵消时为 0。舍入可能让它略超 1，而 log() 会因此变正，故钳位。
    const double count = static_cast<double>(samples_);
    const double resultant = std::sqrt((sum_sin_ * sum_sin_) + (sum_cos_ * sum_cos_)) / count;
    const double clamped = (resultant < 1.0) ? ((resultant > 1e-9) ? resultant : 1e-9) : 1.0;
    last_spread_rad_ = static_cast<float>(std::sqrt(-2.0 * std::log(clamped)));
    return wrap_two_pi(mean_rad);
}

} // namespace core

} // namespace foc
