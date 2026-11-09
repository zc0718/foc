// Conan::ImportStart
#include <foc/core/polarity.hpp>

#include <cmath>

#include <foc/core/transforms.hpp>
// Conan::ImportEnd

/*!
 * @ingroup group_core
 * @file polarity.cpp
 *
 * @brief [en] Implements the polarity test: the pulse pair, the window that decides it, and the failure it reports.
 * @brief [zh] 极性测试的实现：脉冲对、做出判定的那个窗口，以及它报出的失败。
 * @since 0.1.0
 */

namespace foc {

namespace core {

namespace {
/** @cond */


/**
 * @brief [en] Implements valid_config(); the contract is documented on its declaration.
 * @brief [zh] 实现 valid_config()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
constexpr auto valid_config(const MagnetPolarityTest::Config& config) noexcept -> bool {
    return (config.pulse_v > 0.0F) && (config.pulse_s > 0.0F) && (config.settle_s >= 0.0F) &&
           (config.min_asymmetry_a >= 0.0F);
}

/** @endcond */
} // namespace

MagnetPolarityTest::MagnetPolarityTest(Config config) noexcept
    : config_(config), status_(valid_config(config) ? ErrorCode::Ok : ErrorCode::Param), stage_(Stage::SettlePositive),
      verdict_(PolarityVerdict::Inconclusive), theta_e_(0.0F), stage_clock_(), i_before_a_(0.0F),
      i_before_positive_a_(0.0F), delta_positive_a_(0.0F), delta_negative_a_(0.0F) {}

void MagnetPolarityTest::begin(float theta_e) noexcept {
    // Beginning again is how a caller retries after an Inconclusive answer, so it has to clear the
    // *conclusion* as well as the counters: keeping a stale verdict would turn "ask again" into
    // "you already have an answer".
    // 重新开始是调用方在 Inconclusive 之后补试的方式，因此它必须把 **结论** 也清掉而不只是计数器：
    // 留着旧结论会把“再问一次”变成“你已经有答案了”。
    stage_ = Stage::SettlePositive;
    verdict_ = PolarityVerdict::Inconclusive;
    theta_e_ = theta_e;
    stage_clock_.reset();
    i_before_a_ = 0.0F;
    i_before_positive_a_ = 0.0F;
    delta_positive_a_ = 0.0F;
    delta_negative_a_ = 0.0F;
    status_ = valid_config(config_) ? ErrorCode::Ok : ErrorCode::Param;
}

auto MagnetPolarityTest::step(float dt_s, Ab<float> i_ab) noexcept -> Output {
    if (!(dt_s > 0.0F)) {
        status_ = ErrorCode::Param;
        return output();
    }

    if ((status_ == ErrorCode::Param) || (stage_ == Stage::Done)) {
        return output();
    }

    // Project the measurement on the d axis the pulses are applied along: the pulse direction and the
    // measurement direction have to be the same one, otherwise the comparison is between two
    // quantities that are not the same physical amplitude.
    // 把测量投影到脉冲施加的那条 d 轴上：脉冲方向与测量方向必须是同一个，否则比较的是两个并非同一物理
    // 幅值的量。
    const float c = std::cos(theta_e_);
    const float s = std::sin(theta_e_);
    const float i_along = static_cast<float>((i_ab.alpha * c) + (i_ab.beta * s));

    stage_clock_.add_sample(dt_s);

    // Transitions happen on the sample that ends a stage, so the captured current is the one measured
    // at the notch between two stages and not a sample from the middle of a pulse.
    // 阶段切换发生在结束该阶段的那个采样上，于是被记下的电流是两阶段交界处的测量值，而不是脉冲中间的一个
    // 采样。
    switch (stage_) {
    case Stage::SettlePositive:
        if (stage_clock_.reached(config_.settle_s)) {
            i_before_a_ = i_along;
            i_before_positive_a_ = i_along;
            stage_ = Stage::PulsePositive;
            stage_clock_.reset();
        }
        break;
    case Stage::PulsePositive:
        if (stage_clock_.reached(config_.pulse_s)) {
            delta_positive_a_ = static_cast<float>(i_along - i_before_a_);
            stage_ = Stage::SettleNegative;
            stage_clock_.reset();
        }
        break;
    case Stage::SettleNegative:
        if (stage_clock_.reached(config_.settle_s)) {
            i_before_a_ = i_along;
            stage_ = Stage::PulseNegative;
            stage_clock_.reset();
        }
        break;
    case Stage::PulseNegative:
        if (stage_clock_.reached(config_.pulse_s)) {
            delta_negative_a_ = static_cast<float>(i_along - i_before_a_);
            decide();
            stage_ = Stage::Done;
            stage_clock_.reset();
        }
        break;
    case Stage::Done:
    default:
        break;
    }

    return output();
}

auto MagnetPolarityTest::decide() noexcept -> void {
    // The magnet's own flux does the deciding: a current that reinforces it saturates the iron and the
    // incremental inductance falls, so the same volt-second pulse rises further. Small differences are
    // not promoted to an answer -- a machine without saturation has no polarity information at all,
    // and reporting an arbitrary direction for it would be worse than reporting none.
    // 做判断的是磁钢自身的磁链：增强它的电流使铁心饱和、增量电感下降，于是相同伏秒的脉冲升得更多。
    // 差异过小时不升级为结论——没有饱和的电机根本没有极性信息，给它报一个随意方向比不报更糟。
    //
    // The comparison is between magnitudes, and it has to be: the negative pulse drives the current
    // negative, so its signed change is negative by construction and a signed comparison would answer
    // "Aligned" for every machine, including the ones with no asymmetry at all.
    // 比较的是 **幅值** ，而且必须如此：负脉冲把电流推向负方向，其带符号变化按构造就是负的，用带符号比较会
    // 对每一台电机都回答“Aligned”，包括根本没有不对称的那些。
    const float positive = std::fabs(delta_positive_a_);
    const float negative = std::fabs(delta_negative_a_);
    const float asymmetry = static_cast<float>(positive - negative);
    if (std::fabs(asymmetry) < config_.min_asymmetry_a) {
        verdict_ = PolarityVerdict::Inconclusive;
        return;
    }

    // The comparison is only meaningful if both pulses started from the same operating point: the
    // method's premise is "the *same* volt-second pulse in two directions". A settle shorter than the
    // current's decay leaves the second pulse starting from the first pulse's tail, where the
    // inductance is already collapsed -- the second rise is then larger for a reason that has nothing
    // to do with the magnet, and the answer comes out confidently **wrong**. The yardstick is the
    // caller's own declared resolution: if the two start more than one answer-resolution apart, they
    // are not the same pulse and no verdict at that resolution can be supported.
    // 只有在两个脉冲从同一工作点出发时比较才有意义：这个方法的前提是“ **相同** 伏秒的脉冲在两个
    // 方向上”。松弛时间短于电流衰减时间时，第二个脉冲从第一个脉冲的尾巴上出发，那里的电感已经塌
    // 陷——于是第二个上升更大，原因与磁钢无关，而答案会 **自信地错** 。判据用调用方自己声明的分辨
    // 力：两个起点相差超过一个答案分辨力时，它们就不是同一个脉冲，那个分辨力下的任何结论都不成立。
    const float start_spread = std::fabs(i_before_positive_a_ - i_before_a_);
    if (start_spread >= config_.min_asymmetry_a) {
        verdict_ = PolarityVerdict::Inconclusive;
        return;
    }
    verdict_ = (asymmetry > 0.0F) ? PolarityVerdict::Aligned : PolarityVerdict::Reversed;
}

auto MagnetPolarityTest::output() const noexcept -> Output {
    Ab<float> v{0.0F, 0.0F};
    if (status_ == ErrorCode::Ok) {
        float amplitude = 0.0F;
        if (stage_ == Stage::PulsePositive) {
            amplitude = config_.pulse_v;
        } else if (stage_ == Stage::PulseNegative) {
            amplitude = -config_.pulse_v;
        }
        if (amplitude != 0.0F) {
            v = Ab<float>{static_cast<float>(amplitude * std::cos(theta_e_)),
                          static_cast<float>(amplitude * std::sin(theta_e_))};
        }
    }
    return Output{verdict_, v, stage_ != Stage::Done, delta_positive_a_, delta_negative_a_};
}

auto MagnetPolarityTest::status() const noexcept -> ErrorCode { return status_; }

auto MagnetPolarityTest::verdict() const noexcept -> PolarityVerdict { return verdict_; }

} // namespace core

} // namespace foc
