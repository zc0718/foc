// Conan::ImportStart
#pragma once
#include <cmath>
#include <cstdint>
#include <foc/core/hall.hpp>
#include <foc/core/transforms.hpp>
#include <foc/core/types.hpp>
// Conan::ImportEnd

/*!
 * @ingroup group_core
 * @file six_step.hpp
 *
 * @brief [en] What a six-step commutation needs to know, and what it decides each period.
 * @brief [zh] 六步换相需要知道什么，以及它每个周期决定什么。
 * @since 0.1.0
 */

namespace foc {

namespace core {

/**
 * @brief [en] What a six-step commutation needs to know.
 * @brief [zh] 六步换相需要知道的参数。
 * @since 0.1.0
 * @exporter
 */
struct SixStepConfig final {
    /**
     * @brief [en] vector amplitude, 0 < a <= 1.
     * @brief [zh] 矢量幅值，0 < a <= 1。
     */
    float amplitude;
    /**
     * @brief [en] which way the table is pushed.
     * @brief [zh] 朝哪边推表。
     */
    HallDirection direction;
    /**
   * @brief [en] How many sectors one period may move, 1..5.
   * @brief [zh] 一个周期允许跨过的扇区数，1..5。
   * @note [en] One is what a real rotor does between two control periods; more than one means the read was missed, and
   *            the table would happily answer with a vector that is a whole sector or more away from where the rotor
   *            is. The rule here is to *keep the old vector and say so*, because a wrong vector produces torque in the
   *            wrong direction and a held one produces the torque it produced last period.
   * @note [zh] 真实转子在两个控制周期之间只会跨一个；跨更多意味着那一次读取被漏掉了，而表会照样给出一个与转子相隔一整格
   *            甚至更多的矢量。这里的规矩是 **保持旧矢量并说出来** ：错误矢量产生的力矩方向是错的，而保持的那个产生的还
   *            是上一周期的力矩。
   */
    std::uint8_t max_step_sectors;
};

/**
 * @brief [en] What one period of six-step commutation produced.
 * @brief [zh] 六步换相在一个控制周期内产出的东西。
 * @since 0.1.0
 * @exporter
 */
struct SixStepOutput final {
    /**
     * @brief [en] Ok, or Param for an unusable configuration.
     * @brief [zh] Ok；配置不可用时 Param。
     */
    ErrorCode status;
    /**
     * @brief [en] the three duties to apply.
     * @brief [zh] 要施加的三个占空比。
     */
    Uvw<float> duty;
    /**
     * @brief [en] the sector the returned vector belongs to.
     * @brief [zh] 返回的矢量所属扇区。
     */
    std::uint8_t sector;
    /**
     * @brief [en] a new sector was accepted this period.
     * @brief [zh] 本周期接受了一个新扇区。
     */
    bool changed;
    /**
     * @brief [en] an illegal code: the vector is the zero vector.
     * @brief [zh] 编码非法：矢量是零矢量。
     */
    bool holding;
    /**
     * @brief [en] the code named a sector too far away; the old vector is held.
     * @brief [zh] 编码指向太远的扇区，保持旧矢量。
     */
    bool skipped;
};

/**
 * @brief [en] Drive a six-step table from a hall code, and refuse the transitions a real rotor cannot make.
 *
 *   The table and the sector map already live in `hall.hpp`; what is added here is the *sequencing* rule that neither
 *   of them can state. A three-hall set names one of six sectors, and the table answers a vector for whatever it is
 *   told -- including a sector two places away, which is what a missed read looks like and which produces torque in the
 *   wrong direction. So a transition is accepted only when it is a neighbour, and a jump is held and counted instead.
 *   An illegal code (`0` or `7`) is a different thing: the pins are not naming a position at all, and the answer there
 *   is the zero vector plus `holding`.
 *
 * @brief [zh] 由霍尔编码驱动六步表，并拒绝真实转子做不出来的跳变。
 *
 *   表和扇区映射都已经在 `hall.hpp` 里；这里加的是它们都无法陈述的 **序列** 规则。三霍尔组只指明六个扇区之一，而表对它
 *   被告知的任何东西都给一个矢量——包括隔两格的扇区，那正是"漏读一次"的样子，而它产生的力矩方向是错的。因此只有相邻扇区
 *   被接受，跳变则保持旧矢量并计数。非法编码（ `0` 或 `7`）是另一回事：引脚根本没有在指出位置，那里的回答是零矢量加
 *   `holding`。
 * @since 0.1.0
 * @exporter
 */
class SixStepCommutation final {
  public:
    SixStepCommutation() noexcept = default;
    explicit SixStepCommutation(const SixStepConfig& config) noexcept : config_(config) {}

    [[nodiscard]] auto config() const noexcept -> const SixStepConfig& { return config_; }

    void set_config(const SixStepConfig& config) noexcept {
        config_ = config;
        reset();
    }

    /** @brief [en] Ok when the configuration describes a usable table. [zh]
   * 配置描述了一张可用的表时 Ok。 */
    [[nodiscard]] auto validate() const noexcept -> ErrorCode {
        if (!(config_.amplitude > 0.0F) || !(config_.amplitude <= 1.0F) || (config_.max_step_sectors < 1U) ||
            (config_.max_step_sectors > 5U)) {
            return ErrorCode::Param;
        }
        return ErrorCode::Ok;
    }

    void reset() noexcept {
        sector_ = kHallNoSector;
        code_ = 0U;
        rejected_ = 0U;
        illegal_ = 0U;
    }

    /**
   * @brief [en] Advance with the code the hall port reported this period.
   * @brief [zh] 用霍尔端口本周期上报的编码推进。
   */
    [[nodiscard]] auto step(std::uint8_t code) noexcept -> SixStepOutput {
        SixStepOutput out{ErrorCode::Ok, Uvw<float>{0.5F, 0.5F, 0.5F}, sector_, false, false, false};
        if (validate() != ErrorCode::Ok) {
            out.status = ErrorCode::Param;
            return out;
        }
        const std::uint8_t named = hall_sector(code);
        if (named == kHallNoSector) {
            // The pins name no position: zero vector, and the last sector is kept so
            // the next legal read is still compared against where the rotor actually
            // was.
            // 引脚没有指出位置：零矢量，并保留上一个扇区，好让下一次合法读取仍然对标转子真正在过的地方。
            ++illegal_;
            out.duty = six_step_duty(code, config_.amplitude, config_.direction);
            out.holding = true;
            return out;
        }
        if (sector_ == kHallNoSector) {
            accept(named, code);
            out.changed = true;
        } else if (named != sector_) {
            const std::uint8_t gap = angular_distance(sector_, named);
            if (gap > config_.max_step_sectors) {
                ++rejected_;
                out.skipped = true;
            } else {
                accept(named, code);
                out.changed = true;
            }
        }
        out.duty = six_step_duty(code_, config_.amplitude, config_.direction);
        out.sector = sector_;
        return out;
    }

    [[nodiscard]] auto sector() const noexcept -> std::uint8_t { return sector_; }
    /** @brief [en] transitions refused because they were too far. [zh]
   * 因为太远而被拒绝的迁移次数。 */
    [[nodiscard]] auto rejected() const noexcept -> std::uint32_t { return rejected_; }
    /** @brief [en] periods whose code named no sector. [zh]
   * 编码没有指出扇区的周期数。 */
    [[nodiscard]] auto illegal() const noexcept -> std::uint32_t { return illegal_; }

  private:
    /** @brief [en] How far apart two sectors are around the six-sector ring,
   * 0..3. [zh] 两个扇区在六格环上相隔多远，0..3。 */
    [[nodiscard]] static auto angular_distance(std::uint8_t from, std::uint8_t to) noexcept -> std::uint8_t {
        const auto forward = static_cast<std::uint8_t>((to + kHallSectors - from) % kHallSectors);
        const auto backward = static_cast<std::uint8_t>(kHallSectors - forward);
        return (forward < backward) ? forward : backward;
    }

    void accept(std::uint8_t sector, std::uint8_t code) noexcept {
        sector_ = sector;
        code_ = code;
    }

    SixStepConfig config_{0.9F, HallDirection::Forward, 1U};
    std::uint8_t sector_{kHallNoSector};
    std::uint8_t code_{0U};
    std::uint32_t rejected_{0U};
    std::uint32_t illegal_{0U};
};

} // namespace core

} // namespace foc
