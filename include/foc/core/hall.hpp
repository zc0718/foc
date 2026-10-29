// Conan::ImportStart
#pragma once

#include <cmath>
#include <cstdint>

#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] Which way the six-step table should push, given the same sensor code.
 * @brief [zh] 同一个传感器编码下，六步表该往哪个方向推。
 * @note [en] Direction is **not** a property of the sensors. The code says where the rotor is; producing torque in one
 *            direction or the other is the drive's decision, and it is the same code read against the opposite vector
 *            -- exactly 180 degrees away, which on a six-vector star is three places.
 * @note [zh] 方向**不是**传感器的性质。编码只说转子在哪；往哪个方向出力矩是驱动的决定，而它就是同一个编码对着相反的矢量
 *            读——恰好相差 180°，在六矢量星上就是隔三格。
 * @since 0.1.9
 * @exporter
 */
enum class HallDirection : std::uint8_t {
    Forward = 0, ///< [en] push toward increasing electrical angle. [zh] 朝电角度增大的方向推。
    Reverse = 1, ///< [en] push the other way, the same table three sectors later. [zh] 反向推，同一张表往后挪三格。
};


/**
 * @brief [en] How many sectors one electrical revolution is divided into by a three-Hall set.
 * @brief [zh] 三霍尔组把一圈电角度分成的扇区数。
 * @since 0.1.9
 */
inline constexpr std::uint8_t kHallSectors = 6U;

/**
 * @brief [en] A sector index that means "no sector": an illegal code, or no edge seen yet.
 * @brief [zh] 表示"没有扇区"的索引：编码非法，或还没见过边沿。
 * @since 0.1.9
 */
inline constexpr std::uint8_t kHallNoSector = 0xFFU;


/**
 * @brief [en] The sector a code names, or `kHallNoSector` for the two codes a three-bit field has left over.
 * @brief [zh] 编码所指的扇区；对三位字段剩下的两个编码返回 `kHallNoSector`。
 * @note [en] Sector `k` spans electrical angles `[k*60 - 30, k*60 + 30)`, i.e. it is **centred** on `k*60`, because
 *            that is where the three sensors place their edges: sensors at 0/120/240 switch at 30/90/150/... and never
 *            at zero. Measured in `test_hall_physics.cpp` rather than assumed -- an earlier version of that file
 *            assumed the edges fell on multiples of 60 and straddled one (ledger C-7).
 * @note [zh] 扇区 `k` 覆盖电角度 `[k*60 − 30, k*60 + 30)`，即**以 `k*60` 为中心**，因为三个传感器正是把边沿放在那里：
 *            0/120/240 的传感器在 30/90/150/…处切换，从不在零处。这是 `test_hall_physics.cpp` 里量出来的，不是假设的——
 *            那个文件更早的一版假设边沿落在 60 的倍数上，结果跨过了一条边沿（台账 C-7）。
 * @since 0.1.9
 */
[[nodiscard]] constexpr auto hall_sector(std::uint8_t code) noexcept -> std::uint8_t {
    // Indexed by code, bit i = sensor i. The six legal codes are 1..6 and the mapping is the rotor's order, measured:
    // 100 -> 0, 110 -> 1, 010 -> 2, 011 -> 3, 001 -> 4, 101 -> 5.
    // 按编码索引，第 i 位 = 第 i 个传感器。六个合法编码是 1..6，映射是转子的顺序，实测得到。
    constexpr std::uint8_t table[8] = {kHallNoSector, 0U, 2U, 1U, 4U, 5U, 3U, kHallNoSector};
    return table[code & 0x07U];
}

/**
 * @brief [en] Electrical angle at the centre of a sector, rad.
 * @brief [zh] 扇区中心的电角度，rad。
 * @since 0.1.9
 */
[[nodiscard]] constexpr auto hall_sector_centre_rad(std::uint8_t sector) noexcept -> float {
    constexpr float kSectorRad = 1.04719755119659774615F; // pi/3
    return static_cast<float>(sector) * kSectorRad;
}


/**
 * @brief [en] The three duties that hold the six-step vector for this code.
 * @brief [zh] 为该编码保持六步矢量的三个占空比。
 * @note [en] Two phases are driven and the third is left at mid-rail, which is what six-step *is*: the vector can only
 *            ever be one of six, at `90 deg + k*60` for the sector `k` the code names (forward). The 90 is not an
 *            offset to tune -- it is where maximum torque lives, and the six vectors straddle it, which is why six-step
 *            has an irreducible torque ripple: the torque angle runs from +30 to -30 degrees inside a sector, so the
 *            torque runs over `cos(30 deg)` = 0.866 to 1, i.e. 14.0 % peak-to-peak of the mean. Measured on the plant
 *            in `test_hall_decoder.cpp`; the point of an interpolated angle is to remove exactly this. An illegal code
 *            answers all three phases at mid-rail -- a zero vector, which produces no torque, rather than a guess that
 *            produces the wrong torque.
 * @note [zh] 两相驱动、第三相留在中点——这就是六步**本身**：矢量只能是六个之一，对编码所指的扇区 `k` 在 `90° + k*60°`（
 *            正向）。90° 不是可调的偏置——它是最大力矩所在，而六个矢量夹着它，这正是六步不可消除的力矩脉动来源：扇区内转
 *            矩角从 +30° 走到 −30°，力矩在 `cos(30°)` = 0.866 到 1 之间，即均值的 14.0 % 峰峰。在
 *            `test_hall_decoder.cpp` 里对 plant 实测；插值角度的意义就是消掉这一项。非法编码回答三相全在中点——零矢量，
 *            不产生力矩，而不是一个产生错误力矩的猜测。
 * @since 0.1.9
 */
[[nodiscard]] auto six_step_duty(std::uint8_t code, float amplitude, HallDirection direction) noexcept -> Uvw<float>;

/**
 * @brief [en] The sector, the vector and the speed a three-Hall set can offer, estimated from edge timing.
 * @brief [zh] 三霍尔组能提供的扇区、矢量与转速——由边沿时刻估计。
 * @note [en] Why a tracker and not `atan2`. The code alone locates the rotor to one sector, i.e. to +-30 degrees (half
 *            a sector, and no arithmetic improves it -- measured in `test_hall_physics.cpp`). What is left is *when*
 *            the edges arrive: a rotor turning at constant speed crosses a 60-degree sector in a known time, so between
 *            two edges the angle is a straight line and the error collapses from 30 degrees to the timing resolution
 *            times the speed -- 3.2 degrees at 900 rad/s electrical with a 62.5 us period, and it is a *sampling* limit
 *            rather than a sensor limit. What it cannot do, and says so. A stuck sensor freezes its bit, so the code
 *            stays legal for five sixths of a revolution while the angular resolution silently halves and the edge
 *            interval doubles. Neither channel sees all of it: measured on a dead sensor at 300 rad/s, `Fault` (the
 *            illegal window) covers 15.5 %, `State` (the interval check) another 53.9 %, and the remaining 8.4 % reads
 *            `Ok` while wrong by up to 59 degrees -- 13.7 % and 120 degrees with the sensor stuck high. So roughly a
 *            third of a revolution is undeclared, and a drive built on Halls needs a second opinion (current or torque
 *            sanity, an index) rather than a cleverer check on the codes. Neither channel tells a **stall** from a
 *            **missed edge** -- both are "no edge for too long" -- so `status()` reports `State` rather than `Fault`.
 *            The readings are counted, not latched: `illegal_reads()` and `stale_reads()` accumulate, and whether a
 *            single glitched sample should stop an axis is a decision this layer does not make (the same division of
 *            labour as `torque_map`'s `limited`).
 *
 * @note [zh] 为什么要有跟踪器而不是 `atan2`。只看编码只能把转子定位到一个扇区内，即 ±30°（半个扇区，靠算术改不了
 *            ——`test_hall_physics.cpp` 实测）。剩下的是边沿**什么时候**到：匀速转子穿过 60° 扇区的时间是已知的，所以两
 *            个边沿之间角度是一条直线，误差从 30° 塌缩为"定时分辨率 × 转速"——62.5 µs 周期、900 rad/s 电角下是 3.2°，而
 *            且这是**采样**极限，不是传感器极限。
 *
 *     它做不到什么，并且会说出来。卡死的传感器冻结自己那一位，于是转过五分之六圈编码都合法，而角度分辨率悄悄减半、边沿
 *     间隔翻倍。两条通道都看不全：300 rad/s 下对坏一路实测，`Fault`（非法窗口）覆盖 15.5 %， `State`（间隔检查）另占
 *     53.9 %，剩下 8.4 % 报 `Ok` 而误差最多 59°——高位卡死时是 13.7 % 与 120°。所以大约三分之一的圈数没有被申报，建在霍
 *     尔上的驱动需要第二个意见（电流或力矩的合理性、索引信号），而不是在编码上更聪明的检查。两条通道都分不清**堵转**与
 *     **丢边沿**——都是"太久没有边沿"——所以 `status()` 报 `State` 而不是 `Fault`。
 *
 *     读数只计数、不锁存：`illegal_reads()` 与 `stale_reads()` 累加，而"单次毛刺是否该停轴"是本层不做的决定（与
 *     `torque_map` 的 `limited` 同一种分工）。
 * @since 0.1.9
 * @exporter
 */
class HallTracker final {
  public:
    /**
     * @brief [en] How far an interval may stray from the last one before it stops being a measurement: it must lie in
     *             `[period/1.5, period*1.5]`.
     * @brief [zh] 一个间隔可以偏离上一个多远才不再算一次测量：它必须落在 `[周期/1.5, 周期*1.5]` 内。
     * @note [en] 1.5 is the arithmetic midpoint between a healthy interval (1.0) and the interval a dead sensor
     *            produces (2.0, measured in `test_hall_physics.cpp`): the healthy side has 50 % of margin and the fault
     *            side 25 %. It is two-sided because both sides were measured to bite: a doubled interval adopted as the
     *            new reference hides a dead sensor for the rest of the run, and a *short* one -- six samples of a
     *            genuine but sliver-thin sector just after a fault clears -- poisons the reference so that every true
     *            interval afterwards looks late (measured: 375 us against 3.5 ms, `State` from then on).
     * @note [zh] 1.5 是健康间隔（1.0）与坏一路产生的间隔（2.0，`test_hall_physics.cpp` 实测）之间的算术中点：健康一侧留
     *            50 % 余量，故障一侧 25 %。它是双边的，因为两侧实测都会咬人：被当成新参照的翻倍间隔会在此后的整段运行里
     *            把坏掉的一路藏起来，而过**短**的间隔——故障刚消失后一个真实但极窄的扇区留下的六次采样——会把参照带偏，使
     *            此后每个真实间隔都显得迟到（实测：375 µs 对 3.5 ms，从此永远 `State`）。
     */
    static constexpr float kIntervalRatio = 1.5F;

    HallTracker() noexcept = default;

    /**
     * @brief [en] Forget everything: no sector, no speed, no counts.
     * @brief [zh] 忘掉一切：无扇区、无转速、无计数。
     * @since 0.1.9
     */
    void reset() noexcept;

    /**
     * @brief [en] Feed one sample of the sensor code. Call it every control period, not only on change: a period with
     *             no edge is how a stall becomes visible.
     * @brief [zh] 送入一次传感器编码采样。每个控制周期都调，不是只在变化时调：没有边沿的那个周期正是堵转显形的方式。
     * @param now_s [en] monotonic time of the sample, s. @param now_s
     * @param now_s [zh] 采样的单调时间，s。
     * @param code [en] the three-bit code, 0..7. @param code
     * @param code [zh] 三位编码，0..7。
     * @return [en] `Ok` when the sample was a legal code, `Fault` when it was 0 or 7, `State` before the first edge.
     * @return [zh] 采样是合法编码时为 `Ok`，是 0 或 7 时为 `Fault`，在第一个边沿之前为 `State`。
     * @since 0.1.9
     */
    auto update(float now_s, std::uint8_t code) noexcept -> ErrorCode;

    /**
     * @brief [en] Electrical angle interpolated from the last edge, clamped to its sector, rad in [0, 2*pi).
     * @brief [zh] 由最近边沿插值并钳在其扇区内的电角度，rad，范围 [0, 2π)。
     * @note [en] The clamp is not cosmetic: with the rotor stopped the line would keep going and the estimate would
     *            leave the sector the sensors are still reporting, i.e. grow an error without bound while the code says
     *            otherwise. The sensors are the authority on *which* sector, so the estimate never leaves it.
     * @note [zh] 钳位不是装饰：转子停住时直线会继续走，估计值会离开传感器仍在报告的那个扇区，也就是在编码说别的话时误差
     *            无界增长。扇区的归属以传感器为准，所以估计值不越出它。
     * @since 0.1.9
     */
    [[nodiscard]] auto angle_elec_rad(float now_s) const noexcept -> float;

    /**
     * @brief [en] Signed electrical speed from the last two edges, rad/s; 0 until two edges have been seen.
     * @brief [zh] 由最近两个边沿得到的有符号电角速度，rad/s；在见到两个边沿之前为 0。
     * @since 0.1.9
     */
    [[nodiscard]] auto speed_elec_rad_s() const noexcept -> float;

    /**
     * @brief [en] The last sector, or `kHallNoSector`.
     * @brief [zh] 最近的扇区，或 `kHallNoSector`。
     * @since 0.1.9
     */
    [[nodiscard]] auto sector() const noexcept -> std::uint8_t;

    /**
     * @brief [en] `Ok`, `Fault` when the last sample was illegal, or `State` before the first edge or while waiting too
     *             long for one.
     * @brief [zh] `Ok`；最近一次采样非法时为 `Fault`；在第一个边沿之前或等边沿等得太久时为 `State`。
     * @since 0.1.9
     */
    [[nodiscard]] auto status() const noexcept -> ErrorCode;

    /**
     * @brief [en] Time since the last edge, s; `0` when no edge has been seen.
     * @brief [zh] 距最近边沿的时间，s；没见过边沿时为 `0`。
     * @since 0.1.9
     */
    [[nodiscard]] auto age_s() const noexcept -> float;

    /**
     * @brief [en] The last measured sector period, s; 0 until two edges have been seen.
     * @brief [zh] 最近一次实测的扇区周期，s；见到两个边沿之前为 0。
     * @since 0.1.9
     */
    [[nodiscard]] auto period_s() const noexcept -> float;

    /**
     * @brief [en] Samples that read 0 or 7.
     * @brief [zh] 读到 0 或 7 的采样数。
     * @since 0.1.9
     */
    [[nodiscard]] auto illegal_reads() const noexcept -> std::uint32_t;

    /**
     * @brief [en] Times an interval was refused: an edge arriving on time it did not, or arriving where the length
     *             could not be a sector.
     * @brief [zh] 间隔被拒收的次数：边沿迟到而没有按时到，或到了但长度不可能是一个扇区。
     * @since 0.1.9
     */
    [[nodiscard]] auto stale_reads() const noexcept -> std::uint32_t;

  private:
    std::uint8_t sector_{kHallNoSector};
    float edge_s_{0.0F};      ///< [en] time of the last edge. [zh] 最近边沿的时刻。
    float entry_angle_{0.0F}; ///< [en] angle the rotor had at that edge, rad. [zh] 该边沿时转子的角度。
    float speed_{0.0F};       ///< [en] rad/s electrical, signed. [zh] 电角速度，有符号。
    float period_{0.0F};      ///< [en] last sector period, s. [zh] 最近的扇区周期，s。
    float now_s_{0.0F};       ///< [en] time of the last sample. [zh] 最近一次采样的时刻。
    bool seen_edge_{false};
    /**
     * @brief [en] the stored timestamp came from a transition, not the first sample.
     * @brief [zh] 存下的时刻来自一次真实跳变，而不是首个采样。
     */
    bool previous_was_edge_{false};
    /**
     * @brief [en] an illegal read happened since the last edge, so the crossing was missed.
     * @brief [zh] 上次边沿之后出现过非法采样，因此那次穿越没有被看见。
     */
    bool unobserved_{false};
    bool stale_latched_{false};
    ErrorCode status_{ErrorCode::State};
    std::uint32_t illegal_reads_{0U};
    std::uint32_t stale_reads_{0U};
};


namespace {

constexpr float kHallSectorRad = 1.04719755119659774615F; ///< [en] pi/3, one sector, rad. [zh] π/3，一个扇区，rad。
constexpr float kHallTwoPiRad = 6.28318530717958647692F;  ///< [en] 2*pi. [zh] 2π。

auto hall_wrap(float angle_rad) noexcept -> float {
    float wrapped = std::fmod(angle_rad, kHallTwoPiRad);
    if (wrapped < 0.0F) {
        wrapped += kHallTwoPiRad;
    }
    return wrapped;
}

} // namespace

inline auto six_step_duty(std::uint8_t code, float amplitude, HallDirection direction) noexcept -> Uvw<float> {
    constexpr float kMidRail = 0.5F;
    const std::uint8_t sector = hall_sector(code);
    if (sector == kHallNoSector) {
        // No torque, rather than a guess: an illegal code is a sensor fault and the honest command is zero volts.
        // 不产生力矩，而不是猜：非法编码意味着传感器故障，诚实的指令是零电压。
        return Uvw<float>{kMidRail, kMidRail, kMidRail};
    }
    // Half-bridge multipliers, exactly one +1, one -1 and one 0 per step: two phases driven, the third at mid-rail.
    // Step `j` places the vector at `30 + j*60` degrees, and the forward vector for sector `k` belongs at `90 + k*60`,
    // i.e. `j = k + 1`. Reverse is the same table three steps further -- 180 degrees, no second table to keep in sync.
    // 半桥系数，每步恰好一个 +1、一个 −1、一个 0：两相驱动、第三相在中点。第 `j` 步把矢量放在 `30 + j*60` 度，而扇区
    // `k` 的正向矢量应在 `90 + k*60`，即 `j = k + 1`。反向就是同一张表往后挪三步——180°，不需要第二张表来保持同步。
    constexpr float kUnit[6][3] = {
        {1.0F, 0.0F, -1.0F}, // 30 deg:  u+, w-
        {0.0F, 1.0F, -1.0F}, // 90 deg:  v+, w-
        {-1.0F, 1.0F, 0.0F}, // 150 deg: v+, u-
        {-1.0F, 0.0F, 1.0F}, // 210 deg: w+, u-
        {0.0F, -1.0F, 1.0F}, // 270 deg: w+, v-
        {1.0F, -1.0F, 0.0F}, // 330 deg: u+, v-
    };
    const std::uint8_t offset = (direction == HallDirection::Reverse) ? 3U : 0U;
    const auto step = static_cast<std::size_t>((static_cast<std::uint8_t>(sector + 1U + offset)) % kHallSectors);
    const float gain = 0.5F * amplitude;
    return Uvw<float>{kMidRail + (gain * kUnit[step][0]), kMidRail + (gain * kUnit[step][1]),
                      kMidRail + (gain * kUnit[step][2])};
}

inline void HallTracker::reset() noexcept {
    sector_ = kHallNoSector;
    edge_s_ = 0.0F;
    entry_angle_ = 0.0F;
    speed_ = 0.0F;
    period_ = 0.0F;
    now_s_ = 0.0F;
    seen_edge_ = false;
    previous_was_edge_ = false;
    unobserved_ = false;
    stale_latched_ = false;
    status_ = ErrorCode::State;
    illegal_reads_ = 0U;
    stale_reads_ = 0U;
}

inline auto HallTracker::update(float now_s, std::uint8_t code) noexcept -> ErrorCode {
    if (!std::isfinite(now_s)) {
        status_ = ErrorCode::Param;
        return status_;
    }
    now_s_ = now_s;
    const std::uint8_t sector = hall_sector(code);
    if (sector == kHallNoSector) {
        ++illegal_reads_;
        // The wiring went illegal, so any crossing in this stretch happened without being seen. The recovery edge is
        // therefore not an edge in the sense the interval arithmetic needs -- see the transition branch.
        // 接线变成非法，所以这一段里任何穿越都没有被看见。因此恢复时的跳变不是间隔算术所要求的那种边沿——见跳变分支。
        unobserved_ = true;
        status_ = ErrorCode::Fault;
        return status_;
    }
    if (!seen_edge_) {
        seen_edge_ = true;
        sector_ = sector;
        edge_s_ = now_s;
        // One edge is not a speed: the best answer is the centre of the sector the sensors name, which is +-30 deg.
        // 一个边沿不是一个转速：最好的回答是传感器所指扇区的中心，±30°。
        entry_angle_ = hall_sector_centre_rad(sector);
        speed_ = 0.0F;
        period_ = 0.0F;
        previous_was_edge_ = false;
        stale_latched_ = false;
        status_ = ErrorCode::Ok;
        return status_;
    }

    const float elapsed = now_s - edge_s_;
    const bool overdue = (period_ > 0.0F) && (elapsed > kIntervalRatio * period_) && !stale_latched_;
    if (sector != sector_) {
        // How far the code moved, the short way round: 1..3 forward, 4..5 backward. A jump of more than one sector
        // is accepted and divided out rather than refused -- it is what a missing pair of edges looks like, and
        // recovering the speed from it is worth more than declaring a fault the sensors cannot confirm.
        // 编码走了几格（走近路）：1..3 为正向，4..5 为反向。跳超过一格时接受并把时间除开，而不是拒绝——那正是丢了两次
        // 边沿的样子，从中恢复出转速比宣布一个传感器无法确认的故障更有价值。
        const int forward = (static_cast<int>(sector) - static_cast<int>(sector_)) % 6;
        const int turns = (forward < 0) ? (forward + 6) : forward;
        // Which way the code moved, and it is not always the short arc: a jump of four or five sectors is either two
        // or one sector backwards, or four or five forwards, and the codes alone cannot say. Falling back on the last
        // known speed's sign is the only information available, and it is right whenever the rotor did not reverse
        // while the wiring was unreadable -- measured, it takes the worst undetected error from 120 degrees to 59.
        // 编码往哪边走了，而这并不总是短弧：跳四格或五格，既可能是倒退两格或一格，也可能是前进四格或五格，光看编码分不出来。
        // 退回"最近已知转速的符号"是唯一可用的信息，只要转子在接线不可读期间没有反向就是对的——实测它把最坏未被发现的误差
        // 从 120° 降到 59°。
        const bool backwards = (turns > 3) && (speed_ <= 0.0F);
        const int steps = backwards ? (6 - turns) : turns;

        // The period *before* this edge is the only thing that can say whether this interval was too long. Measuring
        // it against the period this same edge just produced would compare the interval with itself and could never
        // fire -- the kind of criterion that is green because it cannot fail.
        // 只有这次边沿**之前**的周期才能说这次间隔是不是太长。拿它和这次边沿刚算出的周期比，等于把间隔与它自己比，永远
        // 不会触发——那种"因为不可能失败而变绿"的判据。
        const float previous_period = period_;

        // An interval is a sector only if *both* of its ends were edges, and its length is spent only if it is not
        // already evidence of a fault. Adopting a doubled interval as the new period is exactly what a dead sensor
        // needs to stay hidden: it turns the whole sector into a silently wrong answer instead of a reported one --
        // measured, 36 % of the revolution reporting `Ok` with an error up to 120 degrees, against 0.4 % once the
        // implausible interval is refused. The price is stated rather than hidden: a deceleration that changes the
        // speed by a third inside one sector is also refused, which is why `State` is not a fault.
        // 只有当间隔**两端**都是边沿时它才是一个扇区，而且只有它还不构成故障证据时，它的长度才被采信。把翻倍的间隔当成新
        // 周期，正是坏一路藏身所需的东西：它把整个扇区变成沉默的错误答案而不是被报告的答案——实测 36% 的圈数报 `Ok` 而
        // 误差最多 120°，拒收不可信间隔之后是 0.4%。代价写出来而不是藏起来：在一个扇区内把转速改变三分之一的减速同样被
        // 拒收，这正是 `State` 不是故障的原因。
        const bool have_reference = previous_was_edge_ && (previous_period > 0.0F);
        const float expected = previous_period * static_cast<float>(steps);
        const bool implausible = have_reference && (unobserved_ || (elapsed > kIntervalRatio * expected) ||
                                                    ((elapsed * kIntervalRatio) < expected));
        if (previous_was_edge_ && (elapsed > 0.0F) && (steps > 0) && !implausible) {
            period_ = elapsed / static_cast<float>(steps);
            const float magnitude = kHallSectorRad / period_;
            speed_ = backwards ? -magnitude : magnitude;
        }
        if (implausible) {
            ++stale_reads_;
        }
        stale_latched_ = implausible;
        unobserved_ = false;

        sector_ = sector;
        edge_s_ = now_s;
        previous_was_edge_ = true;
        // The rotor entered this sector across the boundary shared with the previous one, and which boundary that is
        // depends on the direction: the lower edge going forward, the upper edge going backward.
        // 转子是跨着与上一扇区共享的那条边界进入本扇区的，而那条边界在哪取决于方向：正向是下边沿，反向是上边沿。
        entry_angle_ = hall_sector_centre_rad(sector) + (backwards ? kHallSectorRad * 0.5F : -kHallSectorRad * 0.5F);
        // The status follows the interval verdict, not the fact that a code arrived. A transition with an untrustworthy
        // interval is *not* an `Ok` sample: the first version reported `Ok` here and the next period said `State`, and
        // the single `Ok` sample was the one carrying the 119-degree error -- the worst possible place for it.
        // 状态跟着间隔的判决走，而不是跟着"编码到了"这件事。间隔不可信时的跳变**不是**一个 `Ok` 采样：第一版在这里报
        // `Ok`，下一个周期才报 `State`，而那个唯一的 `Ok` 采样正是带着 119° 误差的那个——最糟糕的位置。
        status_ = stale_latched_ ? ErrorCode::State : ErrorCode::Ok;
        return status_;
    }

    if (overdue) {
        // No edge where one was due. A stall and a missing pair of edges look identical from the code stream alone,
        // so this is a state, not a fault -- the interval check above is what counts the other channel.
        // 该有边沿的地方没有边沿。只从编码流看，堵转与丢了一对边沿完全一样，所以这是一个状态而不是故障——另一条通道由上面
        // 的间隔检查计数。
        ++stale_reads_;
        stale_latched_ = true;
    }
    status_ = (stale_latched_ || ((period_ > 0.0F) && (elapsed > kIntervalRatio * period_))) ? ErrorCode::State
                                                                                             : ErrorCode::Ok;
    return status_;
}

inline auto HallTracker::angle_elec_rad(float now_s) const noexcept -> float {
    if (!seen_edge_) {
        return 0.0F;
    }
    const float half = kHallSectorRad * 0.5F;
    const float centre = hall_sector_centre_rad(sector_);
    float angle = entry_angle_ + (speed_ * (now_s - edge_s_));
    // Clamped to the sector the sensors are still reporting: a stopped rotor would otherwise walk the line out of a
    // sector that no longer contains it.
    // 钳在传感器仍在报告的那个扇区内：否则停住的转子会让直线走出一个已经不再包含它的扇区。
    if (angle < (centre - half)) {
        angle = centre - half;
    }
    if (angle > (centre + half)) {
        angle = centre + half;
    }
    return hall_wrap(angle);
}

inline auto HallTracker::speed_elec_rad_s() const noexcept -> float { return speed_; }

inline auto HallTracker::sector() const noexcept -> std::uint8_t { return sector_; }

inline auto HallTracker::status() const noexcept -> ErrorCode { return status_; }

inline auto HallTracker::age_s() const noexcept -> float {
    // Age as of the most recent sample: the caller owns the clock, so this cannot be fresher than the last `update`.
    // It is the number the interval check used, which is what makes "how late is the edge" answerable by a caller.
    // 最近一次采样时刻的"年龄"：时钟由调用方掌握，因此它不可能比上一次 `update` 更新。它正是间隔检查用的那个数，也正是
    // 让调用方可以回答"边沿迟了多久"的东西。
    return seen_edge_ ? (now_s_ - edge_s_) : 0.0F;
}

inline auto HallTracker::period_s() const noexcept -> float { return period_; }

inline auto HallTracker::illegal_reads() const noexcept -> std::uint32_t { return illegal_reads_; }

inline auto HallTracker::stale_reads() const noexcept -> std::uint32_t { return stale_reads_; }


} // namespace core

} // namespace foc
