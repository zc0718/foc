// Conan::ImportStart
#pragma once

#include <cmath>
#include <cstdint>
// Conan::ImportEnd

namespace foc {

namespace sil {

namespace {

constexpr float kTwoPiRad = 6.28318530717958647692F;
constexpr float kHalfPiRad = 1.57079632679489661923F;
constexpr float kThirdTurnRad = 2.09439510239319549230F;

} // namespace

/**
 * @brief [en] What a set of three Hall sensors is like, in the two ways that matter for a decoder: where each sensor
 *             sits, and whether one of them is dead.
 * @brief [zh] 一组三个霍尔传感器"长什么样"，只看与解码器有关的两件事：每个传感器装在哪，以及其中一个是不是坏了。
 * @note [en] The sensors are 120 degrees apart in *electrical* angle and each is high for 180 degrees, which is the
 *            arrangement every three-Hall BLDC drive uses. `mount_error_rad[i]` displaces sensor `i` from its nominal
 *            position `i * 120 deg`; positive is the direction of rotation. `stuck_sensor == 3` means no fault -- there
 *            is no legal sensor index 3, so the value cannot be reached by accident. Hysteresis is deliberately **not**
 *            modelled. A real sensor's latch width only matters when there is noise or dither at the edge, and this
 *            model has neither, so a hysteresis knob would be a knob with no observable effect -- the shape of defect
 *            the ledger's A-13 records. It arrives together with a noise model or not at all.
 * @note [zh] 三个传感器在**电角度**上相隔 120°，每个高电平持续 180°，也就是所有三霍尔无刷驱动用的那种排布。
 *            `mount_error_rad[i]` 把第 `i` 个传感器从它的标称位置 `i*120°` 挪开，正向为旋转方向。 `stuck_sensor == 3`
 *            表示没有故障——合法的传感器下标里没有 3，所以这个值不会被误撞上。回差（滞环）**故意**没有建模。真实传感器的
 *            锁存宽度只在边沿处有噪声或抖动时才有意义，而这个模型两者都没有，所以一个回差旋钮会是个没有可观测效果的旋钮
 *            ——正是台账 A-13 记的那种缺陷形态。它要么与噪声模型一起到位，要么就不来。
 * @since 0.1.9
 */
struct HallSetup final {
    float mount_error_rad[3]{0.0F, 0.0F, 0.0F}; ///< [en] per-sensor placement error, rad. [zh] 每个传感器的安装误差，rad。
    std::uint8_t stuck_sensor{3U};              ///< [en] 0..2 to stick a sensor, 3 = healthy. [zh] 0..2 表示卡死该传感器，3 = 正常。
    bool stuck_high{false};                     ///< [en] the level a stuck sensor holds. [zh] 卡死的传感器保持的电平。
};

/**
 * @brief [en] Three latching Hall sensors on a rotor, as a function of electrical angle.
 * @brief [zh] 转子上三个锁存型霍尔传感器，作为电角度的函数。
 * @note [en] Sensor `i` reads high while `cos(theta - phi_i) > 0`, i.e. for 180 degrees around its own position, and
 *            `phi_i = i * 120 deg + mount_error_rad[i]`. Bit `i` of the code is sensor `i`, so the six reachable codes
 *            are exactly **1..6** and the two codes a three-bit field has left over -- 0 and 7 -- are the ones no
 *            healthy set of sensors can produce. That is why every driver in the field tests `state == 0 || state ==
 *            7`: it is not a convention, it is the arithmetic of the encoding.
 * @note [zh] 第 `i` 个传感器在 `cos(theta - phi_i) > 0` 时为高，即围绕自身位置 180°，其中 `phi_i = i*120° +
 *            mount_error_rad[i]`。编码的第 `i` 位就是第 `i` 个传感器，因此六个可达编码恰好是 **1..6**，而三位字段剩下的
 *            两个编码 0 和 7 正是任何健康传感器组都产生不出来的。这就是为什么现场每个驱动都会判 `state == 0 || state ==
 *            7`：这不是约定，是这种编码的算术。
 * @since 0.1.9
 */
class HallSensorModel final {
  public:
    explicit HallSensorModel(const HallSetup &setup = HallSetup{}) noexcept;

    /**
     * @brief [en] The three-bit code at this electrical angle, 0..7.
     * @brief [zh] 该电角度下的三位编码，0..7。
     * @since 0.1.9
     */
    [[nodiscard]] auto code(float theta_e_rad) const noexcept -> std::uint8_t;

    /**
     * @brief [en] Electrical angle at which sensor `i` switches on, normalised to [0, 2*pi).
     * @brief [zh] 第 `i` 个传感器由低变高的电角度，归一化到 [0, 2π)。
     * @since 0.1.9
     */
    [[nodiscard]] auto switch_on_rad(std::uint8_t sensor) const noexcept -> float;

    /**
     * @brief [en] True for the six codes a healthy set of sensors can produce.
     * @brief [zh] 健康的传感器组能产生的六个编码为真。
     * @since 0.1.9
     */
    [[nodiscard]] static auto legal(std::uint8_t code) noexcept -> bool;

  private:
    HallSetup setup_;
};

inline auto HallSensorModel::legal(std::uint8_t code) noexcept -> bool {
    return code != 0U && code != 7U;
}

inline auto wrap_two_pi(float angle_rad) noexcept -> float {
    float wrapped = std::fmod(angle_rad, kTwoPiRad);
    if (wrapped < 0.0F) {
        wrapped += kTwoPiRad;
    }
    return wrapped;
}

inline HallSensorModel::HallSensorModel(const HallSetup &setup) noexcept : setup_(setup) {}

inline auto HallSensorModel::switch_on_rad(std::uint8_t sensor) const noexcept -> float {
    // Sensor `i` is high from `phi_i - 90 deg` to `phi_i + 90 deg`; it switches on at the first of those.
    const auto index = static_cast<std::size_t>(sensor);
    const float phi = (static_cast<float>(index) * kThirdTurnRad) + setup_.mount_error_rad[index];
    return wrap_two_pi(phi - kHalfPiRad);
}

inline auto HallSensorModel::code(float theta_e_rad) const noexcept -> std::uint8_t {
    float axis[3];
    for (std::size_t i = 0; i < 3U; ++i) {
        axis[i] = theta_e_rad - ((static_cast<float>(i) * kThirdTurnRad) + setup_.mount_error_rad[i]);
    }
    std::uint8_t bits = 0U;
    for (std::uint8_t i = 0; i < 3U; ++i) {
        const bool high = std::cos(axis[i]) > 0.0F;
        if (high) {
            bits = static_cast<std::uint8_t>(bits | (1U << i));
        }
    }
    if (setup_.stuck_sensor < 3U) {
        const auto mask = static_cast<std::uint8_t>(1U << setup_.stuck_sensor);
        if (setup_.stuck_high) {
            bits = static_cast<std::uint8_t>(bits | mask);
        } else {
            bits = static_cast<std::uint8_t>(bits & static_cast<std::uint8_t>(~mask));
        }
    }
    return bits;
}

} // namespace sil

} // namespace foc
