// Conan::ImportStart
#pragma once

#include <cstdint>

#include <foc/core/transforms.hpp>
#include <foc/core/types.hpp>
#include <foc/driver/stub_brake_switch.hpp>
#include <foc/driver/stub_fault_input.hpp>
#include <foc/hal/brake_switch.hpp>
#include <foc/hal/current_sense.hpp>
#include <foc/hal/dc_link.hpp>
#include <foc/hal/position.hpp>
#include <foc/hal/pwm.hpp>
#include <foc/motor/motor_config.hpp>

#include "sil_bus.hpp"
#include "sil_discrete_inverter.hpp"
#include "sil_pmsm_plant.hpp"
// Conan::ImportEnd

namespace foc {

namespace sil {

using core::Ab;
using core::Dq;
using core::Uvw;
using motor::MotorConfig;

/**
 * @brief [en] Inverter adapter: records the duty and lets the bench push it into the plant.
 *
 * @brief [zh] 逆变器适配层：记录占空比，由台架把它送进 plant。
 *
 *   The duty is held in the **stationary** frame for a whole control period (zero-order hold on the applied stator
 *   voltage vector), which is what a real inverter does. Disabled or not-yet-enabled states hold 50% duty, i.e. zero
 *   voltage. 占空比在整个控制周期内保持于 **静止** 坐标系（作用到定子电压矢量上的零阶保持），这才是真实 逆变器的行为。
 *   未使能/已关闭状态保持 50% 占空比，即零电压。
 * @since 0.1.0
 */
class PlantPwm final : public hal::Pwm {
  public:
    PlantPwm() noexcept = default;

    auto configure(float freq_hz, float deadtime_ns) noexcept -> ErrorCode override;
    void write_duty(Uvw<float> duty) noexcept override;
    auto enable() noexcept -> ErrorCode override;
    void disable() noexcept override;
    [[nodiscard]] auto fault() const noexcept -> bool override;

    /**
   * @brief [en] Last written duty (50% when disabled).
   * @brief [zh] 最近写入的占空比（关闭时为 50%）。
   */
    [[nodiscard]] auto duty() const noexcept -> Uvw<float>;

    /**
   * @brief [en] Carrier frequency in Hz.
   * @brief [zh] 载波频率，Hz。
   */
    [[nodiscard]] auto freq_hz() const noexcept -> float;

    /**
   * @brief [en] Whether the stage is enabled.
   * @brief [zh] 输出级是否使能。
   */
    [[nodiscard]] auto enabled() const noexcept -> bool;

  private:
    Uvw<float> duty_{0.5F, 0.5F, 0.5F};
    float freq_hz_{0.0F};
    float deadtime_ns_{0.0F};
    bool enabled_{false};
};

/**
 * @brief [en] Current sensing adapter reading the plant's phase currents.
 * @brief [zh] 电流采样适配层，读取 plant 的相电流。
 * @since 0.1.0
 */
class PlantCurrentSense final : public hal::CurrentSense {
  public:
    explicit PlantCurrentSense(const PmsmPlant& plant, const PlantPwm& pwm) noexcept;

    auto read_uvw() noexcept -> Uvw<float> override;
    auto calibrate_offset() noexcept -> ErrorCode override;
    [[nodiscard]] auto sample_rate_hz() const noexcept -> float override;

    /**
   * @brief [en] Per-phase gain of the measuring chain; 1 is a perfect sensor.
   * @brief [zh] 测量链的每相增益；1 表示理想传感器。
   * @param gain_uvw [en] multiplier applied to each phase reading.
   * @param gain_uvw [zh] 施加到每相读数的乘数。
   * @note [en] `u+10 v-10 %` is the *differential* case on purpose: a uniform gain error is a loop-gain error and
   *            scales with the current, while a mismatch creates a phantom negative-sequence component that the loop
   *            then tries to regulate away. The two are not the same defect, so the knobs are separate per phase.
   * @note [zh] `u+10 v-10 %` 是 **差分** 情形，故意与均匀增益分开：均匀增益是环路增益误差，随电流成比例；相间失配会造出
   *            一个虚假负序分量，环路随后会去调节它。两者不是同一个缺陷，所以每相分开。
   * @since 0.1.0
   */
    void set_phase_gain(Uvw<float> gain_uvw) noexcept;

    /**
   * @brief [en] Offset added to every phase reading, A; 0 is a calibrated sensor.
   * @brief [zh] 加到每相读数上的偏置，A；0 表示已校准。
   * @note [en] A *common* offset cancels identically in Clarke, so it cannot be the source of a d-q error -- that is
   *            measured, not assumed, and it is why this knob exists at all rather than being left out.
   * @note [zh] **共模** 偏置在 Clarke 里严格相互抵消，因此它不可能成为 d-q 误差的来源——这一点是量出来的、不是假设的，也
   *            正是保留这个旋钮（而不是干脆不提供）的原因。
   * @since 0.1.0
   */
    void set_offset_a(float offset_a) noexcept;

  private:
    const PmsmPlant* plant_;
    const PlantPwm* pwm_;
    Uvw<float> gain_{1.0F, 1.0F, 1.0F};
    float offset_a_{0.0F};
};

/**
 * @brief [en] Position adapter reporting the plant rotor state as a sensored encoder would.
 * @brief [zh] 位置适配层，按带编码器的电机那样汇报 plant 的转子状态。
 * @since 0.1.0
 */
class PlantPosition final : public hal::PositionSensor {
  public:
    explicit PlantPosition(const PmsmPlant& plant) noexcept;

    auto angle_mech_rad() noexcept -> float override;
    auto velocity_rad_s() noexcept -> float override;
    [[nodiscard]] auto has_sensor() const noexcept -> bool override;

    /**
   * @brief [en] Model an encoder mounting error the driver has to find.
   * @brief [zh] 模拟驱动需要找出的编码器安装误差。
   */
    void set_mount_error_rad(float error_rad) noexcept;

    /**
   * @brief [en] Quantise the reported angle to `lsb_rad`; 0 (the default) reports it unquantised.
   * @brief [zh] 把报出的角度量化到 `lsb_rad`；0（默认）表示不量化。
   * @note [en] Why this is a separate knob from the mounting error. A mounting error is a *bias*: the loop can converge
   *            to it and sit there, so it costs the same at every target. Quantisation is a *resolution*: the loop
   *            closes on a staircase, so the residual depends on where the target happens to fall between two counts.
   *            Tying them together would make one number explain two different behaviours.
   * @note [zh] 为什么它与安装误差是两个旋钮。安装误差是 **偏置** ：环路会收敛到它并停在那里，所以每个目标处代价相同。量
   *            化是 **分辨率** ：环路在一个台阶上闭环，残差取决于目标恰好落在两个计数之间的哪里。把两者绑在一起，一个数
   *            字就要解释两种不同的行为。
   */
    void set_resolution_rad(float lsb_rad) noexcept;

    /**
   * @brief [en] Model a sensor-less setup: has_sensor() starts returning false, so the angle has to come from the
   *             observer. Same name as `driver::SimPosition` on purpose -- the scenarios read the same either way.
   * @brief [zh] 模拟无感场景：has_sensor() 开始返回 false，角度必须由观测器给出。故意与 `driver::SimPosition` 同名——两
   *             种台架读起来一样。
   */
    void set_sensor_present(bool present) noexcept;

    /**
     * @brief [en] Model an encoder that reports an angle and no speed: `velocity_rad_s()` answers `NaN`.
     * @brief [zh] 模拟只报角度、不报速度的编码器： `velocity_rad_s()` 答 `NaN`。
     * @note [en] What this buys is the one configuration in which a *sensored* axis steps the observer: the angle comes
     *            from the sensor and the speed from the observer, so both opinions exist in the same tick and the
     *            disagreement between them is a measurement rather than an assumption.
     * @note [zh] 它换来的是带传感器轴 **推进观测器** 的唯一配置：角度来自传感器、速度来自观测器，于是两个意见在同一拍里
     *            都存在，它们之间的分歧是一个测量值而不是一个假设。
     */
    void set_velocity_supported(bool supported) noexcept;

    /**
   * @brief [en] The correction offset handed over by alignment.
   * @brief [zh] 由对齐交给传感器的校正偏置。
   */
    void set_offset_mech_rad(float offset_rad) noexcept;

    /**
   * @brief [en] Offset currently subtracted from the readings, rad.
   * @brief [zh] 当前从读数中减去的偏置，rad。
   */
    [[nodiscard]] auto mech_offset_rad() const noexcept -> float;

    /**
   * @brief [en] Rotor angle the plant is actually at, offset removed: the truth a test compares against.
   * @brief [zh] plant 转子真实所在的角度（不含偏置）：供测试比对的真值。
   */
    [[nodiscard]] auto true_angle_mech_rad() const noexcept -> float;

    /**
   * @brief [en] Give this sensor an index mark at a mechanical position, or take it away.
   * @brief [zh] 给这个传感器一个位于某机械角度的索引标记，或把它拿掉。
   * @param position_mech_rad [en] where the mark sits in plant-truth terms.
   * @param position_mech_rad [zh] 标记在 plant 真值坐标里的位置。
   * @note [en] Off by default: the mark is an optional capability, and a rig that had one without asking would be
   *            modelling a different machine from the one most existing cases describe.
   * @note [zh] 默认关闭：标记是可选能力，不打招呼就带上它的台架，描述的是另一台机器。
   */
    void set_index_position_mech_rad(float position_mech_rad) noexcept;

    /** @brief [en] Whether this sensor has a mark at all. [zh] 这个传感器是否有标记。 */
    [[nodiscard]] auto has_index() const noexcept -> bool;

    /**
   * @brief [en] True on the tick the true mechanical angle crossed the mark.
   * @brief [zh] 真实机械角越过标记的那一个 tick 为真。
   * @note [en] An edge, not a level, and computed from the *plant* angle rather than the reported one: the mark is a
   *            property of the rotor and must not move when the offset correction does.
   * @note [zh] 是边沿而不是电平，并且由 **plant 角度** 而非报出的角度计算：标记是转子的属性，不该随偏置校正一起移动。
   */
    auto index_pulse() noexcept -> bool;

  private:
    const PmsmPlant* plant_;
    float mount_error_rad_{0.0F};
    float offset_rad_{0.0F};
    float resolution_rad_{0.0F};
    bool sensor_present_{true};
    bool velocity_supported_{true};
    bool has_index_{false};
    float index_position_rad_{0.0F};
    float previous_true_rad_{0.0F};
    bool index_primed_{false};
};

/**
 * @brief [en] DC link adapter: a stiff voltage until a `BusMeter` is attached, and then whatever the node equation says
 *             it is.
 * @brief [zh] 母线适配层：没有 `BusMeter` 时是刚性电压，挂上之后就由节点方程说了算。
 * @note [en] Attaching a meter is the difference between a bus that cannot sag and one that can. Keeping the stiff path
 *            as the default is not politeness: every number measured before this port existed was measured on a bus
 *            whose voltage was a constant, and silently giving them a sagging bus would invalidate them without saying
 *            so.
 * @note [zh] 挂不挂 meter 就是「母线不可能跌落」与「母线会跌落」的差别。把刚性那条留作默认不是客气：这个端口存在之前测
 *            的每个数字，都是在一条电压恒定的母线上测的；悄悄给它们一条会跌落的母线，等于在不说一声的情况下把它们作废。
 * @since 0.1.0
 */
class PlantDcLink final : public hal::DcLink {
  public:
    explicit PlantDcLink(float v_bus) noexcept;

    auto voltage_v() noexcept -> float override;

    void set_voltage_v(float v_bus) noexcept;

    /** @brief [en] Read the voltage from this meter instead; `nullptr` goes back to the stiff value. [zh] 改从这个 meter 读电压；`nullptr` 回到刚性值。 */
    void set_meter(BusMeter* meter) noexcept { meter_ = meter; }

    /**
     * @brief [en] Publish the current the bus node carried over the period just integrated.
     * @brief [zh] 公布刚积分完的那一个周期里母线节点流过的电流。
     * @param amps [en] period-mean current into the inverter, A.
     * @param amps [zh] 流入逆变器的周期平均电流，A。
     * @note [en] A **period mean**, not the last sub-step's value: the plant walks the period in sub-steps, so no
     *            single instant is "the" current of the period, and reporting the last one would make the number depend
     *            on the sub-step count. The mean is what the bus actually carried.
     * @note [zh] 是 **周期平均** ，不是最后一个子步的值：plant 把周期切成子步走，所以没有任何一个瞬间是这一周期的"那个"
     *            电流，报最后一个会让这个数取决于子步数。平均值才是母线实际流过的量。
     * @since 0.1.0
     */
    void set_bus_current_a(float amps) noexcept {
        bus_current_a_ = amps;
        bus_current_valid_ = true;
    }

    /**
     * @brief [en] Report "not measured" from now on, as a bench with no bus shunt would.
     * @brief [zh] 从现在起上报"没有测到"，如同没有母线分流器的台架。
     * @since 0.1.0
     */
    void set_bus_current_unavailable() noexcept {
        bus_current_a_ = 0.0F;
        bus_current_valid_ = false;
    }

    /**
     * @brief [en] The period-mean bus current, when the bench measured one.
     * @brief [zh] 台架测到过时的周期平均母线电流。
     * @param out [en] receiver, written only when the answer is true.
     * @param out [zh] 接收者，仅在返回 true 时写入。
     * @return [en] true when a measurement exists.
     * @return [zh] 存在一次测量时为 true。
     * @note [en] Not an override: `hal::DcLink` does not carry this, because a bus fed from a supply may measure no
     *            current and making it required would refuse those drives. `core::concepts::has_bus_current_v` is how a
     *            consumer asks whether this link has it.
     * @note [zh] 不是 override： `hal::DcLink` 不带这个，因为由电源供电的母线可能测不到电流，设成必需会把这类驱动拒之门
     *            外。消费者用 `core::concepts::has_bus_current_v` 问这条母线有没有。
     * @since 0.1.0
     */
    [[nodiscard]] auto read_bus_current_a(float& out) noexcept -> bool {
        if (!bus_current_valid_) {
            return false;
        }
        out = bus_current_a_;
        return true;
    }

  private:
    float v_bus_;
    BusMeter* meter_{nullptr};
    float bus_current_a_{0.0F};
    bool bus_current_valid_{false};
};

/**
 * @brief [en] Complete HalPolicy for the SIL bench: the plant replaces the hardware.
 * @brief [zh] SIL 台架用的完整 HalPolicy：plant 代替硬件。
 * @context [en] advance() owns the plant integration; the controller only sees the HAL contracts.
 * @context [zh] advance() 负责 plant 积分；控制器只看到 HAL 契约。
 * @since 0.1.0
 */
class PlantHal final {
  public:
    PlantHal(PmsmPlant& plant, float v_bus) noexcept;

    PlantHal(const PlantHal&) = delete;
    PlantHal& operator=(const PlantHal&) = delete;
    PlantHal(PlantHal&&) = delete;
    PlantHal& operator=(PlantHal&&) = delete;

    /**
   * @brief [en] Integrate the plant over one control period with the held duty.
   * @brief [zh] 用保持的占空比把 plant 积分一个控制周期。
   * @param dt_ctrl_s [en] control period in s.
   * @param dt_ctrl_s [zh] 控制周期，s。
   * @param substeps [en] integration sub-steps per control period.
   * @param substeps [zh] 每控制周期的积分子步数。
   * @post [en] The applied vector is fixed in the stationary frame over the whole period.
   * @post [zh] 整个周期内作用矢量在静止坐标系里固定。
   * @since 0.1.0
   */
    void advance(float dt_ctrl_s, int substeps) noexcept;

    /**
   * @brief [en] Reset the plant and the inverter to the power-on state.
   * @brief [zh] 把 plant 与逆变器复位到上电态。
   */
    void reset(PlantState initial) noexcept;

    /**
   * @brief [en] Model inverter dead time as a per-phase voltage loss: `v_applied = v_cmd - v_dead * sign(i_phase)`.
   * @brief [zh] 把逆变器死区建模为每相的电压损失： `v_applied = v_cmd - v_dead * sign(i_phase)`。
   * @param v_dead [en] the magnitude, in V, of `(t_dead / T_pwm) * v_bus`; 0 disables the model.
   * @param v_dead [zh] `(t_dead / T_pwm) * v_bus` 的量值，V；0 表示关闭该模型。
   * @note [en] What this model buys: without it, the PRD's U1 ("dead time cannot be verified in SIL") was a statement
   *            about the *plant*, not the library. With it, dead-time distortion becomes a measurable quantity in CI --
   *            which is the difference between "we know we do not compensate" and "we can say by how much". Honest
   *            limitation: `sign(0) = 0` here, so the zero-current neighbourhood is modelled as *better* than real
   *            hardware, where the transition is a finite slope with ringing. Numbers from this model are a lower bound
   *            on the real distortion.
   * @note [zh] 这个模型换来的是：没有它，PRD 的 U1（"死区在 SIL 里无法验证"）是一个关于 **被控对象** 的陈述，而不是关于
   *            库的。有了它，死区畸变在 CI 里成为可测量的量——这正是"我们知道自己没 补偿"与"我们能说出差多少"之间的区
   *            别。 诚实的局限：这里 `sign(0) = 0`，因此零电流附近被建模得比真实硬件 **更好** ——真实的 过渡是一段有限斜
   *            率并伴随振荡。本模型给出的数字是真实畸变的 **下界** 。
   * @since 0.1.0
   */
    void set_deadtime_voltage(float v_dead) noexcept;

    /**
   * @brief [en] Width of the smooth dead-zone transition in A; 0 keeps the ideal `sign()` model.
   * @brief [zh] 平滑死区过渡宽度，A；0 保留理想的 `sign()` 模型。
   */
    void set_deadtime_transition(float width_a) noexcept;

    [[nodiscard]] auto deadtime_voltage() const noexcept -> float;

    /**
   * @brief [en] Hand the voltage path to a switching inverter model (nullptr = the mean-value path).
   * @brief [zh] 把电压通路交给开关逆变器模型（nullptr = 平均值通路）。
   * @param inverter [en] model owned by the caller, or nullptr to restore the legacy behaviour.
   * @param inverter [zh] 由调用方持有的模型；nullptr 恢复旧行为。
   * @note [en] While a model is attached, `set_deadtime_voltage()` / `set_deadtime_transition()` are **not consulted**:
   *            the dead-time effect comes out of the switching process instead of being added to it, and having two
   *            sources would make the numbers unreadable. If the model refuses the request (control period not a whole
   *            number of switching periods), `advance()` falls back to the mean-value path rather than quietly doing
   *            nothing.
   * @note [zh] 挂上模型之后 `set_deadtime_voltage()` / `set_deadtime_transition()` **不再生效** ：死区 效应从开关过程里
   *            出来，而不是加到它上面；两个来源会让数字无法解读。 若模型拒绝该请求（控制周期不是开关周期的整数倍），
   *            `advance()` 退回平均值通路，而不是 悄悄什么都不做。
   * @since 0.1.0
   */
    void set_inverter(DiscreteInverter* inverter) noexcept;

    /**
   * @brief [en] Attach the bus meter: from here on the DC-link voltage comes from it, and the current the inverter
   *             draws is booked against it.
   * @brief [zh] 挂上母线 meter：此后直流母线电压由它给出，逆变器抽取的电流也记在它的账上。
   * @note [en] Optional and off by default. With no meter, `advance()` is byte for byte what it was -- the port adds a
   *            measurement, not a behaviour.
   * @note [zh] 可选，默认关闭。不挂 meter 时 `advance()` 与从前逐字节相同——这个端口加的是 **测量** ，不是行为。
   * @since 0.1.0
   */
    void set_bus(BusMeter* meter) noexcept;

    PlantPwm pwm;
    PlantCurrentSense current_sense;
    PlantPosition position;
    PlantDcLink dc_link;
    /**
     * @brief [en] The brake resistor's switch: what `core::BrakeChopper` writes, and what the bus node equation reads.
     * @brief [zh] 制动电阻的开关： `core::BrakeChopper` 写它，母线节点方程读它。
     * @note [en] A HAL member and not a `set_brake()` pointer, because the switch is part of the power stage rather
     *            than an optional measurement: it is always there, and with nothing writing to it the duty is 0 and the
     *            plant is byte for byte what it was before this member existed. Attaching a *meter* stays optional
     *            because that is what turns a stiff bus into an integrated one.
     * @note [zh] 是 HAL 成员而不是 `set_brake()` 指针，因为开关是功率级的一部分、不是可选的测量端口：它一直在，没有东西
     *            写它时占空比为 0，plant 与这个成员存在之前逐字节相同。而 **挂 meter** 仍然可选，因为那才是把一条刚性母
     *            线变成被积分的母线的那件事。
     * @since 0.1.0
     */
    driver::StubBrakeSwitch brake;
    /**
     * @brief [en] The power stage's fault output: quiet unless a case injects one.
     * @brief [zh] 功率级的故障输出：除非用例注入，否则安静。
     * @note [en] Required by `Motor::start()`, so the bench has to have one; `set_faulted(true)` is how a fault case
     *            makes the drive stop, and it is the same seam that F42's injection harness uses. Nothing writes it by
     *            default, so every existing SIL number is unchanged.
     * @note [zh] `Motor::start()` 要求它在，所以台架必须有一个； `set_faulted(true)` 就是故障用例让驱动停下的方式，也是
     *            F42 注入 harness 用的同一个缝。默认没有东西写它，所以既有的每个 SIL 数字都不变。
     * @since 0.1.0
     */
    driver::StubFaultInput fault_input;

  private:
    /**
     * @brief Publish the period mean of the current the node carried, and start a new period.
     */
    void publish_bus_current() noexcept {
        if ((bus_ != nullptr) && (bus_current_s_ > 0.0)) {
            dc_link.set_bus_current_a(static_cast<float>(bus_current_a_s_ / bus_current_s_));
        }
        bus_current_a_s_ = 0.0;
        bus_current_s_ = 0.0;
    }

    PmsmPlant* plant_;
    DiscreteInverter* inverter_{nullptr};
    BusMeter* bus_{nullptr};
    double bus_current_a_s_{0.0};
    double bus_current_s_{0.0};
    float v_dead_{0.0F};
    float transition_a_{/**
       * @brief [en] dead-zone transition width in A; 0 = ideal `sign()`.
       * @brief [zh] 死区过渡宽度，A；0 = 理想 `sign()`。
       */
                        0.0F};
};

// PlantPwm

inline auto PlantPwm::configure(float freq_hz, float deadtime_ns) noexcept -> ErrorCode {
    if (!(freq_hz > 0.0F) || !(deadtime_ns >= 0.0F)) {
        return ErrorCode::Param;
    }
    freq_hz_ = freq_hz;
    deadtime_ns_ = deadtime_ns;
    return ErrorCode::Ok;
}

inline void PlantPwm::write_duty(Uvw<float> duty) noexcept { duty_ = duty; }

inline auto PlantPwm::enable() noexcept -> ErrorCode {
    enabled_ = true;
    duty_ = Uvw<float>{0.5F, 0.5F, 0.5F};
    return ErrorCode::Ok;
}

inline void PlantPwm::disable() noexcept {
    enabled_ = false;
    duty_ = Uvw<float>{0.5F, 0.5F, 0.5F};
}

inline auto PlantPwm::fault() const noexcept -> bool { return false; }

inline auto PlantPwm::duty() const noexcept -> Uvw<float> { return enabled_ ? duty_ : Uvw<float>{0.5F, 0.5F, 0.5F}; }

inline auto PlantPwm::freq_hz() const noexcept -> float { return freq_hz_; }

inline auto PlantPwm::enabled() const noexcept -> bool { return enabled_; }

// PlantCurrentSense

inline PlantCurrentSense::PlantCurrentSense(const PmsmPlant& plant, const PlantPwm& pwm) noexcept
    : plant_(&plant), pwm_(&pwm) {}

inline void PlantCurrentSense::set_phase_gain(Uvw<float> gain_uvw) noexcept { gain_ = gain_uvw; }

inline void PlantCurrentSense::set_offset_a(float offset_a) noexcept { offset_a_ = offset_a; }

inline auto PlantCurrentSense::read_uvw() noexcept -> Uvw<float> {
    const Uvw<float> raw = plant_->current_uvw();
    return Uvw<float>{static_cast<float>((gain_.u * raw.u) + offset_a_),
                      static_cast<float>((gain_.v * raw.v) + offset_a_),
                      static_cast<float>((gain_.w * raw.w) + offset_a_)};
}

inline auto PlantCurrentSense::calibrate_offset() noexcept -> ErrorCode { return ErrorCode::Ok; }

inline auto PlantCurrentSense::sample_rate_hz() const noexcept -> float { return pwm_->freq_hz(); }

// PlantPosition

inline PlantPosition::PlantPosition(const PmsmPlant& plant) noexcept : plant_(&plant) {}

inline auto PlantPosition::angle_mech_rad() noexcept -> float {
    // Two different quantities live here, and mixing them up is exactly the bug
    // this model has to be able to detect: `mount_error_rad_` is how badly the
    // encoder is installed (physics, set by the test), while `offset_rad_` is the
    // correction the motor's alignment writes through the optional HAL hook. The
    // reading is truth + mounting error - correction, wrapped into [0, 2*pi).
    // 这里有两个不同的量，把它们混同正是本模型必须能检出的那个错误：`mount_error_rad_`
    // 是编码器装错 了多少（物理量，由测试设定），`offset_rad_` 是电机对齐通过可选
    // HAL 钩子写入的校正量。读数为 真值 + 安装误差 - 校正量，并归一化到 [0, 2π)。
    const float two_pi = 6.28318530718F;
    float angle = plant_->theta_mech_rad() + mount_error_rad_ - offset_rad_;
    angle = angle - (two_pi * static_cast<float>(static_cast<int>(angle / two_pi)));
    if (angle < 0.0F) {
        angle += two_pi;
    }
    // A real encoder reports counts, not radians: the loop then closes on a staircase and the true position can sit up
    // to half a count away while the reading says "on target". That is a resolution limit, not an offset -- the error it
    // contributes depends on where the target lands between two counts, which is why the budget gives it half an LSB and
    // not a fixed number.
    // 真实编码器报的是计数而不是弧度：环路随后在台阶上闭环，读数说着"到位"时真值可能还差半格。这是分辨率限制而不是
    // 偏置——它贡献的误差取决于目标落在两个计数之间的哪里，所以预算给它半个 LSB，而不是一个固定数。
    if (resolution_rad_ > 0.0F) {
        angle = resolution_rad_ * std::round(angle / resolution_rad_);
    }
    return angle;
}

inline void PlantPosition::set_resolution_rad(float lsb_rad) noexcept {
    resolution_rad_ = (lsb_rad > 0.0F) ? lsb_rad : 0.0F;
}

inline void PlantPosition::set_mount_error_rad(float error_rad) noexcept { mount_error_rad_ = error_rad; }

inline void PlantPosition::set_offset_mech_rad(float offset_rad) noexcept { offset_rad_ = offset_rad; }

inline auto PlantPosition::mech_offset_rad() const noexcept -> float { return offset_rad_; }

inline auto PlantPosition::true_angle_mech_rad() const noexcept -> float { return plant_->theta_mech_rad(); }
inline void PlantPosition::set_index_position_mech_rad(float position_mech_rad) noexcept {
    has_index_ = true;
    index_position_rad_ = position_mech_rad;
    index_primed_ = false;
}

inline auto PlantPosition::has_index() const noexcept -> bool { return has_index_; }

inline auto PlantPosition::index_pulse() noexcept -> bool {
    if (!has_index_) {
        return false;
    }
    // Edge on the truth, wrapping to one turn: the mark is physical, so it recurs every revolution and does not care what
    // the offset correction is doing.
    // 在真值上取边沿并按一圈折返：标记是物理的，每圈都会再来一次，与偏置校正在做什么无关。
    constexpr float kTurn = 6.28318530717959F;
    const float now = plant_->theta_mech_rad();
    if (!index_primed_) {
        previous_true_rad_ = now;
        index_primed_ = true;
        return false;
    }
    const float mark = index_position_rad_;
    const float before = std::fmod(previous_true_rad_ - mark + kTurn, kTurn);
    const float after = std::fmod(now - mark + kTurn, kTurn);
    previous_true_rad_ = now;
    return after < before;
}


inline auto PlantPosition::velocity_rad_s() noexcept -> float {
    // The HAL contract says a sensor may report an angle and no speed (`NaN` = not supported). No case ever drove the
    // assembly through that branch, which is why the knob below exists: with a speed the observer is not stepped at
    // all, so a sensored axis has no second opinion to compare against.
    // HAL 契约允许传感器只报角度、不报速度（`NaN` = 不支持）。此前没有任何用例把装配层开过这条分支，所以才有下面这个
    // 旋钮：有速度时观测器根本不被推进，于是带传感器的轴上没有第二意见可比。
    return velocity_supported_ ? plant_->omega_mech_rad_s() : std::nanf("");
}

inline void PlantPosition::set_velocity_supported(bool supported) noexcept { velocity_supported_ = supported; }

inline auto PlantPosition::has_sensor() const noexcept -> bool { return sensor_present_; }

inline void PlantPosition::set_sensor_present(bool present) noexcept { sensor_present_ = present; }

// PlantDcLink

inline PlantDcLink::PlantDcLink(float v_bus) noexcept : v_bus_(v_bus) {}

inline auto PlantDcLink::voltage_v() noexcept -> float { return (meter_ != nullptr) ? meter_->voltage_v() : v_bus_; }

inline void PlantDcLink::set_voltage_v(float v_bus) noexcept { v_bus_ = v_bus; }

// PlantHal

inline PlantHal::PlantHal(PmsmPlant& plant, float v_bus) noexcept
    : pwm(), current_sense(plant, pwm), position(plant), dc_link(v_bus), plant_(&plant) {}

inline void PlantHal::advance(float dt_ctrl_s, int substeps) noexcept {
    if (!(dt_ctrl_s > 0.0F) || substeps <= 0) {
        return;
    }

    const float v_bus = dc_link.voltage_v();
    const Uvw<float> duty = pwm.duty();
    // The brake resistor is the fourth current through the bus node, and it is held for the whole period just like the
    // modulation duty: the chopper decides once per control period and a switch does not change its mind mid-period.
    // Taken from the meter so that the law `duty*v/r` is written once, in `sil_bus.hpp`.
    // 制动电阻是母线节点上的第四条电流，它和调制占空比一样在整个周期内保持：斩波器每控制周期只决定一次，开关不会在周期
    // 中间改主意。从 meter 取这个律，让 `duty*v/r` 只在 `sil_bus.hpp` 里写一次。
    const double i_brake = (bus_ != nullptr) ? bus_->brake_current_a(brake.duty()) : 0.0;

    // The switching model, when one is attached: the period is walked sub-step by
    // sub-step and the plant is handed what the legs actually deliver at each
    // instant, including the ripple and the dead-time windows. See
    // sil_discrete_inverter.hpp for why this exists.
    // 挂上开关模型时：逐子步走完这个周期，把桥臂在该时刻实际交付的电压交给
    // plant，纹波与死区窗口都在里面。 存在的理由见 sil_discrete_inverter.hpp。
    if (inverter_ != nullptr) {
        const int model_steps = inverter_->begin_period(duty, v_bus, dt_ctrl_s);
        if (model_steps > 0) {
            const float h = inverter_->step_length_s();
            const auto poles = static_cast<float>(plant_->params().pole_pairs);
            for (int step = 0; step < model_steps; ++step) {
                const Ab<float> v_switched = inverter_->step(plant_->current_uvw());
                const float theta_e = poles * plant_->theta_mech_rad();
                const Dq<float> v_dq = core::park(v_switched, theta_e);
                plant_->set_voltage_dq(v_dq);
                // v and i at the **same instant**: the voltage is the one just applied and the current is the one it acts on.
                // Book after `advance()` would pair this sub-step's voltage with the next one's current, which is the
                // misalignment the whole port exists to avoid.
                // v 与 i 取 **同一时刻** ：电压是刚施加的那个，电流是它作用的那个。放在 `advance()` 之后记，会把本子步的电压与
                // 下一子步的电流配在一起，而那正是这个端口要避免的错位。
                const Dq<float> i_dq = plant_->current_dq();
                if (bus_ != nullptr) {
                    const double i_dc = bus_current_a(v_dq, i_dq, v_bus);
                    bus_->book(i_dc, i_brake, static_cast<double>(h));
                    // The same number the meter books, accumulated so the link can report the period mean. One
                    // formula, two consumers: the meter integrates it into energy, the link averages it.
                    // 与 meter 记账用的同一个数，在这里累加，于是 link 能报出周期平均。一个公式、两个消费者：meter 把它
                    // 积成能量，link 把它取平均。
                    bus_current_a_s_ += i_dc * static_cast<double>(h);
                    bus_current_s_ += static_cast<double>(h);
                }
                plant_->advance(h);
            }
            publish_bus_current();
            return;
        }
    }

    // The inverter holds one voltage vector in the stationary frame for the whole
    // period; Clarke of the phase voltages is immune to the common-mode injection
    // of SVPWM, so the duty maps back to v_ab exactly.
    Uvw<float> v_uvw{duty.u * v_bus, duty.v * v_bus, duty.w * v_bus};

    // Dead time makes the applied phase voltage depend on the sign of that
    // phase's current: during the blanking interval the phase is clamped to the
    // rail the current is coming from.
    // 死区让施加的相电压取决于该相电流的方向：在死区时间内，该相被钳位在电流来自的那条母线上。
    if (v_dead_ > 0.0F) {
        const Uvw<float> i_uvw = plant_->current_uvw();
        const auto phase_loss = [this](float i) noexcept -> float {
            // A real inverter does not switch polarity on a zero crossing; the clamp
            // at the rails is approached smoothly. `transition_a_ > 0` models that as
            // a linear ramp of width
            // +-transition, which is the unmodelled effect S13's `sign()` version
            // cannot see.
            // 真实逆变器不会在过零处瞬间换极性；钳位到母线是平滑趋近的。`transition_a_
            // > 0` 把它建模为 ±transition 宽的线性斜坡，这正是 S13 的 `sign()`
            // 版本看不见的未建模效应。
            if (transition_a_ > 0.0F) {
                const float normalized = i / transition_a_;
                return v_dead_ * ((normalized > 1.0F) ? 1.0F : ((normalized < -1.0F) ? -1.0F : normalized));
            }
            return v_dead_ * static_cast<float>((i > 0.0F) - (i < 0.0F));
        };
        v_uvw.u -= phase_loss(i_uvw.u);
        v_uvw.v -= phase_loss(i_uvw.v);
        v_uvw.w -= phase_loss(i_uvw.w);
    }

    const Ab<float> v_ab = core::clarke(v_uvw);

    const float dt_sub = dt_ctrl_s / static_cast<float>(substeps);
    const auto poles = static_cast<float>(plant_->params().pole_pairs);
    for (int i = 0; i < substeps; ++i) {
        const float theta_e = poles * plant_->theta_mech_rad();
        const Dq<float> v_dq = core::park(v_ab, theta_e);
        plant_->set_voltage_dq(v_dq);
        if (bus_ != nullptr) {
            const double i_dc = bus_current_a(v_dq, plant_->current_dq(), v_bus);
            bus_->book(i_dc, i_brake, static_cast<double>(dt_sub));
            bus_current_a_s_ += i_dc * static_cast<double>(dt_sub);
            bus_current_s_ += static_cast<double>(dt_sub);
        }
        plant_->advance(dt_sub);
    }
    publish_bus_current();
}

inline void PlantHal::set_inverter(DiscreteInverter* inverter) noexcept { inverter_ = inverter; }

inline void PlantHal::set_bus(BusMeter* meter) noexcept {
    bus_ = meter;
    dc_link.set_meter(meter);
}

inline void PlantHal::set_deadtime_voltage(float v_dead) noexcept { v_dead_ = (v_dead > 0.0F) ? v_dead : 0.0F; }

inline void PlantHal::set_deadtime_transition(float width_a) noexcept {
    transition_a_ = (width_a > 0.0F) ? width_a : 0.0F;
}

inline auto PlantHal::deadtime_voltage() const noexcept -> float { return v_dead_; }

inline void PlantHal::reset(PlantState initial) noexcept {
    plant_->reset(initial);
    pwm.disable();
    brake.disable();
}

} // namespace sil

} // namespace foc
