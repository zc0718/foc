// The switchable observer: does it forward honestly, does the seed reach both, and what does a switch do to state?
//
// These three questions are the ones the bridge reconnaissance said the selector depends on. The third matters most: a
// silent reset on `set_kind()` would make a hand-over depend on which member happened to be live, and the test below
// pins the actual behaviour (the idle observer is *frozen*, not reset) so that any future change to it is a decision.
//
// 可切换观测器：转发是否诚实、种子是否到达两者、切换对状态做了什么。
// 这三个问题正是 bridge 侦察指出选择器所依赖的。第三个最重要：`set_kind()` 若静默复位，交接成败就会取决于当时哪个成员恰好在
// 工作；下面的测试钉住实际行为（闲置的观测器是**冻结**而不是复位），于是将来任何改动都必须是有意的决定。

#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/observer/nonlinear_flux_observer.hpp>
#include <foc/core/observer/switchable_observer.hpp>

namespace {

constexpr float kRs = 0.5F;
constexpr float kLd = 4.0e-4F;
constexpr float kLq = 4.0e-4F;
constexpr float kPsiF = 0.0047619F;
constexpr float kDt = 5e-5F;

auto back_emf_config() -> foc::core::BackEmfObserver::Config { return {kRs, kLd}; }

auto nlfo_config() -> foc::core::NonlinearFluxObserver::Config {
    return {kRs, kLq, kPsiF, 0.3F, 3.0F, 0.5F * kPsiF};
}

/** @brief [en] A wrapper config with the given starting kind. [zh] 给定起始 kind 的包装器配置。 */
auto wrapper_config(foc::core::ObserverKind kind) -> foc::core::SwitchableObserver::Config {
    foc::core::SwitchableObserver::Config config{};
    config.back_emf = back_emf_config();
    config.nonlinear_flux = nlfo_config();
    config.kind = kind;
    return config;
}

/** @brief [en] A rotating voltage at a fixed step, so both observers see the same excitation. [zh] 固定步进的旋转电压。 */
auto input_at(int step) -> foc::core::Ab<float> {
    const float angle = 0.01F * static_cast<float>(step);
    return foc::core::Ab<float>{0.5F * std::cos(angle), 0.5F * std::sin(angle)};
}

auto current_at(int step) -> foc::core::Ab<float> {
    const float angle = 0.01F * static_cast<float>(step);
    return foc::core::Ab<float>{0.1F * std::cos(angle), 0.1F * std::sin(angle)};
}

} // namespace

TEST(SwitchableObserver, ForwardsToWhicheverObserverIsSelected) {
    // The criterion: the wrapper's output is bit-identical to a standalone observer of the selected kind driven by the
    // same inputs. Anything else would mean the wrapper is not a forwarder but a third algorithm.
    // 判据：包装器的输出与被选中的那种观测器单独驱动时**逐位相同**。否则它就成不了转发器，而是第三种算法。
    for (const foc::core::ObserverKind kind : {foc::core::ObserverKind::BackEmf, foc::core::ObserverKind::NonlinearFlux}) {
        foc::core::SwitchableObserver wrapper(wrapper_config(kind));
        wrapper.seed_flux(foc::core::Ab<float>{kPsiF, 0.0F});

        foc::core::BackEmfObserver standalone_be(back_emf_config());
        standalone_be.seed_flux(foc::core::Ab<float>{kPsiF, 0.0F});
        foc::core::NonlinearFluxObserver standalone_nlfo(nlfo_config());
        standalone_nlfo.seed_flux(foc::core::Ab<float>{kPsiF, 0.0F});

        foc::core::ObsState wrapped{};
        foc::core::ObsState direct{};
        for (int step = 0; step < 200; ++step) {
            wrapped = wrapper.update(input_at(step), current_at(step), kDt);
            direct = (kind == foc::core::ObserverKind::BackEmf)
                         ? standalone_be.update(input_at(step), current_at(step), kDt)
                         : standalone_nlfo.update(input_at(step), current_at(step), kDt);
        }
        EXPECT_FLOAT_EQ(direct.theta_e, wrapped.theta_e);
        EXPECT_FLOAT_EQ(direct.omega_e, wrapped.omega_e);
        std::printf("\n[switchable] %s: theta %.6f rad, omega %.6f rad/s after 200 steps\n",
                    (kind == foc::core::ObserverKind::BackEmf) ? "back-EMF" : "NLFO",
                    static_cast<double>(wrapped.theta_e), static_cast<double>(wrapped.omega_e));
    }
}

TEST(SwitchableObserver, TheSeedReachesBothObservers) {
    // The seed is the one thing a hand-over relies on. A seed given while one observer is running has to be there for the
    // other when it takes over, otherwise the hand-over starts from whatever that one last believed.
    // 种子是交接唯一依赖的东西。在其中一个观测器运行时给出的种子，必须在另一个接手时已经在位，否则交接就从它上次相信的东西开始。
    foc::core::SwitchableObserver wrapper(wrapper_config(foc::core::ObserverKind::BackEmf));
    // A quarter turn of flux, so a seed that silently did not arrive would be visible as the wrong angle.
    // 四分之一圈的磁链，于是"种子没到"会以角度不对的形式暴露出来。
    wrapper.seed_flux(foc::core::Ab<float>{0.0F, kPsiF});

    wrapper.set_kind(foc::core::ObserverKind::NonlinearFlux);
    // One step with no excitation: the estimate must still be the seed.
    // 一步无励磁：估计应当仍是种子。
    const foc::core::ObsState first = wrapper.update(foc::core::Ab<float>{0.0F, 0.0F}, foc::core::Ab<float>{0.0F, 0.0F}, kDt);
    std::printf("[switchable] after the seed at +90 deg, the NLFO reports theta %.4f rad\n",
                static_cast<double>(first.theta_e));
    EXPECT_NEAR(first.theta_e, 0.5F * 3.14159265F, 0.05F) << "the seed did not reach the observer that took over";
}

TEST(SwitchableObserver, TheIdleObserverIsFrozenRatherThanReset) {
    // The property that decides how a hand-over must be written, tested in the only form that can separate the two
    // hypotheses. The first version of this case could not: with the seed never changing, "frozen" and "reset" give the
    // same answer, so it proved neither. The form below moves the observer away from its seed first, then parks it:
    //
    //   seed -> run the NLFO for 100 steps (state S) -> run the back-EMF observer for 100 steps -> back to the NLFO,
    //   one zero-input step.
    //
    //   frozen hypothesis:  a NLFO seeded the same way, driven for 100 steps, then left alone for 100, then one step.
    //   reset hypothesis:   a NLFO seeded the same way, one zero-input step only.
    //
    // Matching the first and not the second is what "frozen" means, and it is the fact a hand-over has to be written
    // against: the incoming observer carries whatever it last believed, so a caller that wants it aligned with the
    // outgoing estimate must seed it explicitly.
    // 决定交接该怎么写的那条性质，用唯一能区分两个假设的形式来测。这个用例的第一版做不到：种子从不变化时，"冻结"与"复位"
    // 给出同一答案，于是两者都没证明。下面的形式先把观测器推离种子，再让它停：
    //
    //   播种 -> NLFO 跑 100 步（状态 S）-> 反电势观测器跑 100 步 -> 切回 NLFO，零输入走一步。
    //
    //   冻结假设：同样播种的 NLFO 跑 100 步，搁置 100 步，再走一步。
    //   复位假设：同样播种的 NLFO 只走一步零输入。
    //
    // 匹配前者而不是后者，才是"冻结"的含义，也才是交接必须依据的事实：接手者带着它上次相信的东西，因此想要它与交出者对齐的
    // 调用方必须显式播种。
    constexpr int kDriveSteps = 100;
    constexpr int kParkSteps = 100;

    foc::core::SwitchableObserver wrapper(wrapper_config(foc::core::ObserverKind::NonlinearFlux));
    wrapper.seed_flux(foc::core::Ab<float>{kPsiF, 0.0F});
    for (int step = 0; step < kDriveSteps; ++step) {
        static_cast<void>(wrapper.update(input_at(step), current_at(step), kDt));
    }
    wrapper.set_kind(foc::core::ObserverKind::BackEmf);
    for (int step = kDriveSteps; step < (kDriveSteps + kParkSteps); ++step) {
        static_cast<void>(wrapper.update(input_at(step), current_at(step), kDt));
    }
    wrapper.set_kind(foc::core::ObserverKind::NonlinearFlux);
    const foc::core::ObsState after_return =
        wrapper.update(foc::core::Ab<float>{0.0F, 0.0F}, foc::core::Ab<float>{0.0F, 0.0F}, kDt);

    // Frozen: driven for the first 100 steps only, then one zero-input step.
    foc::core::NonlinearFluxObserver frozen(nlfo_config());
    frozen.seed_flux(foc::core::Ab<float>{kPsiF, 0.0F});
    for (int step = 0; step < kDriveSteps; ++step) {
        static_cast<void>(frozen.update(input_at(step), current_at(step), kDt));
    }
    const foc::core::ObsState frozen_out =
        frozen.update(foc::core::Ab<float>{0.0F, 0.0F}, foc::core::Ab<float>{0.0F, 0.0F}, kDt);

    // Reset: the seed and nothing else.
    foc::core::NonlinearFluxObserver reset(nlfo_config());
    reset.seed_flux(foc::core::Ab<float>{kPsiF, 0.0F});
    const foc::core::ObsState reset_out =
        reset.update(foc::core::Ab<float>{0.0F, 0.0F}, foc::core::Ab<float>{0.0F, 0.0F}, kDt);

    std::printf("\n[switchable] returned %.6f rad | frozen %.6f rad | reset %.6f rad\n",
                static_cast<double>(after_return.theta_e), static_cast<double>(frozen_out.theta_e),
                static_cast<double>(reset_out.theta_e));
    ASSERT_GT(std::fabs(frozen_out.theta_e - reset_out.theta_e), 1e-3F)
        << "this case cannot separate the hypotheses if the seed and the driven state agree";
    EXPECT_NEAR(after_return.theta_e, frozen_out.theta_e, 1e-5F) << "the NLFO's state did not survive being parked";
    EXPECT_GT(std::fabs(after_return.theta_e - reset_out.theta_e), 1e-3F) << "the NLFO was reset when it was re-selected";
}

TEST(SwitchableObserver, AHandOverCarriesTheEstimateAndASwitchDoesNot) {
    // The distinction that a start-up chain got wrong, pinned as an A/B so it cannot come back quietly. Two wrappers are
    // driven identically with the back-EMF observer selected; at the same instant one switches with `set_kind()` and the
    // other hands over with `hand_over_to()`. The incoming observer has been idle the whole time, so:
    //   - `set_kind()` lands on whatever that idle observer last believed -- here its construction-time seed, ~0.4 rad
    //     away from where the rotor actually is by then;
    //   - `hand_over_to()` lands on the outgoing estimate, which is what a hand-over means.
    // Measured separation rather than a claim: the gap between the two outcomes is the size of the defect when the
    // obligation is skipped, and it is what a real chain showed as a 3.6 rad step at the switch.
    // 起动链路搞错的那个区别，用 A/B 钉住，免得它悄悄回来。两个包装器在选中反电势观测器的状态下被完全相同地驱动；
    // 在同一时刻一个用 `set_kind()` 切换、另一个用 `hand_over_to()` 交接。接手的观测器全程闲置，于是：
    //   - `set_kind()` 落在那个闲置观测器上次相信的地方——这里是它构造时的种子，此时离转子真实位置约 0.4 rad；
    //   - `hand_over_to()` 落在交出者的估计上，这才是交接的含义。
    // 测出来的是分离度而不是断言：跳过义务时两个结果之间的差距就是缺陷的大小，而一条真实的链把它显示为切换处 3.6 rad 的阶跃。
    constexpr int kSteps = 200;
    constexpr int kSwitch = 150;

    auto drive_until_switch = [](foc::core::SwitchableObserver& w) {
        w.seed_flux(foc::core::Ab<float>{kPsiF, 0.0F});
        for (int step = 0; step < kSwitch; ++step) {
            static_cast<void>(w.update(input_at(step), current_at(step), kDt));
        }
        return w.state().theta_e;
    };

    foc::core::SwitchableObserver switched(wrapper_config(foc::core::ObserverKind::BackEmf));
    const float before_switch = drive_until_switch(switched);
    switched.set_kind(foc::core::ObserverKind::NonlinearFlux);
    const float after_switch = switched.update(input_at(kSwitch), current_at(kSwitch), kDt).theta_e;

    foc::core::SwitchableObserver handed(wrapper_config(foc::core::ObserverKind::BackEmf));
    const float before_handover = drive_until_switch(handed);
    const bool seeded = handed.hand_over_to(foc::core::ObserverKind::NonlinearFlux);
    const float after_handover = handed.update(input_at(kSwitch), current_at(kSwitch), kDt).theta_e;

    const auto step_of = [](float a, float b) {
        float d = a - b;
        while (d <= -3.14159265F) {
            d += 6.28318531F;
        }
        while (d > 3.14159265F) {
            d -= 6.28318531F;
        }
        return std::fabs(d);
    };
    const float switch_step = step_of(after_switch, before_switch);
    const float handover_step = step_of(after_handover, before_handover);

    std::printf("\n[handover] set_kind step %.4f rad | hand_over_to step %.4f rad (seeded=%d)\n",
                static_cast<double>(switch_step), static_cast<double>(handover_step), static_cast<int>(seeded));
    EXPECT_FLOAT_EQ(before_switch, before_handover) << "the two runs must be identical up to the decision";
    EXPECT_TRUE(seeded) << "the incoming observer has a flux estimate, so it must have been seeded";
    EXPECT_LT(handover_step, 0.02F) << "the hand-over did not carry the estimate across";
    EXPECT_GT(switch_step, 0.20F) << "an unseeded switch happened to land close by; the A/B proves nothing this run";

    // A hand-over *to* the injection observer cannot be seeded: it has a carrier phase, not a flux vector. The switch
    // still happens, and the answer says so rather than pretending.
    // 交接**给**注入观测器无法播种：它有的是载波相位，不是磁链矢量。切换仍然发生，而返回值如实说明这一点。
    EXPECT_FALSE(handed.hand_over_to(foc::core::ObserverKind::Hfi));
    EXPECT_EQ(foc::core::ObserverKind::Hfi, handed.kind());
    // And handing over to what is already selected is not a hand-over.
    // 而交接给已经选中的那个不算交接。
    EXPECT_FALSE(handed.hand_over_to(foc::core::ObserverKind::Hfi));
}

TEST(SwitchableObserver, TheSelectedObserversLockVerdictIsReachableThroughTheSlot) {
    // The slot is what a caller actually holds. Without this, a caller cannot tell "the injection observer never locked"
    // from "it locked and the machine is standing still" -- both are an angle that does not move, and the speed reads
    // zero in both. That distinction is not academic: it is exactly the state a carrier past what the control rate can
    // resolve leaves behind, and the only way it was found was by reading a speed that had frozen.
    // 槽位是调用方真正持有的东西。没有它，调用方分不清"注入观测器从未锁定"与"它锁定了、而机器在静止"——两者都是一个不动的
    // 角度，转速都报零。这个区分不是学术问题：它正是超出控制率分辨能力的载波所留下的状态，而发现它的唯一方式是读到一个冻结
    // 的转速。
    foc::core::SwitchableObserver::Config config = wrapper_config(foc::core::ObserverKind::BackEmf);
    config.hfi.inject_v = 12.0F;
    config.hfi.inject_hz = 2000.0F;
    config.hfi.current_hp_hz = 300.0F;
    config.hfi.demod_lpf_hz = 200.0F;
    config.hfi.pll_bandwidth_hz = 40.0F;
    config.hfi.pll_zeta = 0.9F;
    config.hfi.demod_gain_a = foc::core::HfiObserver::predicted_gain_a(12.0F, 2000.0F, kLd, kLq + kLd);
    config.hfi.min_hf_current_a = 0.05F;
    foc::core::SwitchableObserver slot(config);

    // Back-EMF selected: no lock detector exists, so nothing is being withheld and the honest answer is `true`. The
    // error signal is `0` because that observer publishes none -- which the contract makes explicit rather than
    // pretending a zero means a perfect lock.
    // 选中反电势：不存在锁定判据，因此没有东西被扣住，诚实的答案是 `true`。误差信号为 `0`，因为那个观测器不发布误差信号
    // ——契约把这件事写明，而不是假装零意味着完美锁定。
    EXPECT_TRUE(slot.locked());
    EXPECT_FLOAT_EQ(0.0F, slot.error());

    // HFI selected, nothing but zero current fed: the modulator is off as far as this observer is concerned, so its lock
    // detector must report a hold, not a measurement. Before this accessor the slot would have answered with an angle
    // and no way to know it was a held one.
    // 选中注入观测器、只喂零电流：就这个观测器而言调制器是关着的，因此它的锁定判据必须报"保持"而不是"测量"。在这个访问器
    // 之前，槽位会给出一个角度，而没有办法知道它是保持来的。
    slot.set_kind(foc::core::ObserverKind::Hfi);
    for (int step = 0; step < 200; ++step) {
        static_cast<void>(slot.update(foc::core::Ab<float>{0.0F, 0.0F}, foc::core::Ab<float>{0.0F, 0.0F}, kDt));
    }
    EXPECT_FALSE(slot.locked()) << "an injection observer fed no current reported a lock";
    EXPECT_FLOAT_EQ(0.0F, slot.error()) << "an unlocked observer must not publish an error signal";
}

