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

#include <foc/core/machine_params.hpp>
#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/observer/blended_observer.hpp>
#include <foc/core/observer/hfi_observer.hpp>
#include <foc/core/observer/nonlinear_flux_observer.hpp>
#include <foc/core/observer/switchable_observer.hpp>

namespace {

constexpr float kRs = 0.5F;
constexpr float kLd = 4.0e-4F;
constexpr float kLq = 4.0e-4F;
constexpr float kPsiF = 0.0047619F;
constexpr float kDt = 5e-5F;

/** @brief [en] The blend the wrapper builds internally, so a standalone one can be built the same way.
 *  @brief [zh] 包装器内部构造的那个混合观测器类型，于是可以照同样的方式构造一个独立的。 */
using Blend = foc::core::BlendedObserver<foc::core::HfiObserver, foc::core::BackEmfObserver>;

auto back_emf_config() -> foc::core::BackEmfObserver::Config { return {kRs, kLd}; }

auto nlfo_config() -> foc::core::NonlinearFluxObserver::Config {
    return {kRs, kLq, kPsiF, 0.3F, 3.0F, 0.5F * kPsiF};
}

/** @brief [en] A valid carrier configuration at the 20 kHz control rate this file drives at.
 * @brief [zh] 在本文件 20 kHz 控制率下合法的载波配置。
 *
 *     The demodulation gain is passed in as `predicted_gain_a(inject_v, inject_hz, Ld, Lq + Ld)` rather than from the
 *     machine's own saliency: this file's machine has `Ld == Lq`, whose saliency is zero, and a zero gain is refused by
 *     the observer (the error normalisation would divide by it) -- so the plain formula would put the carrier in
 *     `Param` and the two injecting modes would hold at zero forever. The `+ Ld` is the same substitution the lock
 *     verdict case below already uses, and it stands for "a machine that does have saliency" -- a per-machine number,
 *     which is why the observer takes it as a field instead of deriving it.
 *
 *     解调增益按 `predicted_gain_a(inject_v, inject_hz, Ld, Lq + Ld)` 传入，而不是由机器自身的凸极算出：本文件的机器
 *     `Ld == Lq`，其凸极为零，而零增益会被观测器拒绝（误差归一化要除以它）——于是朴素公式会把载波放进 `Param`，两个注入
 *     模式就永远停在零。`+ Ld` 与下面那个锁定判据用例已在用的替换相同，它代表"一台确实有凸极的机器"——这是一个逐机器的
 *     数，也正是观测器把它作为字段而非推导它的原因。
 */
auto hfi_config() -> foc::core::HfiObserver::Config {
    foc::core::HfiObserver::Config config{};
    config.inject_v = 12.0F;
    config.inject_hz = 2000.0F;
    config.current_hp_hz = 300.0F;
    config.demod_lpf_hz = 200.0F;
    config.pll_bandwidth_hz = 40.0F;
    config.pll_zeta = 0.9F;
    config.demod_gain_a =
        foc::core::HfiObserver::predicted_gain_a(config.inject_v, config.inject_hz, kLd, kLq + kLd);
    config.min_hf_current_a = 0.05F;
    return config;
}

auto blend_config() -> Blend::Config { return Blend::Config{1.0F, 5.0F, 20.0F}; }

/** @brief [en] A wrapper config with the given starting kind. [zh] 给定起始 kind 的包装器配置。 */
auto wrapper_config(foc::core::ObserverKind kind) -> foc::core::SwitchableObserver::Config {
    foc::core::SwitchableObserver::Config config{};
    config.back_emf = back_emf_config();
    config.nonlinear_flux = nlfo_config();
    config.hfi = hfi_config();
    config.blended = blend_config();
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
    //
    // All four kinds, not the two that were easy to build. The name of this case claimed "whichever" while it drove
    // back-EMF and the NLFO only, and the two it skipped are exactly the ones with an arm of their own in every switch
    // inside the wrapper -- the carrier observer and the blend. A forwarding bug in either of those would have been a
    // bug in the only two modes the loop had nothing to say about.
    // 判据：包装器的输出与被选中的那种观测器单独驱动时**逐位相同**。否则它就成不了转发器，而是第三种算法。
    //
    // 四种而不是"好构造的两种"。这个用例的名字声称"whichever"，却只驱动了反电势与 NLFO，而它跳过的两种恰恰是在包装器内部
    // 每个 switch 里都各有一条分支的那两种——载波观测器与混合观测器。这两者中的转发错误，会正是循环无话可说的那两处的错误。
    const foc::core::Ab<float> seed{kPsiF, 0.0F};
    for (const foc::core::ObserverKind kind : {foc::core::ObserverKind::BackEmf, foc::core::ObserverKind::NonlinearFlux,
                                               foc::core::ObserverKind::Hfi, foc::core::ObserverKind::Blended}) {
        foc::core::SwitchableObserver wrapper(wrapper_config(kind));
        wrapper.seed_flux(seed);

        foc::core::BackEmfObserver standalone_be(back_emf_config());
        standalone_be.seed_flux(seed);
        foc::core::NonlinearFluxObserver standalone_nlfo(nlfo_config());
        standalone_nlfo.seed_flux(seed);
        foc::core::HfiObserver standalone_hfi(hfi_config());
        Blend standalone_blend(foc::core::HfiObserver{hfi_config()}, foc::core::BackEmfObserver{back_emf_config()},
                               blend_config());
        standalone_blend.seed_flux(seed);

        foc::core::ObsState wrapped{};
        foc::core::ObsState direct{};
        bool saw_carrier = false;
        foc::core::Ab<float> last_carried{0.0F, 0.0F};
        for (int step = 0; step < 200; ++step) {
            wrapped = wrapper.update(input_at(step), current_at(step), kDt);
            switch (kind) {
            case foc::core::ObserverKind::NonlinearFlux:
                direct = standalone_nlfo.update(input_at(step), current_at(step), kDt);
                break;
            case foc::core::ObserverKind::Hfi:
                direct = standalone_hfi.update(input_at(step), current_at(step), kDt);
                break;
            case foc::core::ObserverKind::Blended:
                direct = standalone_blend.update(input_at(step), current_at(step), kDt);
                break;
            case foc::core::ObserverKind::BackEmf:
            default:
                direct = standalone_be.update(input_at(step), current_at(step), kDt);
                break;
            }
            // The carrier has to arrive through the same door, and it is the one output that distinguishes the two
            // injecting modes from the two that do not inject at all.
            // 载波必须从同一扇门到达，而它正是把两个注入模式与两个完全不注入的模式区分开的那个输出。
            const foc::core::Ab<float> carried = wrapper.injection_ab();
            const foc::core::Ab<float> expected_carrier =
                (kind == foc::core::ObserverKind::Hfi)
                    ? standalone_hfi.injection_ab()
                    : ((kind == foc::core::ObserverKind::Blended) ? standalone_blend.injection_ab()
                                                                  : foc::core::Ab<float>{0.0F, 0.0F});
            EXPECT_FLOAT_EQ(expected_carrier.alpha, carried.alpha);
            EXPECT_FLOAT_EQ(expected_carrier.beta, carried.beta);
            saw_carrier = saw_carrier || (carried.alpha != 0.0F) || (carried.beta != 0.0F);
            last_carried = carried;
        }
        EXPECT_FLOAT_EQ(direct.theta_e, wrapped.theta_e);
        EXPECT_FLOAT_EQ(direct.omega_e, wrapped.omega_e);
        // `state()` reads from the same selection rather than from a cache, so it cannot disagree with the last update.
        // `state()` 从同一个选择里读，而不是从缓存读，因此它不可能与上一次 update 不一致。
        EXPECT_FLOAT_EQ(wrapped.theta_e, wrapper.state().theta_e);
        EXPECT_FLOAT_EQ(wrapped.omega_e, wrapper.state().omega_e);
        // Which mode is injecting is a question about the mode, not about the number that comes back. The carrier
        // observer must be putting one on the wire -- otherwise the wind-down below would be indistinguishable from
        // "this mode never injects" -- while the blend, once it has handed the estimate over to its high-speed half,
        // has wound its carrier down to exactly zero. Both are `injecting()`: a carrier of zero is not an absent
        // carrier, which is the distinction the header's `injecting()` note draws.
        //
        // The blend keeps injecting for as long as the handover takes (the weight ramps on the filtered high-speed
        // speed), which is why the claim is "seen, and gone by the end" rather than "never seen": a blend that went to
        // zero on the first step would mean the weight had jumped straight to one, and a blend that never reached zero
        // at 59 rad/s would mean the band's top end does not actually close the carrier.
        // 哪个模式在注入是关于**模式**的问题，而不是关于回来的那个数。载波观测器必须在线上放着一个载波——否则下面那个
        // 收掉就与"这个模式从不注入"无从区分——而混合一旦把估计交给高速那一半，就把载波收到恰好为零。两者都
        // `injecting()`：零载波不是不存在的载波，这正是头文件 `injecting()` 注释所指的那个区分。
        //
        // 混合在交接期间会持续注入（权重是按滤波后的高速转速爬升的），所以这里的主张是"见到过，并且到最后已经收掉"，
        // 而不是"从未见到"：第一步就归零的混合意味着权重直接跳到了 1，而在 59 rad/s 处永不归零则意味着交接带的上端
        // 并没有真正关掉载波。
        const bool injects = (kind == foc::core::ObserverKind::Hfi) || (kind == foc::core::ObserverKind::Blended);
        EXPECT_EQ(wrapper.injecting(), injects);
        if (injects) {
            EXPECT_TRUE(saw_carrier) << "an injecting mode never put a voltage on the wire";
        } else {
            EXPECT_FALSE(saw_carrier) << "a mode that cannot inject reported a carrier";
        }
        if (kind == foc::core::ObserverKind::Blended) {
            EXPECT_FLOAT_EQ(last_carried.alpha, 0.0F) << "the blend's carrier must be wound down once it is handed over";
            EXPECT_FLOAT_EQ(last_carried.beta, 0.0F) << "the blend's carrier must be wound down once it is handed over";
        }
        std::printf("\n[switchable] %s: theta %.6f rad, omega %.6f rad/s after 200 steps, carrier seen %d\n",
                    (kind == foc::core::ObserverKind::BackEmf)         ? "back-EMF"
                    : (kind == foc::core::ObserverKind::NonlinearFlux) ? "NLFO"
                    : (kind == foc::core::ObserverKind::Hfi)           ? "HFI"
                                                                       : "blend",
                    static_cast<double>(wrapped.theta_e), static_cast<double>(wrapped.omega_e),
                    static_cast<int>(saw_carrier));
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

TEST(SwitchableObserver, AHandOverToTheBlendCarriesTheEstimateLikeTheOthers) {
    // The third arm of `hand_over_to()`. The existing A/B case hands over to the NLFO and shows that a hand-over to
    // the carrier observer is refused; the blend is the arm nobody drove, and it is the one that is easiest to get
    // wrong, because the blend takes a seed by forwarding it to its *high-speed* half -- so "it returned true" does
    // not by itself mean the estimate went anywhere useful.
    //
    // The observable is the one the wrapper's own rule allows: a blend built independently and seeded with the vector
    // the hand-over is supposed to have written -- `psi_f` at the outgoing angle -- must then answer bit-identically,
    // step for step. Equality of the two runs is the claim; the reference is the hand-over written out by hand.
    // `psi_f` is `Config::nonlinear_flux.psi_f`, the wrapper's one copy of the machine's flux linkage.
    //
    // `hand_over_to()` 的第三条分支。已有的 A/B 用例交接给 NLFO、并表明交接**给**载波观测器会被拒绝；混合是没人驱动过的
    // 那条分支，而它恰恰是最容易搞错的那条，因为混合是**转发**种子给它的高速那一半——所以"返回了 true"本身并不等于估计
    // 去了什么有用的地方。
    //
    // 观测量是包装器自己的规则允许的那一个：独立构造、并按交接本应写入的那个矢量（交出角处的 `psi_f`）播种的混合观测器，
    // 此后必须**逐步逐位**相同。两个运行相等就是主张；参照物是手写出来的那次交接。`psi_f` 取自
    // `Config::nonlinear_flux.psi_f`，也就是包装器唯一的那个机器磁链副本。
    constexpr int kSteps = 200;
    constexpr int kSwitch = 150;

    foc::core::SwitchableObserver handed(wrapper_config(foc::core::ObserverKind::BackEmf));
    handed.seed_flux(foc::core::Ab<float>{kPsiF, 0.0F});
    for (int step = 0; step < kSwitch; ++step) {
        static_cast<void>(handed.update(input_at(step), current_at(step), kDt));
    }
    const float outgoing = handed.state().theta_e;
    ASSERT_TRUE(handed.hand_over_to(foc::core::ObserverKind::Blended))
        << "the blend has a flux estimate to place, so it must have been seeded";
    ASSERT_EQ(foc::core::ObserverKind::Blended, handed.kind());

    const foc::core::Ab<float> written_by_hand{static_cast<float>(kPsiF * std::cos(outgoing)),
                                               static_cast<float>(kPsiF * std::sin(outgoing))};
    Blend reference(foc::core::HfiObserver{hfi_config()}, foc::core::BackEmfObserver{back_emf_config()},
                    blend_config());
    reference.seed_flux(written_by_hand);

    for (int step = kSwitch; step < kSteps; ++step) {
        const foc::core::ObsState from_slot = handed.update(input_at(step), current_at(step), kDt);
        const foc::core::ObsState from_reference = reference.update(input_at(step), current_at(step), kDt);
        ASSERT_FLOAT_EQ(from_reference.theta_e, from_slot.theta_e)
            << "the hand-over did not write the outgoing estimate into the blend, step " << step;
        ASSERT_FLOAT_EQ(from_reference.omega_e, from_slot.omega_e) << "step " << step;
    }
    std::printf("\n[switchable] hand-over to the blend from theta %.4f rad: %d steps bit-identical\n",
                static_cast<double>(outgoing), kSteps - kSwitch);
}

TEST(SwitchableObserver, ApplyParamsIsEquivalentToConstructingWithThoseParams) {
    // `apply_params()` is how a caller tells an observer the machine changed under it (identification finishing, a
    // re-flashed parameter set). It had never been called by any case, so nothing in the suite would have noticed if
    // it silently did nothing -- and "silently does nothing" is the exact failure mode of a forwarding method that
    // forgets one of its members.
    //
    // The claim, and its falsifier, in one run: an observer told the new machine must answer exactly like one built
    // with it, and an observer *not* told must not -- otherwise the equality proves only that both runs were the same
    // object. The observers are re-seeded after the reconfiguration, because a seed is a state and the point here is
    // the parameters, not the history.
    // `apply_params()` 是调用方告诉观测器"机器在它脚下变了"的方式（辨识刚结束、参数集重新烧写）。它从未被任何用例调用
    // 过，所以若它悄悄什么都不做，整个套件里没有一个地方会注意到——而"悄悄什么都不做"正是一个忘了转发某个成员的转发方法
    // 的确切失效方式。
    //
    // 一次运行里同时给出主张与它的伪证：被告知新机器的观测器必须和用新机器构造的**逐位相同**，而**没**被告知的必须不同
    // ——否则这个相等只证明了两次运行是同一个对象。重配置之后重新播种，因为种子是状态，而这里的要点是参数而非历史。
    constexpr int kSteps = 200;
    const foc::core::MachineParams machine{kRs, kLq, kLq, kPsiF};
    const foc::core::Ab<float> seed{kPsiF, 0.0F};

    for (const foc::core::ObserverKind kind :
         {foc::core::ObserverKind::BackEmf, foc::core::ObserverKind::NonlinearFlux}) {
        foc::core::SwitchableObserver::Config wrong = wrapper_config(kind);
        wrong.back_emf = {2.0F * kRs, kLq};
        wrong.nonlinear_flux = {2.0F * kRs, kLq, kPsiF, 0.3F, 3.0F, 0.5F * kPsiF};

        foc::core::SwitchableObserver told(wrong);
        told.apply_params(machine);
        told.seed_flux(seed);

        foc::core::SwitchableObserver built(wrapper_config(kind));
        built.seed_flux(seed);

        foc::core::SwitchableObserver not_told(wrong);
        not_told.seed_flux(seed);

        for (int step = 0; step < kSteps; ++step) {
            const foc::core::ObsState a = told.update(input_at(step), current_at(step), kDt);
            const foc::core::ObsState b = built.update(input_at(step), current_at(step), kDt);
            static_cast<void>(not_told.update(input_at(step), current_at(step), kDt));
            ASSERT_FLOAT_EQ(b.theta_e, a.theta_e) << "apply_params did not reproduce the constructor, step " << step;
            ASSERT_FLOAT_EQ(b.omega_e, a.omega_e) << "step " << step;
        }

        const float taken = told.state().omega_e;
        const float ignored = not_told.state().omega_e;
        std::printf("\n[switchable] apply_params %s: omega %.6f rad/s as told, %.6f rad/s if ignored (%.3e apart)\n",
                    (kind == foc::core::ObserverKind::BackEmf) ? "back-EMF" : "NLFO", static_cast<double>(taken),
                    static_cast<double>(ignored), static_cast<double>(std::fabs(taken - ignored)));
        EXPECT_GT(std::fabs(taken - ignored), 1.0e-3F * std::fabs(taken) + 1.0e-4F)
            << "doubling Rs changed nothing, so this case cannot tell a working apply_params from an empty one";
    }
}

