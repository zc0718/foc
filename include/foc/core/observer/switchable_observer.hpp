// Conan::ImportStart
#pragma once
#include <cmath>
#include <cstdint>

#include <foc/core/machine_params.hpp>
#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/observer/blended_observer.hpp>
#include <foc/core/observer/hfi_observer.hpp>
#include <foc/core/observer/nonlinear_flux_observer.hpp>
#include <foc/core/observer/observer_policy.hpp>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] Which observer a `SwitchableObserver` forwards to.
 * @brief [zh] `SwitchableObserver` 转发给哪一个观测器。
 * @since 0.1.5
 * @exporter
 */
enum class ObserverKind : std::uint8_t {
    BackEmf = 0,       ///< [en] the back-EMF observer. [zh] 反电势观测器。
    NonlinearFlux = 1, ///< [en] the non-linear flux observer. [zh] 非线性磁链观测器。
    Hfi = 2,           ///< [en] the injection observer; the only one that injects. [zh] 注入观测器；唯一会注入的那个。
    Blended = 3,       ///< [en] injection below the band, back-EMF above it. [zh] 带下用注入，带上用反电势。
};


/**
 * @brief [en] One observer slot that holds two and forwards to the selected one.
 *
 * @brief [zh] 一个观测器槽位，持有两个并按选择转发。
 *
 *     Why it exists: `Motor` composes its observers statically, so without this every choice of observer would be
 *     another `Motor` instantiation and another object pool -- and the bridge's handles, magics and dispatch are all
 *     built for a fixed set of pools. Choosing a member instead of a type keeps one motor type for every choice, which
 *     is what makes a runtime selector possible without touching any of that. 存在的理由：`Motor` 静态组合观测器，没有
 *     它的话每次选择观测器都会变成又一次 `Motor` 实例化和另一个对象池——而 bridge 的句柄、magic 与派发都是围绕固定的池集
 *     合构造的。把选择做成成员而不是类型，所有选择就共用一个电机类型，这正是"不碰那些东西也能有运行时选择器"的前提。
 *
 * @note [en] Both observers are constructed; **only the selected one is stepped**. The other is frozen, not reset: its
 *            state at the moment it is selected again is the state it had when it was last selected. That is deliberate
 *            -- a silent reset would make a hand-over depend on which member happened to be live -- but it means a
 *            caller that wants the incoming observer to start from the outgoing one's estimate must seed it, and
 *            `seed_flux()` is how. `seed_flux()` goes to **both**, because a seed given while one is running is still
 *            the right seed for the other, and forgetting one of them is exactly the kind of omission that only shows
 *            up at a hand-over.
 * @note [zh] 两个观测器都会被构造，但**只有被选中的那个会被推进**。另一个是冻结而不是复位：它再次被选中时的状态，就是它
 *            上次被选中时的状态。这是刻意的——静默复位会让交接成败取决于当时哪个成员恰好是活的——但也意味着希望接手者从交
 *            出者的估计出发的调用方必须自己播种，而 `seed_flux()` 就是那个工具。 `seed_flux()` 会同时给**两者**：在其中
 *            一个运行时给出的种子对另一个同样成立，而漏掉一个正是那种只在交接时才会暴露的疏漏。
 * @since 0.1.5
 * @exporter
 */
class SwitchableObserver final {
  public:
    /**
     * @brief [en] Every observer's configuration in one place, plus which one to start with.
     * @brief [zh] 所有观测器的配置集中在一处，外加起始使用的那一个。
     * @note [en] One aggregate rather than a growing parameter list: adding a mode must not reshuffle a constructor
     *            that call sites already write, and a named field is harder to pass in the wrong order than a
     *            positional one.
     * @note [zh] 用一个聚合而不是不断加长的参数表：增加一种模式不应打乱调用方已经在写的构造函数，而具名字段比位置参数更
     *            难传错顺序。
     * @since 0.1.5
     */
    struct Config final {
        BackEmfObserver::Config back_emf{};
        NonlinearFluxObserver::Config nonlinear_flux{};
        HfiObserver::Config hfi{};
        BlendedObserver<HfiObserver, BackEmfObserver>::Config blended{};
        ObserverKind kind{ObserverKind::BackEmf};
    };

    /**
     * @brief [en] Three configured observers and the one to start with.
     * @brief [zh] 三个已配置的观测器，以及起始使用的那一个。
     */
    explicit SwitchableObserver(Config config) noexcept
        : back_emf_(config.back_emf), nonlinear_flux_(config.nonlinear_flux), hfi_(config.hfi),
          blended_(HfiObserver{config.hfi}, BackEmfObserver{config.back_emf}, config.blended), kind_(config.kind),
          psi_f_wb_(config.nonlinear_flux.psi_f) {}

    /**
     * @brief [en] Advance whichever observer is selected.
     * @brief [zh] 推进当前选中的观测器。
     */
    [[nodiscard]] auto update(Ab<float> v_ab, Ab<float> i_ab, float dt_s) noexcept -> ObsState {
        switch (kind_) {
        case ObserverKind::NonlinearFlux:
            return nonlinear_flux_.update(v_ab, i_ab, dt_s);
        case ObserverKind::Hfi:
            return hfi_.update(v_ab, i_ab, dt_s);
        case ObserverKind::Blended:
            return blended_.update(v_ab, i_ab, dt_s);
        case ObserverKind::BackEmf:
        default:
            return back_emf_.update(v_ab, i_ab, dt_s);
        }
    }

    /**
     * @brief [en] The voltage the selected observer wants added this period, zero for the two that do not inject.
     * @brief [zh] 被选中的观测器本周期希望叠加的电压；两个不注入的观测器返回零。
     * @return [en] The carrier when `Hfi` is selected, `{0, 0}` otherwise.
     * @return [zh] 选中 `Hfi` 时为载波，否则为 `{0, 0}`。
     * @note [en] Only the selected observer's phase advances. Asking the idle one would be a second, invisible clock
     *            running at a different rate from the one being demodulated -- the same "frozen, not stepped" rule as
     *            `update()`, and the reason this is not simply `hfi_.injection_ab()`.
     * @note [zh] 只有被选中的观测器的相位会推进。去问闲置的那个，等于让第二个看不见的时钟以与被解调的那个不同的速率走动
     *            ——与 `update()` 同一条"冻结而不推进"的规则，也是这里不写成 `hfi_.injection_ab()` 的原因。
     * @since 0.1.5
     */
    [[nodiscard]] auto injection_ab() const noexcept -> Ab<float> {
        switch (kind_) {
        case ObserverKind::Hfi:
            return hfi_.injection_ab();
        case ObserverKind::Blended:
            return blended_.injection_ab();
        case ObserverKind::BackEmf:
        case ObserverKind::NonlinearFlux:
        default:
            return Ab<float>{0.0F, 0.0F};
        }
    }

    /**
     * @brief [en] Seed every observer that takes a seed.
     * @brief [zh] 给每个接受种子的观测器播种。
     * @note [en] The injection observer is not seeded: it has no flux estimate to place, only a carrier phase.
     * @note [zh] 注入观测器不播种：它没有磁链估计要安置，只有一个载波相位。
     */
    void seed_flux(Ab<float> psi_ab) noexcept {
        back_emf_.seed_flux(psi_ab);
        nonlinear_flux_.seed_flux(psi_ab);
        blended_.seed_flux(psi_ab);
    }

    /**
     * @brief [en] What the selected observer last reported.
     * @brief [zh] 被选中的观测器上一次报出的状态。
     * @note [en] Read from the selection, not from a cached copy: the wrapper keeps no state of its own, so this cannot
     *            disagree with what `update()` returned.
     * @note [zh] 从被选中的那个读，而不是从缓存的副本读：包装器自身不存状态，因此这里不可能与 `update()` 的返回不一致。
     * @since 0.1.6
     */
    [[nodiscard]] auto state() const noexcept -> ObsState {
        switch (kind_) {
        case ObserverKind::NonlinearFlux:
            return nonlinear_flux_.state();
        case ObserverKind::Hfi:
            return hfi_.state();
        case ObserverKind::Blended:
            return blended_.state();
        case ObserverKind::BackEmf:
        default:
            return back_emf_.state();
        }
    }

    /**
     * @brief [en] Whether the *selected* observer is currently producing a measurement.
     * @brief [zh] 被选中的那个观测器当前是否在产出测量。
     * @return [en] The selected observer's own verdict where it has one: the injection observer's lock detector, the
     *              non-linear flux observer's, the blend's. `true` for the back-EMF observer, which has no detector --
     *              see the note.
     * @return [zh] 被选中的观测器自己的结论（若有）：注入观测器的锁定判据、非线性磁链观测器的、混合观测器的。反电势观测
     *              器没有判据，返回 `true`——见注释。
     * @note [en] Why the wrapper has to forward this at all: a slot is what a caller holds, and without this accessor a
     *            caller cannot tell "the injection observer never locked" from "it locked and the machine is standing
     *            still" -- both look like an angle that does not move. That is not hypothetical: an injection carrier
     *            past what the control rate can resolve leaves exactly that state, and it was found by reading a speed
     *            that had frozen rather than by any error code. `true` for an observer without a detector is the honest
     *            answer: there is no channel that could withhold a verdict, and answering `false` would make every
     *            back-EMF caller check something that does not exist.
     * @note [zh] 包装器为什么必须转发它：槽位是调用方实际持有的东西，没有这个访问器，调用方分不清"注入观测器从未锁定"与
     *            "它锁定了、而机器在静止"——两者看起来都是一个不动的角度。这不是假设：一个超出控制率分辨能力的注入载波正
     *            是留下那个状态的东西，而它是靠读到一个冻结的转速被发现的，不是靠任何错误码。没有判据的观测器返回
     *            `true` 是诚实的答案：不存在能扣住一个结论的通道，而返回 `false` 会让每个反电势调用方去检查一件不存在的
     *            事。
     * @since 0.1.7
     */
    [[nodiscard]] auto locked() const noexcept -> bool {
        switch (kind_) {
        case ObserverKind::Hfi:
            return hfi_.locked();
        case ObserverKind::Blended:
            return blended_.locked();
        case ObserverKind::NonlinearFlux:
            return nonlinear_flux_.locked();
        case ObserverKind::BackEmf:
        default:
            return true; // no detector: nothing is being withheld
        }
    }

    /**
     * @brief [en] The selected observer's normalised error signal, or `0` where that observer publishes none.
     * @brief [zh] 被选中的观测器归一化的误差信号；该观测器不发布误差信号时为 `0`。
     * @return [en] About `-sin(2*angle_error)` for the injection observer, which is the only one that has such a signal
     *              today. `0.0F` for the other three.
     * @return [zh] 注入观测器约为 `-sin(2·角度误差)`——今天只有它有这样的信号。其余三个返回 `0.0F`。
     * @warning [en] `0` is ambiguous by construction: a perfectly locked injection observer also sits near zero. A
     *               caller that needs to know whether an error signal exists has to look at `kind()`; a caller that
     *               needs to know whether it means anything has to look at `locked()`.
     * @warning [zh] `0` 按其构造就是有歧义的：一个完全锁定的注入观测器同样停在零附近。需要知道"是否存在误差信号"的调用
     *               方得去看 `kind()`；需要知道"它是否有意义"的得去看 `locked()`。
     * @since 0.1.7
     */
    [[nodiscard]] auto error() const noexcept -> float {
        switch (kind_) {
        case ObserverKind::Hfi:
            return hfi_.error();
        case ObserverKind::Blended:
        case ObserverKind::NonlinearFlux:
        case ObserverKind::BackEmf:
        default:
            return 0.0F;
        }
    }

    /**
     * @brief [en] Switch, carrying the current estimate into the incoming observer first.
     * @brief [zh] 切换，先把当前估计带进接手的那个观测器。
     * @param next [en] the observer to hand over to.
     * @param next [zh] 要交接过去的观测器。
     * @return [en] `false` if `next` was already selected, or if the incoming observer has no flux estimate to place
     *              (the injection observer has a carrier phase, not a flux vector). `true` means the incoming
     *              observer's state was written from the outgoing one's angle.
     * @return [zh] 若 `next` 已是当前选择，或接手的观测器没有磁链估计可安置（注入观测器有的是载波相位，不是磁链矢量），
     *              返回 `false`。`true` 表示接手者的状态已按交出者的角度写入。
     * @note [en] **This is the switching call a hand-over should use.** `set_kind()` exists for the case where the
     *            caller genuinely wants the incoming observer to resume from whatever it last believed; because the
     *            idle observer is frozen rather than tracking, that belief can be arbitrarily stale, and using
     *            `set_kind()` for a hand-over is how a chain ends up stepping by pi or pi/2 for no visible reason. The
     *            ordering matters and is why this is one call rather than two: seeding after the switch would write
     *            into an observer that is already being stepped, and seeding before it is a rule someone has to
     *            remember. The magnitude comes from `Config::nonlinear_flux.psi_f`, which is the machine's flux linkage
     *            -- there is no second copy of it here. A hand-over *to* the injection observer returns `false`: that
     *            observer acquires the angle itself from the current response and cannot be given one.
     * @note [zh] **交接应当使用这个调用。** `set_kind()` 保留给"调用方确实希望接手者从它上次相信的状态继续"的情形；由于
     *            闲置的观测器是**冻结**的而非在跟踪，那个"相信"可能任意陈旧，而拿 `set_kind()` 去做交接，正是一条链会毫
     *            无可见理由地跳出 π 或 π/2 的原因。顺序很关键，这也是它被做成**一个**调用而不是两个的原因：先切换再播种
     *            会写进一个已经在被推进的观测器，而先播种再切换则是一条需要有人记住的规矩。幅值取自
     *            `Config::nonlinear_flux.psi_f`，也就是这台机器的磁链——这里不存第二份。交接**给**注入观测器时返回
     *            `false`：那个观测器自己从电流响应里获取角度，无法被给予。
     * @since 0.1.6
     */
    auto hand_over_to(ObserverKind next) noexcept -> bool {
        if (next == kind_) {
            return false;
        }
        const float theta = state().theta_e;
        const Ab<float> seed{static_cast<float>(psi_f_wb_ * std::cos(theta)),
                             static_cast<float>(psi_f_wb_ * std::sin(theta))};
        switch (next) {
        case ObserverKind::NonlinearFlux:
            nonlinear_flux_.seed_flux(seed);
            break;
        case ObserverKind::BackEmf:
            back_emf_.seed_flux(seed);
            break;
        case ObserverKind::Blended:
            blended_.seed_flux(seed);
            break;
        case ObserverKind::Hfi:
        default:
            kind_ = next;
            return false; // nothing to seed: the carrier phase is not a flux estimate
        }
        kind_ = next;
        return true;
    }

    /**
     * @brief [en] Choose at run time **without** carrying the estimate across. No observer is reset and none is
     *             stepped.
     * @brief [zh] 在运行时切换，**不**把估计带过去。没有观测器被复位，也没有被推进。
     * @note [en] For a hand-over use `hand_over_to()` instead -- see the note there for why this one is a trap.
     * @note [zh] 交接请用 `hand_over_to()`——原因见那里：这一个是个陷阱。
     */
    void set_kind(ObserverKind kind) noexcept { kind_ = kind; }

    /**
     * @brief [en] The observer currently forwarded to.
     * @brief [zh] 当前转发的观测器。
     */
    [[nodiscard]] auto kind() const noexcept -> ObserverKind { return kind_; }

    /**
     * @brief [en] Replace the machine description every observer uses.
     * @brief [zh] 替换所有观测器共用的机器描述。
     */
    void apply_params(const MachineParams& params) noexcept {
        back_emf_.apply_params(params);
        nonlinear_flux_.apply_params(params);
    }

  private:
    BackEmfObserver back_emf_;
    NonlinearFluxObserver nonlinear_flux_;
    HfiObserver hfi_;
    /**
     * @brief The blend owns its own two observers rather than borrowing the members above. Borrowing would tie the band's behaviour to whatever the standalone members last believed, and the wrapper's whole rule is that only the selected observer is stepped -- so a borrowed copy would be either frozen (and useless) or stealthily advanced (and a second carrier phase). The cost is a second copy of two small objects in a statically sized slot. 交接带持有**自己**的两个观测器，而不是借用上面的成员。借用会让交接带的行为取决于那两个成员上次相信了什么，而包装器的全部规则就是"只推进被选中的那个"——借来的副本要么被冻结（因而无用），要么被偷偷推进（于是多出一个载波相位）。代价是在一个静态定容的槽位里多存两个小对象。
     */
    BlendedObserver<HfiObserver, BackEmfObserver> blended_;
    ObserverKind kind_;
    /**
     * @brief The machine's flux linkage, taken from the one field that already states it (`nonlinear_flux.psi_f`) so a hand-over has a magnitude to place the incoming observer's vector at. Not a second definition of the machine. 这台机器的磁链，取自已经写明它的那**一个**字段（`nonlinear_flux.psi_f`），于是交接有幅值可用来安置接手者的矢量。这不是对这台机器的第二个定义。
     */
    float psi_f_wb_;
};


} // namespace core

} // namespace foc
