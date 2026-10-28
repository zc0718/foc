// Conan::ImportStart
#pragma once
#include <cstdint>

#include <foc/core/machine_params.hpp>
#include <foc/core/observer/back_emf_observer.hpp>
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
     * @brief [en] Two configured observers and the one to start with.
     * @brief [zh] 两个已配置的观测器，以及起始使用的那一个。
     */
    SwitchableObserver(BackEmfObserver::Config back_emf, NonlinearFluxObserver::Config nonlinear_flux,
                       ObserverKind kind) noexcept
        : back_emf_(back_emf), nonlinear_flux_(nonlinear_flux), kind_(kind) {}

    /**
     * @brief [en] Advance whichever observer is selected.
     * @brief [zh] 推进当前选中的观测器。
     */
    [[nodiscard]] auto update(Ab<float> v_ab, Ab<float> i_ab, float dt_s) noexcept -> ObsState {
        return (kind_ == ObserverKind::BackEmf) ? back_emf_.update(v_ab, i_ab, dt_s)
                                                : nonlinear_flux_.update(v_ab, i_ab, dt_s);
    }

    /**
     * @brief [en] Seed both observers.
     * @brief [zh] 同时给两个观测器播种。
     */
    void seed_flux(Ab<float> psi_ab) noexcept {
        back_emf_.seed_flux(psi_ab);
        nonlinear_flux_.seed_flux(psi_ab);
    }

    /**
     * @brief [en] Choose at run time. Neither observer is reset.
     * @brief [zh] 在运行时切换。两个观测器都不会被复位。
     */
    void set_kind(ObserverKind kind) noexcept { kind_ = kind; }

    /**
     * @brief [en] The observer currently forwarded to.
     * @brief [zh] 当前转发的观测器。
     */
    [[nodiscard]] auto kind() const noexcept -> ObserverKind { return kind_; }

    /**
     * @brief [en] Replace the machine description both observers use.
     * @brief [zh] 替换两个观测器共用的机器描述。
     */
    void apply_params(const MachineParams& params) noexcept {
        back_emf_.apply_params(params);
        nonlinear_flux_.apply_params(params);
    }

  private:
    BackEmfObserver back_emf_;
    NonlinearFluxObserver nonlinear_flux_;
    ObserverKind kind_;
};


} // namespace core

} // namespace foc
