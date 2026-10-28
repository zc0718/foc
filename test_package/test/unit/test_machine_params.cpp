#include <cmath>

#include <gtest/gtest.h>

#include <foc/core/machine_params.hpp>
#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/observer/blended_observer.hpp>
#include <foc/core/observer/hfi_observer.hpp>
#include <foc/core/traits.hpp>



namespace {

auto machine() -> foc::core::MachineParams {
    return foc::core::MachineParams{0.35F, 2e-4F, 4e-4F, 4.76e-3F};
}

auto hfi_config() -> foc::core::HfiObserver::Config {
    foc::core::HfiObserver::Config config{};
    config.inject_v = 2.0F;
    config.inject_hz = 2000.0F;
    config.current_hp_hz = 300.0F;
    config.demod_lpf_hz = 200.0F;
    config.pll_bandwidth_hz = 40.0F;
    config.pll_zeta = 0.9F;
    config.demod_gain_a = foc::core::HfiObserver::predicted_gain_a(2.0F, 2000.0F, 2e-4F, 4e-4F);
    config.min_hf_current_a = 0.05F;
    return config;
}

/**
 * @brief [en] A policy that does not model the machine at all, and therefore must not claim the hook.
 * @brief [zh] 完全不建模机器的策略，因此不应声称拥有该钩子。
 */
struct NoParams final {
    auto update(foc::core::Ab<float>, foc::core::Ab<float>, float) noexcept -> foc::core::ObsState {
        return foc::core::ObsState{};
    }
};

} // namespace

TEST(MachineParams, TheHookIsDetectedOnlyWhereItExists) {
    // Detection is the whole mechanism: a slot that cannot take parameters must not be handed any, and a
    // slot that can must not be skipped. Getting either wrong is silent -- nothing fails to compile, the
    // observer just keeps believing the nameplate.
    // 探测就是机制本身：接不了参数的槽位不该被塞参数，接得了的不该被跳过。任一处弄错都是静默的——
    // 编译不会失败，观测器只是继续相信铭牌。
    static_assert(foc::core::concepts::has_apply_params_v<foc::core::HfiObserver>,
                  "the saliency loop derives its sensitivity from the machine");
    static_assert(foc::core::concepts::has_apply_params_v<foc::core::BackEmfObserver>,
                  "the flux observer models Rs and L");
    static_assert(foc::core::concepts::has_apply_params_v<
                      foc::core::BlendedObserver<foc::core::HfiObserver, foc::core::BackEmfObserver>>,
                  "the blend must forward, or half the library stops hearing about the machine");
    static_assert(!foc::core::concepts::has_apply_params_v<NoParams>,
                  "a policy that does not model the machine has nothing to update");
    SUCCEED();
}

TEST(MachineParams, TakingNewParametersDoesNotDisturbTheRunningEstimate) {
    // The estimate must carry on from where it was: a parameter update that reset the observer would turn
    // "the machine got hotter" into "the angle is unknown again", which is a worse outcome than the drift.
    // 估计必须从原处继续：重置观测器的参数更新会把"机器变热了"变成"角度又不知道了"，那比漂移更糟。
    foc::core::HfiObserver hfi(hfi_config());
    for (int i = 0; i < 200; ++i) {
        static_cast<void>(
            hfi.update(foc::core::Ab<float>{0.0F, 0.0F}, foc::core::Ab<float>{0.1F, 0.2F}, 5e-5F));
        static_cast<void>(hfi.injection_ab());
    }
    const float before = hfi.state().theta_e;
    const float omega_before = hfi.state().omega_e;

    hfi.apply_params(machine());
    EXPECT_FLOAT_EQ(before, hfi.state().theta_e);
    EXPECT_FLOAT_EQ(omega_before, hfi.state().omega_e);
    EXPECT_EQ(foc::ErrorCode::Ok, hfi.status());

    foc::core::BackEmfObserver flux(foc::core::BackEmfObserver::Config{0.5F, 2e-4F});
    flux.seed_flux(foc::core::Ab<float>{0.01F, 0.0F});
    static_cast<void>(flux.update(foc::core::Ab<float>{0.0F, 1.0F}, foc::core::Ab<float>{0.1F, 0.2F}, 5e-5F));
    const foc::core::Ab<float> psi_before = flux.flux();
    flux.apply_params(machine());
    EXPECT_FLOAT_EQ(psi_before.alpha, flux.flux().alpha);
    EXPECT_FLOAT_EQ(psi_before.beta, flux.flux().beta);
}

TEST(MachineParams, AnUnusableUpdateIsRejectedRatherThanAbsorbed) {
    // A zero inductance is a broken measurement, and the observer must say so instead of dividing by it.
    // 零电感是一次坏掉的测量，观测器必须说出来而不是拿它做除数。
    foc::core::BackEmfObserver flux(foc::core::BackEmfObserver::Config{0.5F, 2e-4F});
    EXPECT_EQ(foc::ErrorCode::Ok, flux.status());
    flux.apply_params(foc::core::MachineParams{0.35F, 2e-4F, 0.0F, 4.76e-3F});
    EXPECT_EQ(foc::ErrorCode::Param, flux.status());

    // And a good update restores it, so the failure is clearable by fixing the number rather than by
    // rebuilding the object.
    // 而一次好的更新能把它恢复，于是故障可以通过修正数字来清除，不必重建对象。
    flux.apply_params(machine());
    EXPECT_EQ(foc::ErrorCode::Ok, flux.status());
}
