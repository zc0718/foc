#include <limits>

#include <gtest/gtest.h>

#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/observer/ideal_observer.hpp>
#include <foc/core/observer/observer_policy.hpp>
#include <foc/core/traits.hpp>
#include <foc/core/types.hpp>



static_assert(foc::core::concepts::is_observer_v<foc::core::IdealObserver>,
              "IdealObserver must satisfy the observer policy contract");



TEST(IdealObserver, ReportsTheTruthExactly) {
    foc::core::IdealObserver observer;
    observer.set_truth(1.234F, 78.5F);

    const foc::core::ObsState state =
        observer.update(foc::core::Ab<float>{99.0F, -99.0F}, foc::core::Ab<float>{1.0F, 1.0F}, 1e-4F);

    // "R.E." = reported exactly: the cheat channel must not filter, scale or delay anything.
    EXPECT_FLOAT_EQ(1.234F, state.theta_e);
    EXPECT_FLOAT_EQ(78.5F, state.omega_e);
}



TEST(IdealObserver, ResetClearsTheTruth) {
    foc::core::IdealObserver observer;
    observer.set_truth(1.0F, 2.0F);

    observer.reset();

    const foc::core::ObsState state = observer.update(foc::core::Ab<float>{}, foc::core::Ab<float>{}, 1e-4F);
    EXPECT_FLOAT_EQ(0.0F, state.theta_e);
    EXPECT_FLOAT_EQ(0.0F, state.omega_e);
}



TEST(IdealObserver, IgnoresElectricalInputs) {
    foc::core::IdealObserver observer;
    observer.set_truth(0.5F, 10.0F);

    const foc::core::ObsState a =
        observer.update(foc::core::Ab<float>{1.0F, 2.0F}, foc::core::Ab<float>{3.0F, 4.0F}, 1e-4F);
    const foc::core::ObsState b =
        observer.update(foc::core::Ab<float>{-1.0F, -2.0F}, foc::core::Ab<float>{-3.0F, -4.0F}, 5e-5F);

    EXPECT_FLOAT_EQ(a.theta_e, b.theta_e);
    EXPECT_FLOAT_EQ(a.omega_e, b.omega_e);
}

TEST(IdealObserver, TheCheatChannelCannotQuietlyBecomeAnEstimator) {
    // The row this belongs to stays L2, and it stays L2 for a reason worth pinning rather than a reason worth
    // repeating: L3 asks for a sensitivity to an error source, and this class has no model to be wrong -- it reports
    // the plant's truth. What *can* go wrong is that it stops being a cheat channel: a filter, a one-period delay or
    // a plausible-looking "improvement" that reads the electrical inputs would make every comparison that uses it a
    // comparison between two estimators instead of against the truth. So the three properties that make it usable as
    // a reference are asserted against inputs chosen to break them.
    // 这一行留在 L2，而它留在 L2 的理由值得钉住而不是反复陈述：L3 要求对某个误差源有敏感度，而本类没有可错的模型——
    // 它直接报告被控对象的真值。真正可能出问题的是它**不再是一个作弊通道**：一个滤波、一个周期延迟，或一个看起来
    // 合理的"改进"去读电气输入，都会让所有以它为参照的比较变成两个估计器之间的比较。所以此处用专门挑来破坏这三个
    // 性质 的输入，断言让它可当参照的三条性质。
    foc::core::IdealObserver observer;
    observer.set_truth(2.5F, -1400.0F);

    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    struct Input final {
        foc::core::Ab<float> v_ab;
        foc::core::Ab<float> i_ab;
        float dt;
    };
    // Garbage that would destroy any model-based observer: NaN, infinities, zero and negative periods, and a
    // voltage far outside any bus.
    // 能把任何基于模型的观测器毁掉的垃圾输入：NaN、无穷、零与负周期，以及远超任何母线电压的值。
    const Input inputs[] = {{{nan, nan}, {nan, nan}, nan},
                            {{inf, -inf}, {inf, -inf}, inf},
                            {{1.0e9F, -1.0e9F}, {1.0e4F, -1.0e4F}, 0.0F},
                            {{0.0F, 0.0F}, {0.0F, 0.0F}, -1.0e-3F}};
    for (const Input& input : inputs) {
        const foc::core::ObsState state = observer.update(input.v_ab, input.i_ab, input.dt);
        EXPECT_FLOAT_EQ(2.5F, state.theta_e) << "dt " << input.dt << ": the truth was filtered or delayed";
        EXPECT_FLOAT_EQ(-1400.0F, state.omega_e) << "dt " << input.dt << ": the truth was filtered or delayed";
    }

    // Zero lag: the value set this period is the value reported this period, which is what makes it a reference.
    // 零延迟：本周期设的值本周期就报出来，这正是它能当参照的原因。
    observer.set_truth(-0.75F, 12.0F);
    const foc::core::ObsState same_period =
        observer.update(foc::core::Ab<float>{}, foc::core::Ab<float>{}, 1.0e-4F);
    EXPECT_FLOAT_EQ(-0.75F, same_period.theta_e);
    EXPECT_FLOAT_EQ(12.0F, same_period.omega_e);

    // The falsifier: a model-based observer given the same two very different voltages answers differently, so
    // "the output does not move with the inputs" is a property of this class and not of the interface.
    // 反证：一个基于模型的观测器面对同样两个截然不同的电压会给出不同答案，所以"输出不随输入变化"是本类的性质，
    // 而不是接口的性质。
    foc::core::BackEmfObserver real_observer({0.5F, 2.0e-3F});
    real_observer.reset();
    const foc::core::ObsState first =
        real_observer.update(foc::core::Ab<float>{12.0F, 0.0F}, foc::core::Ab<float>{0.0F, 0.0F}, 1.0e-4F);
    real_observer.reset();
    const foc::core::ObsState second =
        real_observer.update(foc::core::Ab<float>{-12.0F, 0.0F}, foc::core::Ab<float>{0.0F, 0.0F}, 1.0e-4F);
    EXPECT_NE(first.theta_e, second.theta_e) << "the comparator does not react to voltage, so the case above proves "
                                                "nothing about the cheat channel";
}
