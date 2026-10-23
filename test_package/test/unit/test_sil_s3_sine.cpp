// S3 - sine tracking, per PRD section 8.4: omega_ref = 50*sin(2*pi*1 Hz*t).
// S3 - 正弦跟踪，对应 PRD 8.4：omega_ref = 50*sin(2*pi*1 Hz*t)。

#include <cmath>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include "sil_bench.hpp"
#include "sil_metrics.hpp"



namespace {

constexpr float kAmplitudeRadS = 50.0F;
constexpr float kSineHz = 1.0F;
constexpr float kTwoPi = 6.28318530718F;
constexpr float kRunSeconds = 2.0F;
constexpr float kSettleSeconds = 0.25F; // skip the initial transient

} // namespace



TEST(SilS3Sine, TracksTheOneHertzSineWithinFivePercent) {
    foc::sil::Bench bench(foc::sil::standard_bench_config());
    bench.motor().set_speed(0.0F);
    ASSERT_EQ(foc::ErrorCode::Ok, bench.init_and_start());
    bench.run(2e-3F);

    const int steps = static_cast<int>((kRunSeconds / bench.config().dt_ctrl_s) + 0.5F);
    const int skip = static_cast<int>((kSettleSeconds / bench.config().dt_ctrl_s) + 0.5F);

    std::vector<foc::sil::Sample> trace;
    std::vector<float> reference;
    trace.reserve(static_cast<std::size_t>(steps - skip));
    reference.reserve(static_cast<std::size_t>(steps - skip));

    float worst_error = 0.0F;
    for (int i = 0; i < steps; ++i) {
        const float t = bench.elapsed_s();
        const float ref = kAmplitudeRadS * std::sin(kTwoPi * kSineHz * t);
        bench.motor().set_speed(ref);
        bench.tick_once();

        if (i >= skip) {
            trace.push_back(bench.sample());
            reference.push_back(ref);
            const float err = std::fabs(trace.back().speed - ref);
            worst_error = (err > worst_error) ? err : worst_error;
        }
    }

    // PRD 8.4: ||err||inf <= 5 % of the amplitude.
    EXPECT_LE(worst_error, 0.05F * kAmplitudeRadS);

    // No phase/amplitude collapse: the traced speed must still swing over the full range.
    const std::vector<float> speed = foc::sil::series_of(trace, &foc::sil::Sample::speed);
    const float measured_amplitude = 0.5F * (foc::sil::max_of(speed) - foc::sil::min_of(speed));
    EXPECT_GE(measured_amplitude, 0.95F * kAmplitudeRadS);
    EXPECT_LE(measured_amplitude, 1.05F * kAmplitudeRadS);
}



TEST(SilS3Sine, DoesNotDriftOrDiverge) {
    foc::sil::Bench bench(foc::sil::standard_bench_config());
    bench.motor().set_speed(0.0F);
    ASSERT_EQ(foc::ErrorCode::Ok, bench.init_and_start());
    bench.run(2e-3F);

    const float run_seconds = 3.0F; // three full periods: head, middle and tail are distinct
    const int steps = static_cast<int>((run_seconds / bench.config().dt_ctrl_s) + 0.5F);
    std::vector<float> speed;
    speed.reserve(static_cast<std::size_t>(steps));

    for (int i = 0; i < steps; ++i) {
        bench.motor().set_speed(kAmplitudeRadS * std::sin(kTwoPi * kSineHz * bench.elapsed_s()));
        bench.tick_once();
        speed.push_back(bench.sample().speed);
    }

    // The mean over any full period must be zero: a drifting loop would not stay there.
    const std::size_t period = static_cast<std::size_t>((1.0F / (kSineHz * bench.config().dt_ctrl_s)) + 0.5F);
    ASSERT_GT(speed.size(), 2U * period) << "run shorter than two periods";

    const std::vector<float> second_period(speed.begin() + static_cast<std::ptrdiff_t>(period),
                                           speed.begin() + static_cast<std::ptrdiff_t>(2U * period));
    const std::vector<float> last_period(speed.begin() + static_cast<std::ptrdiff_t>(speed.size() - period),
                                         speed.end());

    EXPECT_NEAR(0.0F, foc::sil::tail_mean(second_period, 1.0F), 1.0F);
    EXPECT_NEAR(0.0F, foc::sil::tail_mean(last_period, 1.0F), 1.0F);
}
