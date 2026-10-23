#include <gtest/gtest.h>

#include <foc/core/filters.hpp>
#include <foc/core/types.hpp>



TEST(FirstOrderLowPass, RejectsCutoffAboveNyquist) {
    foc::core::FirstOrderLowPass filter;

    // dt = 1e-4 s -> Nyquist is 5 kHz; 5 kHz itself is not below Nyquist.
    EXPECT_EQ(foc::ErrorCode::Param, filter.configure(5000.0F, 1e-4F));
    EXPECT_FALSE(filter.configured());
}



TEST(FirstOrderLowPass, RejectsNonPositiveArguments) {
    foc::core::FirstOrderLowPass filter;

    EXPECT_EQ(foc::ErrorCode::Param, filter.configure(100.0F, 0.0F));
    EXPECT_EQ(foc::ErrorCode::Param, filter.configure(0.0F, 1e-4F));
}



TEST(FirstOrderLowPass, PassesInputThroughWhenUnconfigured) {
    foc::core::FirstOrderLowPass filter;

    EXPECT_FLOAT_EQ(3.5F, filter.step(3.5F));
}



TEST(FirstOrderLowPass, FirstSampleSeedsTheState) {
    foc::core::FirstOrderLowPass filter;
    ASSERT_EQ(foc::ErrorCode::Ok, filter.configure(100.0F, 1e-4F));

    EXPECT_FLOAT_EQ(2.0F, filter.step(2.0F));
}



TEST(FirstOrderLowPass, ConvergesTowardsConstantInput) {
    foc::core::FirstOrderLowPass filter;
    ASSERT_EQ(foc::ErrorCode::Ok, filter.configure(200.0F, 1e-4F));

    float out = 0.0F;
    for (int i = 0; i < 2000; ++i) {
        out = filter.step(1.0F);
    }

    EXPECT_NEAR(1.0F, out, 1e-3F);
}



TEST(FirstOrderLowPass, IsMonotonicForStepInput) {
    foc::core::FirstOrderLowPass filter;
    ASSERT_EQ(foc::ErrorCode::Ok, filter.configure(100.0F, 1e-4F));

    float previous = filter.step(0.0F);
    for (int i = 0; i < 100; ++i) {
        const float out = filter.step(1.0F);
        EXPECT_GE(out, previous);
        EXPECT_LE(out, 1.0F);
        previous = out;
    }
}



TEST(FirstOrderLowPass, ResetReseedsFromNextSample) {
    foc::core::FirstOrderLowPass filter;
    ASSERT_EQ(foc::ErrorCode::Ok, filter.configure(100.0F, 1e-4F));
    static_cast<void>(filter.step(10.0F));

    filter.reset();

    EXPECT_FLOAT_EQ(0.5F, filter.step(0.5F));
}



TEST(MovingAverage, AveragesTheFirstSamples) {
    foc::core::MovingAverage<float, 4> avg;

    EXPECT_FLOAT_EQ(1.0F, avg.step(1.0F));
    EXPECT_FLOAT_EQ(1.5F, avg.step(2.0F));
    EXPECT_FLOAT_EQ(2.0F, avg.step(3.0F));
    EXPECT_FLOAT_EQ(2.5F, avg.step(4.0F));
    EXPECT_EQ(4U, avg.size());
}



TEST(MovingAverage, DropsTheOldestSampleOnceFull) {
    foc::core::MovingAverage<float, 4> avg;
    for (int i = 1; i <= 4; ++i) {
        static_cast<void>(avg.step(static_cast<float>(i)));
    }

    // window becomes 2,3,4,5
    EXPECT_FLOAT_EQ(3.5F, avg.step(5.0F));
    EXPECT_EQ(4U, avg.size());
}



TEST(MovingAverage, ResetClearsTheWindow) {
    foc::core::MovingAverage<float, 2> avg;
    static_cast<void>(avg.step(10.0F));
    static_cast<void>(avg.step(20.0F));

    avg.reset();

    EXPECT_EQ(0U, avg.size());
    EXPECT_FLOAT_EQ(5.0F, avg.step(5.0F));
}



TEST(MovingAverage, WorksWithDouble) {
    foc::core::MovingAverage<double, 3> avg;
    static_cast<void>(avg.step(1.0));
    static_cast<void>(avg.step(2.0));
    static_cast<void>(avg.step(4.0));

    // window [1,2,4] -> push 4 -> [2,4,4]
    EXPECT_NEAR(10.0 / 3.0, avg.step(4.0), 1e-12);
}
