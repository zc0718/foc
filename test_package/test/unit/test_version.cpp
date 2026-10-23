#include <cstring>

#include <gtest/gtest.h>

#include <foc/core/types.hpp>
#include <foc/version.h>
#include <foc/core/version.hpp>



TEST(Version, CTargetString) {
    EXPECT_STREQ("0.1.0", foc_version_string());
    EXPECT_STREQ("0.1.0", foc::core::version_string());
}



TEST(Version, PathsAgree) { EXPECT_STREQ(foc_version_string(), foc::core::version_string()); }



TEST(Types, DefaultConstructionIsZero) {
    const foc::core::Dq<float> dq{};
    const foc::core::Ab<float> ab{};
    const foc::core::Uvw<float> uvw{};

    EXPECT_FLOAT_EQ(0.0F, dq.d);
    EXPECT_FLOAT_EQ(0.0F, dq.q);
    EXPECT_FLOAT_EQ(0.0F, ab.alpha);
    EXPECT_FLOAT_EQ(0.0F, ab.beta);
    EXPECT_FLOAT_EQ(0.0F, uvw.u);
    EXPECT_FLOAT_EQ(0.0F, uvw.v);
    EXPECT_FLOAT_EQ(0.0F, uvw.w);
}



TEST(Types, AggregateLayoutIsTrivial) {
    static_assert(sizeof(foc::core::Dq<float>) == 2U * sizeof(float), "Dq<float> must be two floats");

    const foc::core::Dq<double> dq{1.0, -2.0};
    const foc::core::Dq<double> copy = dq;

    EXPECT_DOUBLE_EQ(1.0, copy.d);
    EXPECT_DOUBLE_EQ(-2.0, copy.q);
}
