#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "controller/pid/pid_calculator.hpp"

using rmcs_core::controller::pid::PidCalculator;

TEST(PidCalculator, ResetClearsIntegralAndPreviousError) {
    PidCalculator pid{1., .5, 2.};
    EXPECT_DOUBLE_EQ(pid.update(2.), 2.);
    EXPECT_DOUBLE_EQ(pid.update(3.), 6.);
    pid.reset();
    EXPECT_DOUBLE_EQ(pid.update(3.), 3.);
}

TEST(PidCalculator, DisabledInputsResetBothDerivativeHistoriesAndIntegral) {
    PidCalculator pid{1., .5, 2.};
    for (const double invalid :
         {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
        pid.update(2.);
        pid.update(3.);
        EXPECT_TRUE(std::isnan(pid.update(invalid)));
        EXPECT_DOUBLE_EQ(pid.update(1.), 1.);
        pid.update(4., 20.);
        EXPECT_TRUE(std::isnan(pid.update(1., invalid)));
        EXPECT_DOUBLE_EQ(pid.update(1., -20.), 1.);
    }
}

TEST(PidCalculator, MeasurementDerivativeRejectsReferenceKickAndRespectsOutputBounds) {
    PidCalculator pid{1., 0., 2.};
    EXPECT_DOUBLE_EQ(pid.update(1., 5.), 1.);
    EXPECT_DOUBLE_EQ(pid.update(3., 5.), 3.);
    EXPECT_DOUBLE_EQ(pid.update(3., 6.), 1.);
    pid.output_min = -2.;
    pid.output_max = 2.;
    EXPECT_DOUBLE_EQ(pid.update(20., 6.), 2.);
    EXPECT_DOUBLE_EQ(pid.update(-20., 6.), -2.);
}
