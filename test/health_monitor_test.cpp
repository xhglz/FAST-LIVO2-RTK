#include "health_monitor.h"
#include <gtest/gtest.h>

namespace {
backend::GnssEvent fix(double time = 100.0)
{
    backend::GnssEvent event;
    event.raw_unix_time = event.measurement_time = time;
    event.valid_fix = true;
    event.fix_type = 3;
    event.carrier_solution = 2;
    event.satellites = 20;
    event.horizontal_accuracy = 0.01;
    event.vertical_accuracy = 0.02;
    return event;
}

backend::Quality quality()
{
    backend::Quality q;
    q.lidar_points = 200;
    q.lidar_matches = 100;
    q.match_ratio = 0.5;
    q.angular_coverage = 0.8;
    q.normalized_residual_rms = 1.0;
    q.lidar_information.setIdentity();
    return q;
}
} // namespace

TEST(InputDiagnostics, ReverseAndInvalidDoNotMoveTimeWatermark)
{
    health::InputMonitor monitor;
    ASSERT_TRUE(monitor.observe(health::Sensor::IMU, 10.0, 10.0, true));
    EXPECT_FALSE(monitor.observe(health::Sensor::IMU, 9.0, 10.1, true));
    EXPECT_FALSE(monitor.observe(health::Sensor::IMU, 10.0, 10.1, true));
    EXPECT_FALSE(monitor.observe(health::Sensor::IMU, 11.0, 10.1, false));
    EXPECT_FALSE(monitor.observe(health::Sensor::IMU, 4294967296.0, 10.1, true));
    EXPECT_DOUBLE_EQ(monitor.inputs[0].last_time, 10.0);
    EXPECT_EQ(monitor.inputs[0].rejected, 4u);
    EXPECT_EQ(monitor.inputs[0].counts[1], 2u);
}

TEST(InputDiagnostics, MissingDelayAndHysteresisRecovery)
{
    health::InputMonitor monitor;
    monitor.inputs[0].config.max_gap = 0.3;
    for (int i = 0; i < 8; ++i) {
        const double t = 10.0 + i * 0.2;
        monitor.observe(health::Sensor::IMU, t, t, true);
        monitor.poll(t);
    }
    EXPECT_EQ(monitor.inputs[0].health.state, health::State::NORMAL);
    for (double t : {11.8, 12.0, 12.2}) monitor.poll(t);
    EXPECT_TRUE(monitor.inputs[0].flags & health::MISSING);
    EXPECT_TRUE(monitor.inputs[0].flags & health::DELAYED);
    EXPECT_EQ(monitor.inputs[0].health.state, health::State::DEGRADED);
    for (int i = 0; i < 15; ++i) {
        const double t = 12.4 + i * 0.2;
        monitor.observe(health::Sensor::IMU, t, t, true);
        monitor.poll(t);
    }
    EXPECT_EQ(monitor.inputs[0].health.state, health::State::NORMAL);
    EXPECT_EQ(monitor.inputs[0].health.transitions, 3u);
}

TEST(InputDiagnostics, PauseAndDisabledSensorDoNotCreateFalseMissing)
{
    health::InputMonitor monitor;
    monitor.inputs[3].config.enabled = false;
    monitor.observe(health::Sensor::IMU, 10.0, 10.0, true);
    monitor.poll(10.0);
    for (int i = 0; i < 50; ++i) monitor.poll(10.0);
    EXPECT_EQ(monitor.inputs[0].health.good, 1);
    EXPECT_FALSE(monitor.inputs[0].flags & health::MISSING);
    monitor.poll(100.0);
    EXPECT_EQ(monitor.inputs[3].flags, health::NONE);
    EXPECT_EQ(monitor.inputs[3].health.state, health::State::INITIALIZING);
}

TEST(InputDiagnostics, SaturationFutureLatencyAndQueueAreDistinct)
{
    health::InputMonitor monitor;
    monitor.observe(health::Sensor::IMU, 10.0, 10.0, true, true);
    monitor.poll(10.0);
    EXPECT_TRUE(monitor.inputs[0].flags & health::SATURATED);
    monitor.observe(health::Sensor::CAMERA, 11.0, 10.0, true);
    monitor.poll(10.2);
    EXPECT_TRUE(monitor.inputs[2].flags & health::FUTURE);
    monitor.observe(health::Sensor::LIDAR, 9.0, 10.2, true);
    monitor.inputs[1].queue = 101;
    monitor.poll(10.4);
    EXPECT_TRUE(monitor.inputs[1].flags & health::DELAYED);
    EXPECT_TRUE(monitor.inputs[1].flags & health::BACKLOG);
    EXPECT_EQ(monitor.inputs[1].counts[6], 1u);
    monitor.poll(10.6);
    EXPECT_EQ(monitor.inputs[1].counts[6], 1u);
}

TEST(GnssDiagnostics, TrustRequiresConsecutiveFixAndVarianceHasFloor)
{
    health::GnssMonitor monitor;
    for (int i = 0; i < 5; ++i) {
        const double t = 100.0 + i * 0.1;
        const auto decision = monitor.observe(fix(t), t);
        EXPECT_TRUE(decision.trusted);
        EXPECT_EQ(decision.stable, i == 4);
        EXPECT_DOUBLE_EQ(decision.variance.x(), 0.05 * 0.05);
        EXPECT_DOUBLE_EQ(decision.variance.z(), 0.10 * 0.10);
    }
    EXPECT_EQ(monitor.health.state, health::State::NORMAL);
}

TEST(GnssDiagnostics, FloatUnknownInvalidSatelliteAccuracyAndAgeClasses)
{
    health::GnssMonitor monitor;
    auto event = fix();
    event.carrier_solution = 1;
    EXPECT_EQ(monitor.observe(event, 100.0).classification, "FLOAT");
    event = fix();
    event.quality_available = false;
    EXPECT_EQ(monitor.observe(event, 100.0).classification, "QUALITY_UNKNOWN");
    event = fix();
    event.valid_fix = false;
    EXPECT_EQ(monitor.observe(event, 100.0).classification, "NO_VALID_FIX");
    event = fix();
    event.satellites = 5;
    EXPECT_EQ(monitor.observe(event, 100.0).classification, "LOW_SATELLITES");
    event = fix();
    event.horizontal_accuracy = 1.0;
    EXPECT_EQ(monitor.observe(event, 100.0).classification, "POOR_ACCURACY");
    event = fix();
    EXPECT_EQ(monitor.observe(event, 102.0).classification, "STALE");
    EXPECT_EQ(monitor.observe(event, 99.0).classification, "FUTURE_TIME");
    EXPECT_EQ(monitor.observe(event, 0.0).classification, "CLOCK_UNAVAILABLE");
}

TEST(GnssDiagnostics, JumpDoesNotPoisonTemporalAnchor)
{
    health::GnssMonitor monitor;
    ASSERT_TRUE(monitor.observe(fix(), 100.0).trusted);
    auto jump = fix(100.1);
    jump.position_enu.x() = 100.0;
    const auto decision = monitor.observe(jump, 100.1);
    EXPECT_EQ(decision.classification, "POSITION_JUMP");
    EXPECT_FALSE(decision.trusted);
    EXPECT_GT(decision.innovation_score, 36.0);
    EXPECT_DOUBLE_EQ(monitor.anchor.position_enu.x(), 0.0);
    EXPECT_TRUE(monitor.observe(fix(100.2), 100.2).trusted);
}

TEST(GnssDiagnostics, DropoutForcesFreshAcquisition)
{
    health::GnssMonitor monitor;
    for (int i = 0; i < 5; ++i) monitor.observe(fix(100.0 + i * 0.1), 100.0 + i * 0.1);
    ASSERT_TRUE(monitor.last.stable);
    monitor.unavailable("MISSING");
    EXPECT_FALSE(monitor.last.stable);
    EXPECT_FALSE(monitor.observe(fix(105.0), 105.0).stable);
}

TEST(FrontendDiagnostics, ScaledInformationIsDiagnosticOnly)
{
    health::FrontendMonitor monitor;
    monitor.config.warmup_frames = 0;
    auto q = quality();
    const auto original = q.lidar_information;
    ASSERT_TRUE(monitor.assess(q));
    EXPECT_TRUE(q.lidar_information.isApprox(original));
    EXPECT_NEAR(q.scaled_information(0, 0), 0.1 * 0.1 / 100.0, 1e-12);
    EXPECT_NEAR(q.scaled_information(3, 3), 1.0 / 100.0, 1e-12);
    EXPECT_GT(q.eigen_ratio, 0.0);
    EXPECT_FALSE(q.static_filter_available);
}

TEST(FrontendDiagnostics, WeakGeometryDegradesAndRecoversWithHysteresis)
{
    health::FrontendMonitor monitor;
    monitor.config.warmup_frames = 0;
    auto q = quality();
    for (int i = 0; i < 5; ++i) monitor.assess(q);
    ASSERT_EQ(monitor.health.state, health::State::NORMAL);
    q.lidar_information.setZero();
    q.lidar_information(0, 0) = 1.0;
    EXPECT_FALSE(monitor.assess(q));
    EXPECT_EQ(monitor.health.state, health::State::NORMAL);
    monitor.assess(q);
    monitor.assess(q);
    EXPECT_EQ(monitor.health.state, health::State::DEGRADED);
    EXPECT_NE(monitor.reason.find("WEAK_GEOMETRY"), std::string::npos);
    q = quality();
    for (int i = 0; i < 5; ++i) monitor.assess(q);
    EXPECT_EQ(monitor.health.state, health::State::NORMAL);
}

TEST(FrontendDiagnostics, ResidualCoverageCorrectionAndNonfiniteAreReported)
{
    health::FrontendMonitor monitor;
    monitor.config.warmup_frames = 0;
    auto q = quality();
    q.angular_coverage = 0.01;
    q.normalized_residual_rms = 10.0;
    q.translation_correction = 2.0;
    EXPECT_FALSE(monitor.assess(q));
    EXPECT_NE(monitor.reason.find("LOW_COVERAGE"), std::string::npos);
    EXPECT_NE(monitor.reason.find("HIGH_NORMALIZED_RESIDUAL"), std::string::npos);
    EXPECT_NE(monitor.reason.find("LARGE_CORRECTION"), std::string::npos);
    health::FrontendMonitor warmup;
    q = quality();
    q.lidar_information(0, 0) = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(warmup.assess(q));
    EXPECT_NE(warmup.reason.find("NONFINITE_ESTIMATE"), std::string::npos);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
