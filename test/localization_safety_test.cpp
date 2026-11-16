#include "localization_safety.h"
#include <gtest/gtest.h>

namespace {
safety::Evidence evidence(double time)
{
    safety::Evidence e; e.time = time; return e;
}
void initialize(safety::Machine& machine)
{
    for (int i = 0; i < 5; ++i) machine.observe(evidence(10.0 + i * 0.1));
}
safety::NormalCloud::Ptr room(bool single_plane = false)
{
    safety::NormalCloud::Ptr cloud(new safety::NormalCloud);
    for (int plane = 0; plane < (single_plane ? 1 : 3); ++plane)
        for (int a = -6; a <= 6; ++a)
            for (int b = -6; b <= 6; ++b) {
                pcl::PointNormal p;
                p.x = plane == 0 ? 3.0 : a * 0.3;
                p.y = plane == 1 ? 3.0 : (plane == 0 ? a * 0.3 : b * 0.3);
                p.z = plane == 2 ? -2.0 : b * 0.3;
                p.normal_x = plane == 0; p.normal_y = plane == 1; p.normal_z = plane == 2;
                cloud->push_back(p);
            }
    return cloud;
}
safety::XYZCloud::Ptr xyz(const safety::NormalCloud::ConstPtr& cloud)
{
    safety::XYZCloud::Ptr out(new safety::XYZCloud);
    for (const auto& p : *cloud) out->push_back(pcl::PointXYZ(p.x, p.y, p.z));
    return out;
}
} // namespace

TEST(LocalizationState, InitializationHasNoTrustedOutputUntilVerified)
{
    safety::Machine machine;
    for (int i = 0; i < 4; ++i) {
        machine.observe(evidence(10.0 + i * 0.1));
        EXPECT_FALSE(machine.poseValid());
        EXPECT_FALSE(machine.mapWritable());
    }
    machine.observe(evidence(10.4));
    EXPECT_EQ(machine.state(), safety::State::NORMAL);
    EXPECT_TRUE(machine.poseValid());
}

TEST(LocalizationState, CameraAndGnssFailuresDoNotLoseLocalMatching)
{
    safety::Machine machine;
    initialize(machine);
    auto e = evidence(10.5); e.camera = false;
    machine.observe(e);
    EXPECT_EQ(machine.state(), safety::State::DEGRADED);
    EXPECT_TRUE(machine.poseValid());
    EXPECT_TRUE(machine.mapWritable());
    e.time = 10.6; e.camera = true; e.gnss = false;
    machine.observe(e);
    EXPECT_TRUE(machine.poseValid());
    EXPECT_EQ(machine.reason(), "GNSS_UNAVAILABLE");
}

TEST(LocalizationState, WeakGeometryImmediatelyFreezesAndThenLoses)
{
    safety::Machine machine;
    initialize(machine);
    for (int i = 0; i < 3; ++i) {
        auto e = evidence(10.5 + i * 0.1); e.geometry = false;
        machine.observe(e);
        EXPECT_FALSE(machine.poseValid());
        EXPECT_FALSE(machine.mapWritable());
    }
    EXPECT_EQ(machine.state(), safety::State::LOST);
}

TEST(LocalizationState, ShortGeometryFailureRequiresConsecutiveVerification)
{
    safety::Machine machine;
    initialize(machine);
    auto e = evidence(10.5); e.geometry = false;
    machine.observe(e);
    for (int i = 0; i < 4; ++i) {
        machine.observe(evidence(10.6 + i * 0.1));
        EXPECT_FALSE(machine.mapWritable());
    }
    machine.observe(evidence(11.0));
    EXPECT_TRUE(machine.poseValid());
}

TEST(LocalizationState, GnssOrOrdinaryLioCannotClearLost)
{
    safety::Machine machine;
    initialize(machine);
    machine.lose("TEST");
    for (int i = 0; i < 10; ++i) machine.observe(evidence(10.5 + i * 0.1));
    EXPECT_EQ(machine.state(), safety::State::LOST);
    EXPECT_FALSE(machine.poseValid());
}

TEST(LocalizationState, ShortRecoveryUsesRecoveryCountNotInitializationCount)
{
    safety::Config config;
    config.initialize_frames = 2;
    config.recover_frames = 4;
    safety::Machine machine(config);
    machine.observe(evidence(10.0));
    machine.observe(evidence(10.1));
    ASSERT_TRUE(machine.poseValid());
    auto e = evidence(10.2); e.geometry = false;
    machine.observe(e);
    for (int i = 0; i < 3; ++i) {
        machine.observe(evidence(10.3 + i * 0.1));
        EXPECT_FALSE(machine.mapWritable());
    }
    machine.observe(evidence(10.6));
    EXPECT_TRUE(machine.mapWritable());
}

TEST(LocalizationState, TrustedSubmapNeedsFiveFramesBeforeWriting)
{
    safety::Machine machine;
    initialize(machine);
    machine.lose("TEST");
    for (int i = 0; i < 5; ++i) {
        auto e = evidence(10.5 + i * 0.1); e.relocalized = true;
        machine.observe(e);
        if (i < 4) {
            EXPECT_EQ(machine.state(), safety::State::RECOVERING);
            EXPECT_FALSE(machine.poseValid());
        }
    }
    EXPECT_TRUE(machine.mapWritable());
}

TEST(LocalizationState, RecoveryFailureCannotAccumulateOldGoodFrames)
{
    safety::Machine machine;
    initialize(machine);
    machine.lose("TEST");
    auto e = evidence(10.5); e.relocalized = true;
    machine.observe(e);
    e.time = 10.6; e.geometry = false;
    machine.observe(e);
    EXPECT_EQ(machine.state(), safety::State::LOST);
    e.time = 10.7; e.geometry = true;
    machine.observe(e);
    EXPECT_FALSE(machine.mapWritable());
}

TEST(LocalizationState, ImuAndUncertaintyLimitsAreImmediate)
{
    safety::Machine imu, uncertainty;
    initialize(imu); initialize(uncertainty);
    auto e = evidence(10.5); e.imu = false;
    imu.observe(e);
    EXPECT_EQ(imu.state(), safety::State::LOST);
    e.imu = true; e.position_sigma = 0.51;
    uncertainty.observe(e);
    EXPECT_EQ(uncertainty.state(), safety::State::LOST);
}

TEST(LocalizationState, PredictionTimeAndSensorWatchdogExpireOutput)
{
    safety::Machine machine;
    initialize(machine);
    EXPECT_TRUE(machine.predictionAllowed(10.8, 0.1, 0.1));
    EXPECT_FALSE(machine.predictionAllowed(11.0, 0.1, 0.1));
    EXPECT_FALSE(machine.predictionAllowed(10.8, 0.6, 0.1));
    machine.tick(11.0, true, true);
    EXPECT_EQ(machine.state(), safety::State::LOST);
    EXPECT_FALSE(machine.poseValid());
}

TEST(LocalizationState, NewSegmentInvalidatesOldPoseAndRequiresInitialization)
{
    safety::Machine machine;
    initialize(machine);
    machine.newSegment();
    EXPECT_EQ(machine.segment(), 1u);
    EXPECT_EQ(machine.state(), safety::State::INITIALIZING);
    EXPECT_FALSE(machine.poseValid());
    machine.observe(evidence(20.0));
    EXPECT_FALSE(machine.mapWritable());
}

TEST(Relocalization, NearbyThreePlaneScenePassesQualityGates)
{
    safety::Relocalizer relocalizer;
    const auto target = room();
    relocalizer.remember(target, 10.0, V3D::Zero());
    Eigen::Isometry3d guess = Eigen::Isometry3d::Identity();
    guess.translation() = V3D(0.12, -0.1, 0.1);
    const auto result = relocalizer.align(xyz(target), guess);
    EXPECT_TRUE(result.accepted) << result.reason << " " << result.rms << " " << result.eigen_ratio;
    EXPECT_LT(result.pose.translation().norm(), 0.05);
}

TEST(Relocalization, DegeneratePlaneCannotPassWithOnlyLowResidual)
{
    safety::Relocalizer relocalizer;
    const auto plane = room(true);
    relocalizer.remember(plane, 10.0, V3D::Zero());
    const auto result = relocalizer.align(xyz(plane), Eigen::Isometry3d::Identity());
    EXPECT_FALSE(result.accepted);
    EXPECT_LT(result.eigen_ratio, 1e-5);
}

TEST(Relocalization, NoSubmapOrFarSceneCannotRecover)
{
    safety::Relocalizer relocalizer;
    const auto target = room();
    EXPECT_FALSE(relocalizer.align(xyz(target), Eigen::Isometry3d::Identity()).accepted);
    relocalizer.remember(target, 10.0, V3D::Zero());
    auto source = xyz(target);
    for (auto& p : *source) p.x += 50.0;
    EXPECT_FALSE(relocalizer.align(source, Eigen::Isometry3d::Identity()).accepted);
}

TEST(Relocalization, SnapshotAndPointHistoryAreBounded)
{
    safety::Config c;
    c.submap_frames = 3; c.submap_points = 300;
    safety::Relocalizer relocalizer(c);
    for (int i = 0; i < 10; ++i) relocalizer.remember(room(), 10.0 + i, V3D::Zero());
    EXPECT_LE(relocalizer.size(), 300u);
    relocalizer.clear();
    EXPECT_EQ(relocalizer.size(), 0u);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
