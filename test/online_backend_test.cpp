#include "online_backend.h"
#include <gtest/gtest.h>
#include <set>

namespace {
online::Config config()
{
    online::Config c;
    c.gnss.acquire_frames = 1;
    c.init_min_pairs = 4;
    c.init_baseline = 2.0;
    c.init_min_duration = 1.5;
    c.lever = Eigen::Vector3d(0.4, 0.1, 0.2);
    return c;
}
gtsam::Pose3 alignment()
{
    return gtsam::Pose3(gtsam::Rot3::Rz(0.4), Eigen::Vector3d(10, 20, 3));
}
backend::KeyframeEvent frame(int sequence, bool stationary = false)
{
    backend::KeyframeEvent event;
    event.sequence = sequence;
    event.measurement_time = 100.0 + sequence * 0.5;
    event.T_odom_imu.translation() = Eigen::Vector3d(stationary ? 0.0 : sequence * 0.5, 0, 0);
    event.gravity_odom = Eigen::Vector3d(0, 0, -9.81);
    event.quality.lidar_matches = 100;
    event.quality.eigen_ratio = 0.01;
    event.quality.normalized_residual_rms = 1.0;
    PointCloudXYZRGB::Ptr cloud(new PointCloudXYZRGB());
    PointTypeRGB point;
    point.x = 1; point.y = 2; point.z = 3;
    cloud->push_back(point);
    event.cloud_imu = cloud;
    return event;
}
backend::GnssEvent fix(const backend::KeyframeEvent& f, const online::Config& c,
    const gtsam::Pose3& T = alignment())
{
    backend::GnssEvent event;
    event.sequence = f.sequence;
    event.measurement_time = event.raw_unix_time = f.measurement_time;
    event.position_enu = T.compose(gtsam::Pose3(f.T_odom_imu.matrix())).transformFrom(c.lever);
    event.valid_fix = event.trusted_fix = event.stable_fix = true;
    event.fix_type = 3; event.carrier_solution = 2; event.satellites = 20;
    event.horizontal_accuracy = 0.02; event.vertical_accuracy = 0.04;
    event.observation_age = 0.1;
    return event;
}
void initialize(online::Core& core, const online::Config& c)
{
    for (int i = 0; i < 12; ++i) {
        auto f = frame(i);
        core.addKeyframe(f);
        core.addGnss(fix(f, c));
    }
}
} // namespace

TEST(OnlineFactor, AntennaLeverAndRealTimestampInterpolation)
{
    const gtsam::Pose3 a(gtsam::Rot3::Rz(0.2), Eigen::Vector3d(1, 2, 3));
    const gtsam::Pose3 b(gtsam::Rot3::Rz(0.5), Eigen::Vector3d(2, 4, 3));
    const Eigen::Vector3d lever(0.4, 0.2, 0.1);
    const Eigen::Vector3d measured = online::interpolatePose(a, b, 0.25).transformFrom(lever);
    online::AntennaFactor factor(0, 1, 0.25, lever, measured,
        gtsam::noiseModel::Isotropic::Sigma(3, 0.1));
    gtsam::Matrix H1, H2;
    EXPECT_LT(factor.evaluateError(a, b, H1, H2).norm(), 1e-10);
    EXPECT_EQ(H1.rows(), 3); EXPECT_EQ(H1.cols(), 6);
    EXPECT_TRUE(H1.allFinite() && H2.allFinite());
    EXPECT_GT(H1.leftCols(3).norm(), 0.01);
    EXPECT_NEAR(H1.rightCols(3).norm(), 0.75 * std::sqrt(3.0), 1e-6);
    EXPECT_NEAR(H2.rightCols(3).norm(), 0.25 * std::sqrt(3.0), 1e-6);
}

TEST(OnlineSelection, UsesMotionTimeAndQuality)
{
    const auto c = config();
    auto f = frame(1);
    EXPECT_TRUE(online::selectable(f, c));
    EXPECT_TRUE(online::shouldSelect(gtsam::Pose3(), 100.0, f, c));
    f.measurement_time = 100.01;
    EXPECT_FALSE(online::shouldSelect(gtsam::Pose3(), 100.0, f, c));
    f.quality.lidar_update_accepted = false;
    EXPECT_FALSE(online::selectable(f, c));
    f.quality.lidar_update_accepted = true;
    f.quality.eigen_ratio = 0;
    EXPECT_FALSE(online::selectable(f, c));
}

TEST(OnlineInitialization, StationaryAndMissingGravityWait)
{
    const auto c = config();
    online::Core stationary(c), no_gravity(c);
    for (int i = 0; i < 12; ++i) {
        auto f = frame(i, true);
        stationary.addKeyframe(f);
        stationary.addGnss(fix(f, c));
        f = frame(i);
        f.gravity_odom.setZero();
        no_gravity.addKeyframe(f);
        no_gravity.addGnss(fix(f, c));
    }
    EXPECT_FALSE(stationary.ready());
    EXPECT_FALSE(no_gravity.ready());
    EXPECT_EQ(no_gravity.reason(), "WAITING_FOR_GRAVITY");
    EXPECT_EQ(stationary.gnssFactors(), 0u);
}

TEST(OnlineInitialization, GravityYawTranslationAndRawPoses)
{
    const auto c = config();
    online::Core core(c);
    const Eigen::Matrix3d tilt = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()).toRotationMatrix();
    const gtsam::Pose3 T(gtsam::Rot3(alignment().rotation().matrix() * tilt), alignment().translation());
    for (int i = 0; i < 12; ++i) {
        auto f = frame(i);
        f.T_odom_imu.linear() = tilt.transpose();
        f.T_odom_imu.translation() = tilt.transpose() * f.T_odom_imu.translation();
        f.gravity_odom = tilt.transpose() * Eigen::Vector3d(0, 0, -9.81);
        core.addKeyframe(f);
        core.addGnss(fix(f, c, T));
    }
    ASSERT_TRUE(core.ready());
    EXPECT_TRUE(core.correction().equals(T, 1e-5));
    for (const auto& node : core.nodes()) {
        const Eigen::Vector3d expected = tilt.transpose() * Eigen::Vector3d(node.sequence * 0.5, 0, 0);
        EXPECT_TRUE(node.raw.translation().isApprox(expected, 1e-9));
        EXPECT_TRUE(node.optimized.equals(T.compose(node.raw), 1e-5));
    }
}

TEST(OnlineGnss, QualityAndDuplicateRejectedAndOneFactorPerMeasurement)
{
    const auto c = config();
    online::Core core(c);
    initialize(core, c);
    ASSERT_TRUE(core.ready());
    EXPECT_EQ(core.gnssFactors(), 12u);
    const std::set<double> timestamps(core.factorTimes().begin(), core.factorTimes().end());
    EXPECT_EQ(timestamps.size(), core.gnssFactors());
    auto bad = fix(frame(12), c);
    bad.carrier_solution = 1;
    EXPECT_FALSE(core.addGnss(bad));
    EXPECT_FALSE(core.addGnss(fix(frame(11), c)));
    EXPECT_EQ(core.gnssFactors(), 12u);
}

TEST(OnlineInitialization, QualityBreakDiscardsPreviousFixSegment)
{
    const auto c = config();
    online::Core core(c);
    for (int i = 0; i < 3; ++i) {
        core.addKeyframe(frame(i));
        core.addGnss(fix(frame(i), c));
    }
    ASSERT_FALSE(core.ready());
    ASSERT_EQ(core.pending(), 3u);
    core.addKeyframe(frame(3));
    auto invalid = fix(frame(3), c);
    invalid.carrier_solution = 1;
    EXPECT_FALSE(core.addGnss(invalid));
    EXPECT_EQ(core.pending(), 0u);
    for (int i = 4; i < 8; ++i) {
        core.addKeyframe(frame(i));
        core.addGnss(fix(frame(i), c));
        EXPECT_FALSE(core.ready());
    }
    core.addKeyframe(frame(8));
    core.addGnss(fix(frame(8), c));
    ASSERT_TRUE(core.ready());
    ASSERT_EQ(core.gnssFactors(), 5u);
    EXPECT_DOUBLE_EQ(core.factorTimes().front(), frame(4).measurement_time);
}

TEST(OnlineGnss, DelayedMeasurementWaitsForBracketAndNeverDuplicates)
{
    const auto c = config();
    online::Core core(c);
    initialize(core, c);
    const std::size_t count = core.gnssFactors();
    auto f = frame(12);
    f.measurement_time -= 0.25;
    f.T_odom_imu.translation().x() -= 0.25;
    auto gps = fix(f, c);
    core.addGnss(gps);
    EXPECT_EQ(core.gnssFactors(), count);
    core.addKeyframe(frame(12));
    EXPECT_EQ(core.gnssFactors(), count + 1);
    EXPECT_DOUBLE_EQ(core.factorTimes().back(), gps.measurement_time);
    EXPECT_FALSE(core.addGnss(gps));
    EXPECT_EQ(core.gnssFactors(), count + 1);
}

TEST(OnlineGnss, TrustedButWrongPositionFailsInnovationWithoutMovingLocalPose)
{
    auto c = config();
    online::Core core(c);
    initialize(core, c);
    ASSERT_TRUE(core.ready());
    const auto old_correction = core.correction();
    const auto count = core.gnssFactors();
    const auto f = frame(12);
    core.addKeyframe(f);
    auto bad = fix(f, c);
    bad.position_enu.x() += 100.0;
    core.addGnss(bad);
    EXPECT_EQ(core.gnssFactors(), count);
    EXPECT_EQ(core.reason(), "GNSS_INNOVATION_REJECTED");
    EXPECT_FALSE(core.gnssLive());
    EXPECT_TRUE(core.correction().equals(old_correction, 1e-5));
    EXPECT_TRUE(core.nodes().back().raw.translation().isApprox(f.T_odom_imu.translation()));
}

TEST(OnlineGnss, LossKeepsOdometryFactorsAndRequiresFreshConsecutiveFix)
{
    auto c = config();
    c.gnss.acquire_frames = 3;
    online::Core core(c);
    initialize(core, c);
    ASSERT_TRUE(core.ready());
    const auto factors = core.gnssFactors(), updates = core.updates();
    core.tick(115.0);
    EXPECT_FALSE(core.gnssLive());
    for (int i = 30; i < 33; ++i) {
        core.addKeyframe(frame(i));
        core.addGnss(fix(frame(i), c));
        if (i < 32) EXPECT_EQ(core.gnssFactors(), factors);
    }
    EXPECT_GT(core.updates(), updates);
    EXPECT_EQ(core.gnssFactors(), factors + 1);
    EXPECT_TRUE(core.gnssLive());
}

TEST(OnlineGnss, ReacquisitionNeverReportsLiveFixEarly)
{
    const auto c = config();
    online::Core core(c);
    initialize(core, c);
    ASSERT_TRUE(core.gnssLive());
    core.addKeyframe(frame(12));
    auto event = fix(frame(12), c);
    event.stable_fix = false;
    EXPECT_FALSE(core.addGnss(event));
    EXPECT_FALSE(core.gnssLive());
    EXPECT_EQ(core.reason(), "GNSS_REACQUIRING");
}

TEST(OnlineGnss, StaleAssociationDoesNotPreventNewFixAcquisition)
{
    auto c = config();
    c.gnss.acquire_frames = 3;
    online::Core core(c);
    initialize(core, c);
    ASSERT_TRUE(core.ready());
    const auto count = core.gnssFactors();
    core.addGnss(fix(frame(12), c));
    core.addGnss(fix(frame(13), c));
    core.tick(106.8);
    EXPECT_FALSE(core.gnssLive());
    EXPECT_EQ(core.reason(), "WAITING_FOR_GNSS_ASSOCIATION");
    core.addKeyframe(frame(14));
    core.addKeyframe(frame(15));
    core.addGnss(fix(frame(15), c));
    EXPECT_TRUE(core.gnssLive());
    EXPECT_EQ(core.gnssFactors(), count + 1);
}

TEST(OnlineMap, UsesOptimizedImuPoseNotAntennaAndOwnsCloudSnapshot)
{
    auto c = config();
    online::Core core(c);
    auto f = frame(0);
    PointCloudXYZRGB::Ptr input(new PointCloudXYZRGB(*f.cloud_imu));
    f.cloud_imu = input;
    core.addKeyframe(f);
    input->points[0].x = 99;
    core.addGnss(fix(f, c));
    for (int i = 1; i < 12; ++i) {
        core.addKeyframe(frame(i));
        core.addGnss(fix(frame(i), c));
    }
    ASSERT_TRUE(core.ready());
    EXPECT_FLOAT_EQ(core.nodes().front().cloud->points[0].x, 1.0f);
    const auto map = core.buildMap();
    const Eigen::Vector3d expected = alignment().transformFrom(Eigen::Vector3d(1, 2, 3));
    double distance = 1e9;
    for (const auto& point : *map)
        distance = std::min(distance, (Eigen::Vector3d(point.x, point.y, point.z) - expected).norm());
    EXPECT_LT(distance, 1e-4);
}

TEST(OnlineBounds, InitialHistoryAndPendingCloudsAreBounded)
{
    auto c = config();
    c.init_baseline = 100.0;
    c.max_init_frames = 5;
    c.max_pending_gnss = 8;
    c.cloud_cache = 2;
    online::Core core(c);
    for (int i = 0; i < 30; ++i) {
        core.addKeyframe(frame(i));
        core.addGnss(fix(frame(i), c));
    }
    EXPECT_FALSE(core.ready());
    EXPECT_LE(core.nodes().size(), 5u);
    EXPECT_LE(core.pending(), 8u);
    std::size_t clouds = 0;
    for (const auto& node : core.nodes()) if (node.cloud) ++clouds;
    EXPECT_LE(clouds, 2u);
}

TEST(OnlineMap, RemovalOnlyEventClearsCachedMapWithoutChangingRawPoses)
{
    const auto c = config();
    online::Core core(c);
    initialize(core, c);
    ASSERT_TRUE(core.ready());
    const auto raw = core.nodes().front().raw;
    auto event = frame(12);
    event.cloud_imu.reset(new PointCloudXYZRGB());
    event.removed_cell_size = 0.3;
    event.removed_cells_odom = {{3, 6, 10}};
    ASSERT_TRUE(backend::validKeyframe(event));
    EXPECT_FALSE(core.addKeyframe(event));
    EXPECT_TRUE(core.nodes().front().cloud->empty());
    EXPECT_TRUE(core.nodes().front().raw.equals(raw));
    EXPECT_EQ(core.nodes().size(), 12u);
    EXPECT_EQ(core.gnssFactors(), 12u);
}

TEST(OnlineSafety, InvalidLocalStateSuspendsGnssIndependently)
{
    const auto c = config();
    online::Core core(c);
    initialize(core, c);
    const auto factors = core.gnssFactors();
    core.setLocalValid(false);
    EXPECT_FALSE(core.addGnss(fix(frame(12), c)));
    EXPECT_EQ(core.gnssFactors(), factors);
    EXPECT_FALSE(core.gnssLive());
}

TEST(OnlineSafety, NewSegmentHasNoCrossSegmentOdometryFactor)
{
    const auto c = config();
    online::Core core(c);
    initialize(core, c);
    ASSERT_TRUE(core.ready());
    auto start = frame(12);
    start.segment_id = 1;
    start.segment_start = true;
    start.pose_frame = "odom_s1";
    start.cloud_imu.reset(new PointCloudXYZRGB);
    ASSERT_TRUE(backend::validKeyframe(start));
    EXPECT_FALSE(core.addKeyframe(start));
    EXPECT_EQ(core.segment(), 1u);
    EXPECT_FALSE(core.ready());
    EXPECT_TRUE(core.nodes().empty());
    EXPECT_EQ(core.gnssFactors(), 0u);
    core.setLocalValid(true);
    for (int i = 20; i < 32; ++i) {
        auto f = frame(i);
        f.segment_id = 1;
        f.pose_frame = "odom_s1";
        f.T_odom_imu.translation().x() -= 10.0;
        core.addKeyframe(f);
        core.addGnss(fix(f, c));
    }
    ASSERT_TRUE(core.ready());
    EXPECT_EQ(core.nodes().size(), 12u);
    EXPECT_EQ(core.nodes().front().sequence, 20u);
    EXPECT_TRUE(core.correction().equals(alignment(), 1e-5));
    auto stale = frame(32);
    EXPECT_FALSE(core.addKeyframe(stale));
}

TEST(OnlineResources, NodeBudgetStopsGlobalWithoutUnboundedGrowth)
{
    auto c = config(); c.max_graph_nodes = 12; c.cloud_cache = 3;
    online::Core core(c);
    initialize(core, c);
    ASSERT_TRUE(core.ready());
    EXPECT_LE(core.cachedPoints(), 3u);
    const auto factors = core.graphFactors();
    EXPECT_FALSE(core.addKeyframe(frame(12)));
    EXPECT_TRUE(core.failed());
    EXPECT_FALSE(core.ready());
    EXPECT_EQ(core.reason(), "GRAPH_NODE_BUDGET_EXHAUSTED");
    for (int i = 13; i < 100; ++i) core.addKeyframe(frame(i));
    EXPECT_EQ(core.nodes().size(), 12u);
    EXPECT_EQ(core.graphFactors(), factors);
}

TEST(OnlineResources, FactorBudgetIsCheckedBeforeIsamMutation)
{
    auto c = config(); c.max_graph_factors = 24;
    online::Core core(c);
    initialize(core, c);
    ASSERT_TRUE(core.ready());
    ASSERT_EQ(core.graphFactors(), 24u);
    core.addKeyframe(frame(12));
    EXPECT_TRUE(core.failed());
    EXPECT_EQ(core.reason(), "GRAPH_FACTOR_BUDGET_EXHAUSTED");
    EXPECT_EQ(core.graphFactors(), 24u);
    EXPECT_FALSE(core.addGnss(fix(frame(12), c)));
}

int main(int argc, char** argv)
{
    ros::Time::init();
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
