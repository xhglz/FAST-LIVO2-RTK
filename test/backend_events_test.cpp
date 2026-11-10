#include "backend_events.h"
#include "voxel_map.h"
#include "optimization.h"
#include <gtest/gtest.h>

namespace {
void configureBackend(bool gps)
{
    ros::NodeHandle nh;
    nh.setParam("laserMapping/outputfilepath", "/tmp/livo-backend-unit-" + std::to_string(getpid()));
    nh.setParam("gps/gps_en", gps);
    nh.setParam("gps/extrinsic_T", std::vector<double>{0.4, 0.0, 0.0});
    nh.setParam("opt/debug_mode", false);
}

backend::KeyframeEvent makeKeyframe(std::uint64_t sequence, const V3D& position)
{
    backend::KeyframeEvent event;
    event.sequence = sequence;
    event.measurement_time = 100.0 + sequence;
    event.T_odom_imu.translation() = position;
    PointCloudXYZRGB::Ptr cloud(new PointCloudXYZRGB());
    PointTypeRGB point;
    point.x = 1.0;
    point.y = 2.0;
    point.z = 3.0;
    cloud->push_back(point);
    event.cloud_imu = cloud;
    return event;
}
} // namespace

TEST(BackendEvents, GnssTimeOffsetAndQualityArePreserved)
{
    gnss_comm::GnssPVTSolnMsg msg;
    msg.time.week = 2400;
    msg.time.tow = 123456.125;
    msg.latitude = 30.0;
    msg.longitude = 120.0;
    msg.altitude = 20.0;
    msg.h_acc = 0.02;
    msg.v_acc = 0.04;
    msg.vel_e = 1.0;
    msg.vel_n = 2.0;
    msg.vel_d = 3.0;
    msg.valid_fix = true;
    msg.carr_soln = 2;
    msg.num_sv = 20;
    GeographicLib::LocalCartesian origin;
    bool initialized = false;
    backend::GnssEvent event;
    ASSERT_TRUE(backend::decodeGnss(msg, 0.001, 18.0, origin, initialized, event));
    EXPECT_DOUBLE_EQ(event.raw_unix_time, 315964800.0 + 2400 * 604800.0 + 123456.125 - 18.0);
    EXPECT_DOUBLE_EQ(event.measurement_time, event.raw_unix_time - 0.001);
    EXPECT_DOUBLE_EQ(event.gps_tow, msg.time.tow);
    EXPECT_NEAR(event.position_enu.norm(), 0.0, 1e-8);
    EXPECT_DOUBLE_EQ(event.velocity_enu.z(), -3.0);
    EXPECT_DOUBLE_EQ(event.position_variance.x(), 0.02 * 0.02);
    EXPECT_TRUE(event.valid_fix);
    EXPECT_EQ(event.carrier_solution, 2);
    EXPECT_EQ(event.satellites, 20);
    const auto first_time = event.raw_unix_time;
    ASSERT_TRUE(backend::decodeGnss(msg, 0.001, 18.0, origin, initialized, event));
    EXPECT_DOUBLE_EQ(event.raw_unix_time, first_time);
}

TEST(BackendEvents, InvalidGnssDoesNotInitializeOrigin)
{
    gnss_comm::GnssPVTSolnMsg msg;
    msg.time.tow = 604800.0;
    GeographicLib::LocalCartesian origin;
    bool initialized = false;
    backend::GnssEvent event;
    EXPECT_FALSE(backend::decodeGnss(msg, 0.0, 18.0, origin, initialized, event));
    EXPECT_FALSE(initialized);
    msg.time.tow = 1.0;
    msg.latitude = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(backend::decodeGnss(msg, 0.0, 18.0, origin, initialized, event));
    EXPECT_FALSE(initialized);
}

TEST(BackendEvents, KeyframeRequiresFiniteRigidPoseAndBodyCloud)
{
    backend::KeyframeEvent event;
    event.measurement_time = 100.125;
    EXPECT_FALSE(backend::validKeyframe(event));
    PointCloudXYZRGB::Ptr cloud(new PointCloudXYZRGB());
    cloud->push_back(PointTypeRGB());
    event.cloud_imu = cloud;
    EXPECT_TRUE(backend::validKeyframe(event));
    event.T_odom_imu.linear()(0, 0) = 2.0;
    EXPECT_FALSE(backend::validKeyframe(event));
    event.T_odom_imu = Eigen::Isometry3d::Identity();
    event.measurement_time = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(backend::validKeyframe(event));
}

TEST(BackendEvents, CovarianceOrderAndRotationAxesRoundTrip)
{
    const M3D R = Eigen::AngleAxisd(0.5, V3D::UnitZ()).toRotationMatrix();
    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Identity();
    covariance(0, 0) = 2.0;
    covariance(3, 3) = 4.0;
    covariance(1, 4) = covariance(4, 1) = 0.1;
    const auto ros_covariance = backend::poseCovarianceToRos(R, covariance);
    EXPECT_DOUBLE_EQ(ros_covariance(0, 0), 4.0);
    EXPECT_TRUE((ros_covariance.block<3, 3>(3, 3).isApprox(
        R * covariance.block<3, 3>(0, 0) * R.transpose())));
    EXPECT_TRUE(backend::poseCovarianceFromRos(R, ros_covariance).isApprox(covariance));
}

TEST(LocalRtkRows, AppendedRowsDoNotIndexLidarMatches)
{
    VoxelMapConfig config{};
    config.max_iterations_ = 1;
    config.max_voxel_size_ = 1.0;
    config.beam_err_ = 0.01;
    config.dept_err_ = 0.01;
    std::unordered_map<VOXEL_LOCATION, VoxelOctoTree*> map;
    VoxelMapManager manager(config, map);
    manager.extR_ = M3D::Identity();
    manager.extT_ = V3D::Zero();
    manager.feats_down_size_ = 1;
    PointType point;
    point.x = 1.0;
    point.y = 0.5;
    point.z = 0.2;
    manager.feats_down_body_->push_back(point);
    StatesGroup propagated;
    manager.state_ = propagated;
    manager.StateEstimation(propagated, true, V3D::Zero());
    EXPECT_EQ(manager.lidar_match_count_, 0u);
    EXPECT_EQ(manager.effct_feat_num_, 3);
    EXPECT_TRUE(manager.state_.cov.allFinite());
    EXPECT_TRUE(manager.lidar_information_.isZero());
}

TEST(LocalRtkRows, DisabledRtkPreservesPropagationWithoutLidarMatches)
{
    VoxelMapConfig config{};
    config.max_iterations_ = 1;
    config.max_voxel_size_ = 1.0;
    std::unordered_map<VOXEL_LOCATION, VoxelOctoTree*> map;
    VoxelMapManager manager(config, map);
    manager.extR_ = M3D::Identity();
    manager.extT_ = V3D::Zero();
    manager.feats_down_size_ = 0;
    StatesGroup propagated;
    manager.state_ = propagated;
    manager.StateEstimation(propagated, false, V3D::Zero());
    EXPECT_EQ(manager.effct_feat_num_, 0);
    EXPECT_TRUE(manager.state_.pos_end.isApprox(propagated.pos_end));
    EXPECT_TRUE(manager.state_.cov.isApprox(propagated.cov));
}

TEST(BackendStorage, SnapshotAndMonotonicity)
{
    if (!ros::master::check()) GTEST_SKIP() << "Run under an isolated ROS master for backend tests";
    configureBackend(false);
    ros::NodeHandle nh;
    optimization backend(nh);
    auto first = makeKeyframe(0, V3D::Zero());
    PointCloudXYZRGB::Ptr mutable_cloud(new PointCloudXYZRGB(*first.cloud_imu));
    first.cloud_imu = mutable_cloud;
    backend.onKeyframe(first);
    ASSERT_EQ(backend.keyFrames.size(), 1u);
    mutable_cloud->points[0].x = 99.0;
    EXPECT_FLOAT_EQ(backend.keyFrames[0].cloud->points[0].x, 1.0);
    backend.onKeyframe(first);
    auto invalid = makeKeyframe(1, V3D::Zero());
    invalid.body_frame = "lidar_link";
    backend.onKeyframe(invalid);
    EXPECT_EQ(backend.keyFrames.size(), 1u);
    backend.onKeyframe(makeKeyframe(2, V3D::Ones()));
    EXPECT_EQ(backend.keyFrames.size(), 2u);
    backend::GnssEvent gps;
    gps.raw_unix_time = gps.measurement_time = 101.0;
    backend.onGnss(gps);
    EXPECT_TRUE(backend.gnss_events.empty());
}

TEST(BackendStorage, AlignmentDoesNotOverwriteRawDataOrUseAntennaPoseForCloud)
{
    if (!ros::master::check()) GTEST_SKIP() << "Run under an isolated ROS master for backend tests";
    configureBackend(true);
    ros::NodeHandle nh;
    optimization backend(nh);
    const std::vector<V3D, Eigen::aligned_allocator<V3D>> positions = {
        V3D::Zero(), V3D(1.0, 0.0, 0.0), V3D(1.0, 2.0, 0.0)};
    const gtsam::Pose3 alignment(gtsam::Rot3::Rz(0.3), gtsam::Point3(10.0, 20.0, 30.0));
    for (std::size_t i = 0; i < positions.size(); ++i) {
        const auto event = makeKeyframe(i, positions[i]);
        backend.onKeyframe(event);
        backend::GnssEvent gps;
        gps.sequence = i;
        gps.raw_unix_time = gps.measurement_time = event.measurement_time;
        const V3D antenna = positions[i] + V3D(0.4, 0.0, 0.0);
        gps.position_enu = alignment.transformFrom(antenna);
        backend.onGnss(gps);
    }
    backend.onGnss(backend.gnss_events.back());
    ASSERT_EQ(backend.gnss_events.size(), 3u);
    ASSERT_TRUE(backend.initialAlign());
    EXPECT_TRUE(backend.T_map_odom.equals(alignment, 1e-6));
    ASSERT_TRUE(backend.initialAlign());
    EXPECT_TRUE(backend.T_map_odom.equals(alignment, 1e-6));
    for (std::size_t i = 0; i < positions.size(); ++i) {
        EXPECT_TRUE(backend.keyFrames[i].pose.translation().isApprox(positions[i]));
        EXPECT_DOUBLE_EQ(backend.gnss_events[i].raw_unix_time, 100.0 + i);
    }
    backend.saveOptimizedGlobalMap();
    PointCloudXYZRGB map;
    ASSERT_EQ(pcl::io::loadPCDFile(backend.global_map_pcd_path_ + "before_optimization.pcd", map), 0);
    const auto expected = alignment.transformFrom(V3D(1.0, 2.0, 3.0));
    EXPECT_NEAR(map.points[0].x, expected.x(), 1e-5);
    EXPECT_NEAR(map.points[0].y, expected.y(), 1e-5);
    EXPECT_NEAR(map.points[0].z, expected.z(), 1e-5);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "backend_events_test", ros::init_options::AnonymousName);
    ros::Time::init();
    std::unique_ptr<ros::NodeHandle> keepalive;
    if (ros::master::check()) keepalive.reset(new ros::NodeHandle());
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
