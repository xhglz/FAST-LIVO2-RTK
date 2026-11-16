#pragma once

#include "health_monitor.h"
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <condition_variable>
#include <chrono>
#include <deque>
#include <fstream>
#include <mutex>
#include <thread>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <fast_livo/LocalizationStatus.h>

namespace online {

gtsam::Pose3 interpolatePose(const gtsam::Pose3& a, const gtsam::Pose3& b, double alpha);

// One genuine GNSS observation, at its timestamp between two IMU pose nodes.
class AntennaFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>
{
public:
    AntennaFactor(gtsam::Key a, gtsam::Key b, double alpha, const Eigen::Vector3d& lever,
                  const Eigen::Vector3d& measured, const gtsam::SharedNoiseModel& noise);
    gtsam::Vector evaluateError(const gtsam::Pose3& a, const gtsam::Pose3& b,
        boost::optional<gtsam::Matrix&> H1 = boost::none,
        boost::optional<gtsam::Matrix&> H2 = boost::none) const override;
private:
    double alpha_;
    Eigen::Vector3d lever_, measured_;
};

struct Config
{
    backend::Frames frames;
    health::GnssConfig gnss;
    Eigen::Vector3d lever = Eigen::Vector3d::Zero();
    double keyframe_distance = 0.5, keyframe_rotation = 0.15;
    double keyframe_max_time = 0.5, keyframe_min_time = 0.05;
    std::size_t min_matches = 30, init_min_pairs = 10;
    double min_eigen_ratio = 1e-6;
    double max_normalized_rms = 5.0;
    double init_baseline = 5.0, init_min_duration = 2.0;
    double init_max_rms = 0.5, init_min_correlation = 0.8;
    double gravity_tolerance = 0.08726646;
    double association_max_gap = 1.0, pending_max_lag = 2.0;
    double odom_rotation_sigma = 0.01, odom_position_sigma = 0.1;
    double prior_tilt_sigma = 0.02, prior_yaw_sigma = 0.5, prior_position_sigma = 1.0;
    double interpolation_sigma_per_second = 0.2;
    double innovation_sigma = 0.5, innovation_chi2 = 16.27, innovation_max_distance = 5.0;
    double gnss_timeout = 1.0;
    double map_period = 5.0, map_leaf = 0.25;
    std::size_t max_queue = 200, max_pending_gnss = 300, max_init_frames = 500;
    std::size_t cloud_cache = 100, cloud_max_points = 20000, map_max_points = 300000;
    std::size_t max_graph_nodes = 0, max_graph_factors = 0, path_max_poses = 0;
    std::string output_directory;
};

bool selectable(const backend::KeyframeEvent& event, const Config& config);
bool shouldSelect(const gtsam::Pose3& previous, double previous_time,
                  const backend::KeyframeEvent& event, const Config& config);
void loadConfig(ros::NodeHandle& nh, Config& config);

struct Node
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double time = 0.0;
    std::uint64_t sequence = 0;
    gtsam::Pose3 raw, optimized;
    Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
    PointCloudXYZRGB::ConstPtr cloud;
};
using Nodes = std::vector<Node, Eigen::aligned_allocator<Node>>;

// Single-owner core. It is also usable synchronously in deterministic tests.
class Core
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit Core(const Config& config);
    bool addKeyframe(const backend::KeyframeEvent& event);
    bool addGnss(const backend::GnssEvent& event);
    void tick(double measurement_time);
    void setLocalValid(bool valid);
    bool beginSegment(std::uint64_t segment);
    std::uint64_t segment() const { return segment_; }
    bool ready() const { return ready_; }
    bool failed() const { return failed_; }
    void disable() { failed_ = true; ready_ = false; gnss_live_ = false; reason_ = "OPTIMIZER_FAILED"; }
    bool gnssLive() const { return gnss_live_; }
    const Nodes& nodes() const { return nodes_; }
    const gtsam::Pose3& correction() const { return correction_; }
    const std::string& reason() const { return reason_; }
    std::size_t gnssFactors() const { return gnss_factors_; }
    std::size_t rejectedGnss() const { return gnss_rejected_; }
    std::size_t updates() const { return updates_; }
    std::size_t pending() const { return pending_.size(); }
    std::size_t graphFactors() const { return isam_->getFactorsUnsafe().size(); }
    std::size_t cachedPoints() const;
    double lastFactorTime() const { return last_factor_time_; }
    double lastInnovation() const { return last_innovation_; }
    double initRms() const { return init_rms_; }
    const std::vector<double>& factorTimes() const { return factor_times_; }
    gtsam::Matrix66 latestCovariance() const;
    PointCloudXYZRGB::Ptr buildMap() const;
private:
    bool associate(double time, std::size_t& a, std::size_t& b, double& alpha) const;
    bool initialize();
    void processPending();
    void resetFix();
    void exhaustBudget(const char* reason);
    void update(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values);
    gtsam::SharedNoiseModel gnssNoise(const backend::GnssEvent& event, double interval) const;
    Config config_;
    std::string base_odom_frame_;
    std::uint64_t segment_ = 0;
    bool local_valid_ = true;
    Nodes nodes_;
    std::deque<backend::GnssEvent, Eigen::aligned_allocator<backend::GnssEvent>> pending_;
    std::unique_ptr<gtsam::ISAM2> isam_;
    bool ready_ = false, failed_ = false, gnss_live_ = false;
    std::string reason_ = "WAITING_FOR_MOTION_AND_FIX";
    gtsam::Pose3 correction_;
    double last_key_time_ = -1.0, last_gnss_time_ = -1.0;
    double last_factor_time_ = -1.0, last_innovation_ = 0.0, init_rms_ = 0.0;
    std::uint64_t last_key_sequence_ = 0, last_gnss_sequence_ = 0;
    bool have_key_ = false, have_gnss_ = false;
    int fix_streak_ = 0;
    std::size_t gnss_factors_ = 0, gnss_rejected_ = 0, updates_ = 0;
    std::vector<double> factor_times_;
};

// Callbacks enqueue snapshots only. GTSAM, map construction and IO run on worker_.
class Backend
{
public:
    explicit Backend(ros::NodeHandle& nh);
    ~Backend();
    void onKeyframe(const backend::KeyframeEvent& event);
    void onGnss(const backend::GnssEvent& event);
private:
    void onStatus(const fast_livo::LocalizationStatus::ConstPtr& status);
    void archiveSegment();
    struct Event {
        std::chrono::steady_clock::time_point enqueued;
        bool is_gnss = false;
        backend::KeyframeEvent key;
        backend::GnssEvent gps;
    };
    void run();
    void publish(const backend::KeyframeEvent& latest, double now, bool local_healthy,
                 std::size_t queue_depth, std::size_t dropped);
    void record(const std::string& type, double time);
    Config config_;
    bool gps_enabled_ = false, shutdown_ = false, have_latest_ = false, have_selected_ = false;
    backend::KeyframeEvent latest_;
    gtsam::Pose3 last_selected_;
    double last_selected_time_ = -1.0;
    std::mutex mutex_;
    std::condition_variable wake_;
    std::deque<Event> queue_;
    std::size_t dropped_ = 0;
    std::unique_ptr<Core> core_;
    bool safety_enabled_ = false;
    fast_livo::LocalizationStatus local_status_;
    ros::Subscriber status_sub_;
    double safety_timeout_ = 0.5;
    std::thread worker_;
    ros::Publisher odom_pub_, path_pub_, map_pub_, status_pub_;
    tf::TransformBroadcaster tf_;
    double last_pose_stamp_ = -1.0, last_status_time_ = -1.0, last_map_time_ = -1.0;
    std::size_t last_path_update_ = 0, logged_factors_ = 0;
    std::ofstream events_file_, trajectory_file_, factors_file_;
    bool resources_enabled_ = false, budget_archived_ = false;
    std::ofstream resource_file_;
};

} // namespace online
