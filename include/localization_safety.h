#pragma once

#include "health_monitor.h"
#include <pcl/point_types.h>

namespace safety {

enum class State { INITIALIZING, NORMAL, DEGRADED, LOST, RECOVERING };
const char* name(State state);

struct Config {
    bool enabled = false;
    int initialize_frames = 5, lost_frames = 3, recover_frames = 5;
    double max_prediction_time = 0.5, max_imu_gap = 0.05;
    double max_position_sigma = 0.5, max_rotation_sigma = 0.15;
    double status_timeout = 0.5;
    double snapshot_period = 0.5, submap_radius = 25.0;
    int submap_frames = 8, submap_points = 30000;
    double registration_distance = 0.6, registration_rms = 0.15;
    double registration_overlap = 0.6, registration_translation = 2.0;
    double registration_rotation = 0.35, registration_eigen_ratio = 1e-5;
};

struct Evidence {
    double time = 0.0;
    bool finite = true, imu = true, lidar = true, geometry = true;
    bool camera = true, gnss = true, initialized = true, relocalized = false;
    double position_sigma = 0.0, rotation_sigma = 0.0;
};

class Machine {
public:
    explicit Machine(const Config& config = Config()) : config_(config) {}
    void observe(const Evidence& evidence);
    void tick(double now, bool imu, bool lidar);
    void lose(const std::string& reason);
    void newSegment();
    State state() const { return state_; }
    const std::string& reason() const { return reason_; }
    bool poseValid() const { return valid_; }
    bool mapWritable() const { return valid_ && (state_ == State::NORMAL || state_ == State::DEGRADED); }
    bool recovering() const { return state_ == State::LOST || state_ == State::RECOVERING; }
    bool predictionAllowed(double time, double position_sigma, double rotation_sigma) const;
    double lastGoodTime() const { return last_good_; }
    std::uint64_t segment() const { return segment_; }
    std::size_t transitions() const { return transitions_; }
private:
    void transition(State state, const std::string& reason);
    Config config_;
    State state_ = State::INITIALIZING;
    std::string reason_ = "WAITING_FOR_LOCAL_INITIALIZATION";
    bool valid_ = false;
    int good_ = 0, bad_ = 0;
    double last_time_ = -1.0, last_good_ = -1.0;
    std::uint64_t segment_ = 0;
    std::size_t transitions_ = 0;
};

using NormalCloud = pcl::PointCloud<pcl::PointNormal>;
using XYZCloud = pcl::PointCloud<pcl::PointXYZ>;
struct Registration {
    bool accepted = false;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    double rms = std::numeric_limits<double>::infinity(), overlap = 0.0, eigen_ratio = 0.0;
    std::string reason = "NO_TRUSTED_SUBMAP";
};

class Relocalizer {
public:
    explicit Relocalizer(const Config& config = Config()) : config_(config) {}
    void remember(const NormalCloud::ConstPtr& points, double time, const V3D& origin);
    Registration align(const XYZCloud::ConstPtr& imu_points, const Eigen::Isometry3d& guess) const;
    void clear() { snapshots_.clear(); last_time_ = -1.0; }
    std::size_t size() const;
private:
    Config config_;
    std::deque<NormalCloud::ConstPtr> snapshots_;
    double last_time_ = -1.0;
};

void loadConfig(ros::NodeHandle& nh, Config& config);
} // namespace safety
