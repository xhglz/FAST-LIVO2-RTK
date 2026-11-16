#include "localization_safety.h"
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Eigenvalues>
#include <stdexcept>

namespace safety {
const char* name(State state)
{
    switch (state) {
    case State::INITIALIZING: return "INITIALIZING";
    case State::NORMAL: return "NORMAL";
    case State::DEGRADED: return "DEGRADED";
    case State::LOST: return "LOST";
    case State::RECOVERING: return "RECOVERING";
    }
    return "LOST";
}

void Machine::transition(State state, const std::string& reason)
{
    if (state != state_) ++transitions_;
    state_ = state;
    reason_ = reason;
}

void Machine::lose(const std::string& reason)
{
    valid_ = false;
    good_ = bad_ = 0;
    transition(State::LOST, reason);
}

void Machine::observe(const Evidence& e)
{
    if (!std::isfinite(e.time) || e.time <= last_time_) {
        lose("MEASUREMENT_TIME_INVALID");
        return;
    }
    last_time_ = e.time;
    const bool finite = e.finite && std::isfinite(e.position_sigma) && std::isfinite(e.rotation_sigma);
    const bool bounded = finite && e.position_sigma >= 0.0 && e.rotation_sigma >= 0.0 &&
        e.position_sigma <= config_.max_position_sigma && e.rotation_sigma <= config_.max_rotation_sigma;
    if (!finite || !e.imu || !bounded) {
        lose(!finite ? "NONFINITE_STATE" : !e.imu ? "IMU_UNAVAILABLE" : "UNCERTAINTY_LIMIT");
        return;
    }
    const bool local = e.lidar && e.geometry && e.initialized;
    if (!local) {
        valid_ = false;
        good_ = 0;
        if (state_ == State::RECOVERING ||
            (last_good_ > 0.0 && e.time - last_good_ > config_.max_prediction_time) ||
            (++bad_ >= config_.lost_frames && state_ != State::INITIALIZING))
            lose("LOCAL_GEOMETRY_LOST");
        else if (state_ != State::LOST && state_ != State::INITIALIZING)
            transition(State::DEGRADED, "LOCAL_GEOMETRY_WEAK");
        return;
    }
    bad_ = 0;
    if (state_ == State::LOST) {
        if (!e.relocalized) return; // A healthy GNSS or LIO flag alone cannot clear LOST.
        good_ = 1;
        valid_ = false;
        transition(State::RECOVERING, "VERIFYING_TRUSTED_SUBMAP");
        return;
    }
    if (state_ == State::RECOVERING) {
        if (!e.relocalized) { lose("RELOCALIZATION_NOT_VERIFIED"); return; }
        if (++good_ < config_.recover_frames) return;
    } else if (state_ == State::INITIALIZING) {
        if (++good_ < config_.initialize_frames) return;
    } else if (state_ == State::DEGRADED && !valid_) {
        if (++good_ < config_.recover_frames) {
            reason_ = "VERIFYING_LOCAL_GEOMETRY";
            return;
        }
    }
    valid_ = true;
    last_good_ = e.time;
    if (!e.camera || !e.gnss)
        transition(State::DEGRADED, !e.camera ? "CAMERA_UNAVAILABLE" : "GNSS_UNAVAILABLE");
    else transition(State::NORMAL, "OK");
}

void Machine::tick(double now, bool imu, bool lidar)
{
    if (!std::isfinite(now)) { lose("CLOCK_INVALID"); return; }
    if (state_ == State::INITIALIZING && last_good_ < 0.0) return;
    if (!imu || !lidar || (last_time_ > 0 && now - last_time_ > config_.max_prediction_time))
        lose(!imu ? "IMU_UNAVAILABLE" : !lidar ? "LIDAR_UNAVAILABLE" : "PREDICTION_TIME_LIMIT");
}

bool Machine::predictionAllowed(double time, double ps, double rs) const
{
    return valid_ && std::isfinite(time) && time >= last_good_ &&
        time - last_good_ <= config_.max_prediction_time && std::isfinite(ps) && std::isfinite(rs) &&
        ps >= 0.0 && rs >= 0.0 && ps <= config_.max_position_sigma && rs <= config_.max_rotation_sigma;
}

void Machine::newSegment()
{
    ++segment_;
    valid_ = false;
    good_ = bad_ = 0;
    last_time_ = last_good_ = -1.0;
    transition(State::INITIALIZING, "NEW_ODOMETRY_SEGMENT");
}

void Relocalizer::remember(const NormalCloud::ConstPtr& points, double time, const V3D& origin)
{
    if (!points || time - last_time_ < config_.snapshot_period || !origin.allFinite()) return;
    NormalCloud::Ptr nearby(new NormalCloud);
    for (const auto& p : *points) {
        const V3D position(p.x, p.y, p.z), normal(p.normal_x, p.normal_y, p.normal_z);
        if (position.allFinite() && normal.allFinite() && normal.norm() > 0.5 &&
            (position - origin).norm() <= config_.submap_radius) nearby->push_back(p);
    }
    pcl::VoxelGrid<pcl::PointNormal> filter;
    filter.setLeafSize(0.25f, 0.25f, 0.25f);
    filter.setInputCloud(nearby);
    NormalCloud::Ptr sampled(new NormalCloud);
    filter.filter(*sampled);
    const std::size_t per_frame = std::max(1, config_.submap_points / config_.submap_frames);
    if (sampled->size() > per_frame) {
        NormalCloud::Ptr bounded(new NormalCloud);
        for (std::size_t i = 0; i < per_frame; ++i) bounded->push_back((*sampled)[i * sampled->size() / per_frame]);
        sampled = bounded;
    }
    if (sampled->size() < 30) return;
    last_time_ = time;
    snapshots_.push_back(sampled);
    while (snapshots_.size() > static_cast<std::size_t>(config_.submap_frames)) snapshots_.pop_front();
}

std::size_t Relocalizer::size() const
{
    std::size_t n = 0;
    for (const auto& snapshot : snapshots_) n += snapshot->size();
    return n;
}

Registration Relocalizer::align(const XYZCloud::ConstPtr& points, const Eigen::Isometry3d& guess) const
{
    Registration result;
    if (!points || points->size() < 30 || size() < 30 || !guess.matrix().allFinite()) return result;
    NormalCloud::Ptr normals(new NormalCloud);
    for (const auto& snapshot : snapshots_) *normals += *snapshot;
    XYZCloud::Ptr target(new XYZCloud), source(new XYZCloud);
    for (const auto& p : *normals) target->push_back(pcl::PointXYZ(p.x, p.y, p.z));
    for (const auto& p : *points)
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) source->push_back(p);
    if (source->size() < 30) return result;
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setLeafSize(0.25f, 0.25f, 0.25f);
    filter.setInputCloud(source);
    XYZCloud::Ptr sampled(new XYZCloud);
    filter.filter(*sampled);
    if (sampled->size() > static_cast<std::size_t>(config_.submap_points)) {
        XYZCloud::Ptr bounded(new XYZCloud);
        for (int i = 0; i < config_.submap_points; ++i)
            bounded->push_back((*sampled)[static_cast<std::size_t>(i) * sampled->size() / config_.submap_points]);
        sampled = bounded;
    }
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(sampled);
    icp.setInputTarget(target);
    icp.setMaxCorrespondenceDistance(config_.registration_distance);
    icp.setMaximumIterations(30);
    icp.setTransformationEpsilon(1e-7);
    XYZCloud aligned;
    icp.align(aligned, guess.matrix().cast<float>());
    if (!icp.hasConverged()) { result.reason = "ICP_NOT_CONVERGED"; return result; }
    result.pose.matrix() = icp.getFinalTransformation().cast<double>();
    if (!result.pose.matrix().allFinite()) { result.reason = "ICP_NONFINITE"; return result; }
    Eigen::Quaterniond rotation(result.pose.linear());
    rotation.normalize();
    result.pose.linear() = rotation.toRotationMatrix();
    const auto relative = guess.inverse() * result.pose;
    const double angle = Eigen::AngleAxisd(relative.linear()).angle();
    if (!result.pose.matrix().allFinite() || relative.translation().norm() > config_.registration_translation ||
        angle > config_.registration_rotation) {
        result.reason = "RELOCALIZATION_CORRECTION_LIMIT";
        return result;
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(target);
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    double squared = 0.0;
    std::size_t inliers = 0;
    for (const auto& p : aligned) {
        std::vector<int> indices(1);
        std::vector<float> distances(1);
        if (!tree.nearestKSearch(p, 1, indices, distances) ||
            distances[0] > config_.registration_rms * config_.registration_rms * 4.0) continue;
        const auto& n = (*normals)[indices[0]];
        V3D normal(n.normal_x, n.normal_y, n.normal_z);
        if (normal.norm() < 0.5) continue;
        normal.normalize();
        const V3D relative_point = V3D(p.x, p.y, p.z) - result.pose.translation();
        Eigen::Matrix<double, 6, 1> J;
        J.head<3>() = relative_point.cross(normal) * 0.1;
        J.tail<3>() = normal;
        H += J * J.transpose();
        squared += distances[0];
        ++inliers;
    }
    result.overlap = aligned.empty() ? 0.0 : static_cast<double>(inliers) / aligned.size();
    result.rms = inliers ? std::sqrt(squared / inliers) : std::numeric_limits<double>::infinity();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(H);
    if (solver.info() == Eigen::Success)
        result.eigen_ratio = std::max(0.0, solver.eigenvalues()[0]) / std::max(1e-12, solver.eigenvalues()[5]);
    result.accepted = result.overlap >= config_.registration_overlap &&
        result.rms <= config_.registration_rms && result.eigen_ratio >= config_.registration_eigen_ratio;
    result.reason = result.accepted ? "TRUSTED_SUBMAP_MATCH" : "RELOCALIZATION_QUALITY_REJECTED";
    return result;
}

void loadConfig(ros::NodeHandle& nh, Config& c)
{
    nh.param("safety/enabled", c.enabled, c.enabled);
    auto positive = [&](const std::string& key, auto& value) {
        nh.param("safety/" + key, value, value);
        if (!std::isfinite(static_cast<double>(value)) || value <= 0)
            throw std::invalid_argument("safety/" + key + " must be positive and finite");
    };
    positive("initialize_frames", c.initialize_frames); positive("lost_frames", c.lost_frames);
    positive("recover_frames", c.recover_frames); positive("max_prediction_time", c.max_prediction_time);
    positive("max_imu_gap", c.max_imu_gap); positive("max_position_sigma", c.max_position_sigma);
    positive("max_rotation_sigma", c.max_rotation_sigma); positive("status_timeout", c.status_timeout);
    positive("snapshot_period", c.snapshot_period); positive("submap_radius", c.submap_radius);
    positive("submap_frames", c.submap_frames); positive("submap_points", c.submap_points);
    positive("registration_distance", c.registration_distance); positive("registration_rms", c.registration_rms);
    positive("registration_overlap", c.registration_overlap); positive("registration_translation", c.registration_translation);
    positive("registration_rotation", c.registration_rotation); positive("registration_eigen_ratio", c.registration_eigen_ratio);
    if (c.recover_frames < 2 || c.initialize_frames < 2 || c.registration_overlap > 1 ||
        c.max_imu_gap > c.max_prediction_time || c.submap_points < 30 || c.submap_frames > c.submap_points)
        throw std::invalid_argument("inconsistent safety bounds");
}
} // namespace safety
