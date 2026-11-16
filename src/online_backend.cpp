#include "online_backend.h"
#include "resource_limits.h"
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <algorithm>
#include <cmath>
#include <boost/make_shared.hpp>

namespace online {

gtsam::Pose3 interpolatePose(const gtsam::Pose3& a, const gtsam::Pose3& b, double alpha)
{
    const Eigen::Quaterniond qa(a.rotation().matrix()), qb(b.rotation().matrix());
    const Eigen::Vector3d p = (1.0 - alpha) * a.translation() + alpha * b.translation();
    return gtsam::Pose3(gtsam::Rot3(qa.slerp(alpha, qb).toRotationMatrix()), p);
}

AntennaFactor::AntennaFactor(gtsam::Key a, gtsam::Key b, double alpha,
    const Eigen::Vector3d& lever, const Eigen::Vector3d& measured,
    const gtsam::SharedNoiseModel& noise)
    : NoiseModelFactor2(noise, a, b), alpha_(alpha), lever_(lever), measured_(measured)
{
}

gtsam::Vector AntennaFactor::evaluateError(const gtsam::Pose3& a, const gtsam::Pose3& b,
    boost::optional<gtsam::Matrix&> H1, boost::optional<gtsam::Matrix&> H2) const
{
    const std::function<gtsam::Vector3(const gtsam::Pose3&, const gtsam::Pose3&)> error =
        [&](const gtsam::Pose3& x, const gtsam::Pose3& y) -> gtsam::Vector3 {
            return interpolatePose(x, y, alpha_).transformFrom(lever_) - measured_;
        };
    if (H1) *H1 = gtsam::numericalDerivative21<gtsam::Vector3, gtsam::Pose3, gtsam::Pose3>(error, a, b);
    if (H2) *H2 = gtsam::numericalDerivative22<gtsam::Vector3, gtsam::Pose3, gtsam::Pose3>(error, a, b);
    return error(a, b);
}

bool selectable(const backend::KeyframeEvent& event, const Config& config)
{
    return backend::validKeyframe(event) && event.pose_frame == config.frames.odom &&
        event.body_frame == config.frames.imu && !event.cloud_imu->empty() &&
        event.quality.local_pose_valid && event.quality.lidar_update_accepted &&
        event.quality.lidar_matches >= config.min_matches &&
        std::isfinite(event.quality.eigen_ratio) && event.quality.eigen_ratio >= config.min_eigen_ratio &&
        std::isfinite(event.quality.normalized_residual_rms) &&
        event.quality.normalized_residual_rms <= config.max_normalized_rms;
}

bool shouldSelect(const gtsam::Pose3& previous, double previous_time,
    const backend::KeyframeEvent& event, const Config& config)
{
    const double dt = event.measurement_time - previous_time;
    const gtsam::Pose3 relative = previous.between(gtsam::Pose3(event.T_odom_imu.matrix()));
    return dt >= config.keyframe_min_time &&
        (dt >= config.keyframe_max_time || relative.translation().norm() >= config.keyframe_distance ||
         gtsam::Rot3::Logmap(relative.rotation()).norm() >= config.keyframe_rotation);
}

Core::Core(const Config& config) : config_(config), base_odom_frame_(config.frames.odom)
{
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.01;
    params.relinearizeSkip = 1;
    isam_.reset(new gtsam::ISAM2(params));
}

gtsam::SharedNoiseModel Core::gnssNoise(const backend::GnssEvent& event, double interval) const
{
    const Eigen::Vector3d variance = health::gnssVariance(event, config_.gnss) +
        Eigen::Vector3d::Constant(std::pow(config_.interpolation_sigma_per_second * interval, 2));
    return gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345),
        gtsam::noiseModel::Diagonal::Variances(variance));
}

bool Core::associate(double time, std::size_t& a, std::size_t& b, double& alpha) const
{
    if (nodes_.size() < 2 || time < nodes_.front().time || time > nodes_.back().time) return false;
    auto upper = std::lower_bound(nodes_.begin(), nodes_.end(), time,
        [](const Node& node, double t) { return node.time < t; });
    b = std::max<std::size_t>(1, upper - nodes_.begin());
    a = b - 1;
    const double dt = nodes_[b].time - nodes_[a].time;
    if (dt <= 0.0 || dt > config_.association_max_gap) return false;
    alpha = (time - nodes_[a].time) / dt;
    return alpha >= 0.0 && alpha <= 1.0;
}

bool Core::addKeyframe(const backend::KeyframeEvent& event)
{
    if (backend::validKeyframe(event) && event.body_frame == config_.frames.imu &&
        event.segment_start && event.segment_id > segment_ &&
        event.pose_frame == backend::segmentFrame(base_odom_frame_, event.segment_id)) {
        beginSegment(event.segment_id);
        return false;
    }
    if (event.segment_id != segment_) return false;
    if (backend::validKeyframe(event) && event.pose_frame == config_.frames.odom &&
        event.body_frame == config_.frames.imu && backend::validRemovals(event)) {
        for (auto& node : nodes_) {
            if (!node.cloud) continue;
            Eigen::Isometry3d raw = Eigen::Isometry3d::Identity();
            raw.matrix() = node.raw.matrix();
            node.cloud = backend::removeMapCells(node.cloud, raw, event);
        }
    }
    if (failed_ || !selectable(event, config_) ||
        (have_key_ && (event.measurement_time <= last_key_time_ || event.sequence <= last_key_sequence_)))
        return false;
    last_key_time_ = event.measurement_time;
    last_key_sequence_ = event.sequence;
    have_key_ = true;
    tick(event.measurement_time);
    if (!nodes_.empty() && !shouldSelect(nodes_.back().raw, nodes_.back().time, event, config_))
        return false;
    if (config_.max_graph_nodes && nodes_.size() >= config_.max_graph_nodes) {
        exhaustBudget("GRAPH_NODE_BUDGET_EXHAUSTED");
        return false;
    }

    Node node;
    node.time = event.measurement_time;
    node.sequence = event.sequence;
    node.raw = gtsam::Pose3(event.T_odom_imu.matrix());
    node.gravity = event.gravity_odom;
    node.optimized = ready_ ? correction_.compose(node.raw) : node.raw;
    pcl::VoxelGrid<PointTypeRGB> filter;
    filter.setLeafSize(config_.map_leaf, config_.map_leaf, config_.map_leaf);
    const auto bounded = (config_.max_graph_nodes || config_.max_graph_factors) ?
        resources::boundedCloud(event.cloud_imu, config_.cloud_max_points) : event.cloud_imu;
    filter.setInputCloud(bounded);
    PointCloudXYZRGB::Ptr cloud(new PointCloudXYZRGB());
    filter.filter(*cloud);
    if (cloud->size() > config_.cloud_max_points) {
        PointCloudXYZRGB::Ptr sampled(new PointCloudXYZRGB());
        const double stride = static_cast<double>(cloud->size()) / config_.cloud_max_points;
        for (std::size_t i = 0; i < config_.cloud_max_points; ++i)
            sampled->push_back(cloud->points[static_cast<std::size_t>(i * stride)]);
        cloud = sampled;
    }
    node.cloud = cloud;
    nodes_.push_back(node);
    if (!ready_ && nodes_.size() > config_.max_init_frames) nodes_.erase(nodes_.begin());
    if (nodes_.size() > config_.cloud_cache)
        nodes_[nodes_.size() - config_.cloud_cache - 1].cloud.reset();
    if (ready_) {
        const std::size_t key = nodes_.size() - 1;
        gtsam::NonlinearFactorGraph graph;
        gtsam::Values values;
        gtsam::Vector6 sigmas;
        const double scale = std::sqrt(std::max(1.0, (node.time - nodes_[key - 1].time) /
                                                          config_.keyframe_max_time));
        sigmas.head<3>().setConstant(config_.odom_rotation_sigma * scale);
        sigmas.tail<3>().setConstant(config_.odom_position_sigma * scale);
        graph.add(gtsam::BetweenFactor<gtsam::Pose3>(key - 1, key,
            nodes_[key - 1].raw.between(node.raw), gtsam::noiseModel::Diagonal::Sigmas(sigmas)));
        values.insert(key, node.optimized);
        update(graph, values);
        processPending();
    } else initialize();
    return !failed_;
}

void Core::resetFix()
{
    fix_streak_ = 0;
    gnss_live_ = false;
    // Never join pre-loss pending observations to a newly acquired FIX segment.
    gnss_rejected_ += pending_.size();
    pending_.clear();
}

bool Core::addGnss(const backend::GnssEvent& event)
{
    if (failed_) return false;
    if (!local_valid_) { ++gnss_rejected_; reason_ = "LOCAL_ODOMETRY_INVALID"; return false; }
    const bool structure = backend::validGnss(event) && event.position_frame == config_.frames.map &&
        event.body_frame == config_.frames.gnss &&
        (!have_gnss_ || (event.measurement_time > last_gnss_time_ && event.sequence > last_gnss_sequence_));
    if (!structure) {
        ++gnss_rejected_;
        resetFix();
        reason_ = "GNSS_INVALID_OR_REVERSED";
        return false;
    }
    if (have_gnss_ && event.measurement_time - last_gnss_time_ > config_.gnss_timeout) resetFix();
    last_gnss_time_ = event.measurement_time;
    last_gnss_sequence_ = event.sequence;
    have_gnss_ = true;
    const bool trusted = event.quality_available && event.trusted_fix && event.valid_fix &&
        event.fix_type == 3 && event.carrier_solution == 2 && event.satellites >= config_.gnss.min_satellites &&
        std::isfinite(event.horizontal_accuracy) && event.horizontal_accuracy >= 0.0 &&
        event.horizontal_accuracy <= config_.gnss.max_h_accuracy &&
        std::isfinite(event.vertical_accuracy) && event.vertical_accuracy >= 0.0 &&
        event.vertical_accuracy <= config_.gnss.max_v_accuracy &&
        std::isfinite(event.observation_age) && event.observation_age <= config_.gnss.max_age &&
        event.observation_age >= -config_.gnss.future_tolerance;
    if (!trusted) {
        ++gnss_rejected_;
        resetFix();
        reason_ = "GNSS_QUALITY_REJECTED";
        return false;
    }
    fix_streak_ = std::min(fix_streak_ + 1, config_.gnss.acquire_frames);
    if (!event.stable_fix || fix_streak_ < config_.gnss.acquire_frames) {
        ++gnss_rejected_;
        gnss_live_ = false;
        gnss_rejected_ += pending_.size();
        pending_.clear();
        reason_ = "GNSS_REACQUIRING";
        return false;
    }
    pending_.push_back(event);
    while (pending_.size() > config_.max_pending_gnss) {
        pending_.pop_front();
        ++gnss_rejected_;
    }
    if (ready_) processPending();
    else initialize();
    return true;
}

void Core::setLocalValid(bool valid)
{
    if (!valid && local_valid_) resetFix();
    local_valid_ = valid;
    if (!valid) reason_ = "LOCAL_ODOMETRY_INVALID";
}

bool Core::beginSegment(std::uint64_t segment)
{
    if (segment <= segment_) return false;
    segment_ = segment;
    config_.frames.odom = backend::segmentFrame(base_odom_frame_, segment);
    nodes_.clear(); pending_.clear(); factor_times_.clear();
    ready_ = failed_ = gnss_live_ = have_key_ = have_gnss_ = false;
    local_valid_ = false;
    fix_streak_ = 0;
    last_key_time_ = last_gnss_time_ = last_factor_time_ = -1.0;
    last_key_sequence_ = last_gnss_sequence_ = 0;
    gnss_factors_ = gnss_rejected_ = updates_ = 0;
    last_innovation_ = init_rms_ = 0.0;
    correction_ = gtsam::Pose3();
    reason_ = "NEW_ODOMETRY_SEGMENT";
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.01;
    params.relinearizeSkip = 1;
    isam_.reset(new gtsam::ISAM2(params));
    return true;
}

bool Core::initialize()
{
    if (ready_ || failed_ || nodes_.size() < 2) return false;
    if (!nodes_.front().gravity.allFinite() || nodes_.front().gravity.norm() < 1.0) {
        reason_ = "WAITING_FOR_GRAVITY";
        return false;
    }
    const Eigen::Vector3d gravity = nodes_.front().gravity.normalized();
    for (const auto& node : nodes_) {
        if (!node.gravity.allFinite() || node.gravity.norm() < 1.0 ||
            node.gravity.normalized().dot(gravity) < std::cos(config_.gravity_tolerance)) {
            reason_ = "UNSTABLE_GRAVITY";
            return false;
        }
    }
    const Eigen::Matrix3d tilt = Eigen::Quaterniond::FromTwoVectors(
        gravity, -Eigen::Vector3d::UnitZ()).toRotationMatrix();
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> local, global;
    std::vector<backend::GnssEvent, Eigen::aligned_allocator<backend::GnssEvent>> samples;
    // Only the latest uninterrupted FIX segment is used to initialize alignment.
    double previous = -1.0;
    for (const auto& sample : pending_) {
        std::size_t a, b;
        double alpha;
        if (!associate(sample.measurement_time, a, b, alpha)) continue;
        if (previous > 0.0 && sample.measurement_time - previous > config_.gnss_timeout) {
            local.clear(); global.clear(); samples.clear();
        }
        previous = sample.measurement_time;
        const gtsam::Pose3 raw = interpolatePose(nodes_[a].raw, nodes_[b].raw, alpha);
        local.push_back(tilt * raw.transformFrom(config_.lever));
        global.push_back(sample.position_enu);
        samples.push_back(sample);
    }
    if (samples.size() < config_.init_min_pairs ||
        samples.back().measurement_time - samples.front().measurement_time < config_.init_min_duration) {
        reason_ = "WAITING_FOR_MOTION_AND_FIX";
        return false;
    }
    Eigen::Vector3d lc = Eigen::Vector3d::Zero(), gc = Eigen::Vector3d::Zero();
    for (std::size_t i = 0; i < local.size(); ++i) { lc += local[i]; gc += global[i]; }
    lc /= local.size(); gc /= global.size();
    double dot = 0.0, cross = 0.0, energy_local = 0.0, energy_global = 0.0, baseline = 0.0;
    for (std::size_t i = 0; i < local.size(); ++i) {
        const Eigen::Vector2d l = (local[i] - lc).head<2>(), g = (global[i] - gc).head<2>();
        dot += l.dot(g);
        cross += l.x() * g.y() - l.y() * g.x();
        energy_local += l.squaredNorm(); energy_global += g.squaredNorm();
        baseline = std::max(baseline, (local[i] - local.front()).head<2>().norm());
    }
    const double correlation = std::hypot(dot, cross) /
        std::sqrt(std::max(1e-12, energy_local * energy_global));
    if (baseline < config_.init_baseline || energy_global < 1.0 ||
        correlation < config_.init_min_correlation) {
        reason_ = "INSUFFICIENT_HORIZONTAL_MOTION";
        return false;
    }
    const Eigen::Matrix3d yaw = Eigen::AngleAxisd(std::atan2(cross, dot), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d translation = gc - yaw * lc;
    double squared_error = 0.0;
    for (std::size_t i = 0; i < local.size(); ++i)
        squared_error += (yaw * local[i] + translation - global[i]).squaredNorm();
    init_rms_ = std::sqrt(squared_error / local.size());
    if (!std::isfinite(init_rms_) || init_rms_ > config_.init_max_rms) {
        reason_ = "ALIGNMENT_INCONSISTENT";
        return false;
    }
    correction_ = gtsam::Pose3(gtsam::Rot3(yaw * tilt), translation);
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
    gtsam::Vector6 prior;
    prior << config_.prior_tilt_sigma, config_.prior_tilt_sigma, config_.prior_yaw_sigma,
             config_.prior_position_sigma, config_.prior_position_sigma, config_.prior_position_sigma;
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
        values.insert(i, correction_.compose(nodes_[i].raw));
        if (i == 0) graph.add(gtsam::PriorFactor<gtsam::Pose3>(0,
            correction_.compose(nodes_[0].raw), gtsam::noiseModel::Diagonal::Sigmas(prior)));
        else {
            gtsam::Vector6 noise;
            const double scale = std::sqrt(std::max(1.0,
                (nodes_[i].time - nodes_[i - 1].time) / config_.keyframe_max_time));
            noise.head<3>().setConstant(config_.odom_rotation_sigma * scale);
            noise.tail<3>().setConstant(config_.odom_position_sigma * scale);
            graph.add(gtsam::BetweenFactor<gtsam::Pose3>(i - 1, i,
                nodes_[i - 1].raw.between(nodes_[i].raw), gtsam::noiseModel::Diagonal::Sigmas(noise)));
        }
    }
    // Each sample contributes once. Initial samples are consumed, not replayed.
    for (const auto& sample : samples) {
        std::size_t a, b;
        double alpha;
        associate(sample.measurement_time, a, b, alpha);
        graph.add(boost::make_shared<AntennaFactor>(a, b, alpha, config_.lever,
            sample.position_enu, gnssNoise(sample, nodes_[b].time - nodes_[a].time)));
    }
    last_factor_time_ = samples.back().measurement_time;
    pending_.erase(std::remove_if(pending_.begin(), pending_.end(), [&](const backend::GnssEvent& sample) {
        return sample.measurement_time <= last_factor_time_;
    }), pending_.end());
    ready_ = true;
    update(graph, values);
    if (failed_) return false;
    gnss_factors_ += samples.size();
    for (const auto& sample : samples) factor_times_.push_back(sample.measurement_time);
    gnss_live_ = true;
    reason_ = "NORMAL";
    processPending();
    return true;
}

void Core::processPending()
{
    if (!ready_ || failed_) return;
    gtsam::NonlinearFactorGraph graph;
    std::size_t accepted = 0;
    std::vector<double> times;
    double factor_time = last_factor_time_;
    while (!pending_.empty()) {
        const auto sample = pending_.front();
        if (sample.measurement_time > nodes_.back().time) break;
        pending_.pop_front();
        std::size_t a, b;
        double alpha;
        if (!associate(sample.measurement_time, a, b, alpha) ||
            sample.measurement_time <= last_factor_time_ ||
            nodes_.back().time - sample.measurement_time > config_.pending_max_lag) {
            ++gnss_rejected_;
            reason_ = "GNSS_ASSOCIATION_REJECTED";
            continue;
        }
        const Eigen::Vector3d predicted = interpolatePose(nodes_[a].optimized,
            nodes_[b].optimized, alpha).transformFrom(config_.lever);
        const Eigen::Vector3d residual = predicted - sample.position_enu;
        const double interval = nodes_[b].time - nodes_[a].time;
        const Eigen::Vector3d variance = health::gnssVariance(sample, config_.gnss) +
            Eigen::Vector3d::Constant(config_.innovation_sigma * config_.innovation_sigma +
                std::pow(config_.interpolation_sigma_per_second * interval, 2));
        last_innovation_ = (residual.array().square() / variance.array()).sum();
        if (!std::isfinite(last_innovation_) || last_innovation_ > config_.innovation_chi2 ||
            residual.norm() > config_.innovation_max_distance) {
            gnss_rejected_ += 1 + pending_.size();
            pending_.clear();
            fix_streak_ = 0;
            gnss_live_ = false;
            reason_ = "GNSS_INNOVATION_REJECTED";
            break;
        }
        graph.add(boost::make_shared<AntennaFactor>(a, b, alpha, config_.lever,
            sample.position_enu, gnssNoise(sample, interval)));
        ++accepted;
        times.push_back(sample.measurement_time);
        factor_time = sample.measurement_time;
    }
    if (accepted) {
        update(graph, gtsam::Values());
        if (!failed_) {
            gnss_factors_ += accepted;
            factor_times_.insert(factor_times_.end(), times.begin(), times.end());
            last_factor_time_ = factor_time;
            // An earlier accepted batch must not hide a later rejected observation.
            if (fix_streak_ >= config_.gnss.acquire_frames) {
                gnss_live_ = true;
                reason_ = "NORMAL";
            }
        }
    }
}

void Core::update(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values)
{
    if (config_.max_graph_factors && !resources::fits(graphFactors(), graph.size(), config_.max_graph_factors)) {
        exhaustBudget("GRAPH_FACTOR_BUDGET_EXHAUSTED");
        return;
    }
    try {
        isam_->update(graph, values);
        isam_->update();
        const auto estimate = isam_->calculateEstimate();
        for (std::size_t i = 0; i < nodes_.size(); ++i) {
            nodes_[i].optimized = estimate.at<gtsam::Pose3>(i);
            if (!nodes_[i].optimized.matrix().allFinite()) throw std::runtime_error("nonfinite optimized pose");
        }
        correction_ = nodes_.back().optimized.compose(nodes_.back().raw.inverse());
        ++updates_;
    } catch (const std::exception& error) {
        failed_ = true;
        ready_ = false;
        reason_ = "OPTIMIZER_FAILED";
        ROS_ERROR("[online_backend] optimizer disabled: %s", error.what());
    }
}

void Core::exhaustBudget(const char* reason)
{
    failed_ = true;
    ready_ = gnss_live_ = false;
    pending_.clear();
    reason_ = reason;
}

std::size_t Core::cachedPoints() const
{
    std::size_t count = 0;
    for (const auto& node : nodes_) if (node.cloud) count += node.cloud->size();
    return count;
}

void Core::tick(double time)
{
    if (failed_) return;
    if (have_gnss_ && time - last_gnss_time_ > config_.gnss_timeout) {
        resetFix();
        reason_ = ready_ ? "GNSS_LOST" : "WAITING_FOR_MOTION_AND_FIX";
    } else if (ready_ && fix_streak_ >= config_.gnss.acquire_frames &&
               time - last_factor_time_ > config_.gnss_timeout) {
        gnss_live_ = false;
        reason_ = "WAITING_FOR_GNSS_ASSOCIATION";
    }
}

gtsam::Matrix66 Core::latestCovariance() const
{
    if (!ready_ || nodes_.empty()) return gtsam::Matrix66::Identity() * 1e6;
    try { return isam_->marginalCovariance(nodes_.size() - 1); }
    catch (const std::exception&) { return gtsam::Matrix66::Identity() * 1e6; }
}

PointCloudXYZRGB::Ptr Core::buildMap() const
{
    PointCloudXYZRGB::Ptr combined(new PointCloudXYZRGB());
    if (!ready_) return combined;
    for (const auto& node : nodes_) {
        if (!node.cloud) continue;
        PointCloudXYZRGB transformed;
        pcl::transformPointCloud(*node.cloud, transformed, node.optimized.matrix().cast<float>());
        *combined += transformed;
    }
    PointCloudXYZRGB::Ptr result(new PointCloudXYZRGB());
    pcl::VoxelGrid<PointTypeRGB> filter;
    filter.setLeafSize(config_.map_leaf, config_.map_leaf, config_.map_leaf);
    filter.setInputCloud(combined);
    filter.filter(*result);
    if (result->size() > config_.map_max_points) {
        PointCloudXYZRGB::Ptr sampled(new PointCloudXYZRGB());
        const double stride = static_cast<double>(result->size()) / config_.map_max_points;
        for (std::size_t i = 0; i < config_.map_max_points; ++i)
            sampled->push_back(result->points[static_cast<std::size_t>(i * stride)]);
        return sampled;
    }
    return result;
}

} // namespace online
