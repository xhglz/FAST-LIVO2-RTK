#include "online_backend.h"
#include <diagnostic_msgs/DiagnosticArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <chrono>

namespace online {
namespace {
void positive(ros::NodeHandle& nh, const std::string& name, double& value)
{
    nh.param<double>(name, value, value);
    if (!std::isfinite(value) || value <= 0.0) throw std::invalid_argument(name + " must be finite and positive");
}
void count(ros::NodeHandle& nh, const std::string& name, std::size_t& value)
{
    int n = static_cast<int>(value);
    nh.param<int>(name, n, n);
    if (n < 1) throw std::invalid_argument(name + " must be positive");
    value = n;
}
} // namespace

void loadConfig(ros::NodeHandle& nh, Config& c)
{
    nh.param<std::string>("frames/map", c.frames.map, c.frames.map);
    nh.param<std::string>("frames/odom", c.frames.odom, c.frames.odom);
    nh.param<std::string>("frames/imu", c.frames.imu, c.frames.imu);
    nh.param<std::string>("frames/gnss", c.frames.gnss, c.frames.gnss);
    if (c.frames.map.empty() || c.frames.odom.empty() || c.frames.imu.empty() ||
        c.frames.map == c.frames.odom || c.frames.map == c.frames.imu || c.frames.odom == c.frames.imu)
        throw std::invalid_argument("online backend requires distinct, nonempty map/odom/imu frames");
    std::vector<double> lever;
    nh.param<std::vector<double>>("gps/extrinsic_T", lever, std::vector<double>(3, 0.0));
    if (lever.size() != 3) throw std::invalid_argument("gps/extrinsic_T must have three components");
    c.lever = Eigen::Vector3d(lever[0], lever[1], lever[2]);
    if (!c.lever.allFinite()) throw std::invalid_argument("nonfinite antenna lever arm");
    health::loadGnssConfig(nh, c.gnss);
    count(nh, "health/frontend/min_matches", c.min_matches);
    positive(nh, "health/frontend/min_eigen_ratio", c.min_eigen_ratio);
    positive(nh, "health/frontend/max_normalized_rms", c.max_normalized_rms);
    const std::string p = "backend/online/";
    positive(nh, p + "keyframe_distance", c.keyframe_distance);
    positive(nh, p + "keyframe_rotation", c.keyframe_rotation);
    positive(nh, p + "keyframe_max_time", c.keyframe_max_time);
    positive(nh, p + "keyframe_min_time", c.keyframe_min_time);
    count(nh, p + "init_min_pairs", c.init_min_pairs);
    positive(nh, p + "init_baseline", c.init_baseline);
    positive(nh, p + "init_min_duration", c.init_min_duration);
    positive(nh, p + "init_max_rms", c.init_max_rms);
    positive(nh, p + "init_min_correlation", c.init_min_correlation);
    positive(nh, p + "gravity_tolerance", c.gravity_tolerance);
    positive(nh, p + "association_max_gap", c.association_max_gap);
    positive(nh, p + "pending_max_lag", c.pending_max_lag);
    positive(nh, p + "odom_rotation_sigma", c.odom_rotation_sigma);
    positive(nh, p + "odom_position_sigma", c.odom_position_sigma);
    positive(nh, p + "prior_tilt_sigma", c.prior_tilt_sigma);
    positive(nh, p + "prior_yaw_sigma", c.prior_yaw_sigma);
    positive(nh, p + "prior_position_sigma", c.prior_position_sigma);
    positive(nh, p + "interpolation_sigma_per_second", c.interpolation_sigma_per_second);
    positive(nh, p + "innovation_sigma", c.innovation_sigma);
    positive(nh, p + "innovation_chi2", c.innovation_chi2);
    positive(nh, p + "innovation_max_distance", c.innovation_max_distance);
    positive(nh, p + "gnss_timeout", c.gnss_timeout);
    positive(nh, p + "map_period", c.map_period);
    positive(nh, p + "map_leaf", c.map_leaf);
    count(nh, p + "max_queue", c.max_queue);
    count(nh, p + "max_pending_gnss", c.max_pending_gnss);
    count(nh, p + "max_init_frames", c.max_init_frames);
    count(nh, p + "cloud_cache", c.cloud_cache);
    count(nh, p + "cloud_max_points", c.cloud_max_points);
    count(nh, p + "map_max_points", c.map_max_points);
    if (c.init_min_pairs < 3 || c.max_init_frames < 2 || c.init_min_correlation > 1.0 ||
        c.keyframe_min_time > c.keyframe_max_time || c.keyframe_max_time > c.association_max_gap)
        throw std::invalid_argument("inconsistent online initialization or association limits");
    nh.param<std::string>("laserMapping/outputfilepath", c.output_directory, "");
}

Backend::Backend(ros::NodeHandle& nh)
{
    loadConfig(nh, config_);
    nh.param<bool>("gps/gps_en", gps_enabled_, false);
    bool debug = false;
    nh.param<bool>("opt/debug_mode", debug, false);
    if (debug) throw std::invalid_argument("online mode does not import legacy logs; select backend/mode=batch");
    core_.reset(new Core(config_));
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odometry/global", 20);
    path_pub_ = nh.advertise<nav_msgs::Path>("/path/global", 1, true);
    map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/map/global", 1, true);
    status_pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10, true);
    if (!config_.output_directory.empty()) {
        const auto directory = std::filesystem::path(config_.output_directory) / "online";
        std::filesystem::create_directories(directory);
        events_file_.open((directory / "events.csv").string());
        trajectory_file_.open((directory / "global_trajectory.txt").string());
        factors_file_.open((directory / "gnss_factors.csv").string());
        if (!events_file_ || !trajectory_file_ || !factors_file_)
            throw std::runtime_error("cannot open online backend logs");
        events_file_ << "type,time,reason,ready,gnss_live,nodes,gnss_factors,rejected_gnss,updates,"
                        "last_factor_time,innovation,init_rms,tx,ty,tz,qx,qy,qz,qw\n";
        factors_file_ << "measurement_time\n";
    }
    ROS_INFO("[online_backend] iSAM2 worker enabled; no Enter trigger, fixed timing and lever arm.");
    worker_ = std::thread(&Backend::run, this);
}

Backend::~Backend()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        shutdown_ = true;
    }
    wake_.notify_one();
    if (worker_.joinable()) worker_.join();
}

void Backend::onKeyframe(const backend::KeyframeEvent& event)
{
    if (!backend::validKeyframe(event) || event.pose_frame != config_.frames.odom ||
        event.body_frame != config_.frames.imu) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (shutdown_ || (have_latest_ && event.measurement_time <= latest_.measurement_time)) return;
    latest_ = event;
    latest_.cloud_imu.reset();
    have_latest_ = true;
    const bool selected = selectable(event, config_) &&
        (!have_selected_ || shouldSelect(last_selected_, last_selected_time_, event, config_));
    if (!selected && !backend::validRemovals(event)) {
        wake_.notify_one();
        return;
    }
    if (queue_.size() >= config_.max_queue) { ++dropped_; return; }
    Event input;
    input.key = event;
    input.key.cloud_imu.reset(new PointCloudXYZRGB(selected ? *event.cloud_imu : PointCloudXYZRGB()));
    queue_.push_back(std::move(input));
    if (selected) {
        last_selected_ = gtsam::Pose3(event.T_odom_imu.matrix());
        last_selected_time_ = event.measurement_time;
        have_selected_ = true;
    }
    wake_.notify_one();
}

void Backend::onGnss(const backend::GnssEvent& event)
{
    if (!gps_enabled_) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (shutdown_) return;
    if (queue_.size() >= config_.max_queue) { ++dropped_; return; }
    Event input;
    input.is_gnss = true;
    input.gps = event;
    queue_.push_back(std::move(input));
    wake_.notify_one();
}

void Backend::record(const std::string& type, double time)
{
    if (events_file_) {
        const auto& T = core_->correction();
        const auto q = T.rotation().toQuaternion();
        events_file_ << std::setprecision(17) << type << "," << time << "," << core_->reason()
            << "," << core_->ready() << "," << core_->gnssLive() << "," << core_->nodes().size()
            << "," << core_->gnssFactors() << "," << core_->rejectedGnss() << "," << core_->updates()
            << "," << core_->lastFactorTime() << "," << core_->lastInnovation() << "," << core_->initRms()
            << "," << T.x() << "," << T.y() << "," << T.z()
            << "," << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << "\n";
        events_file_.flush();
    }
    while (logged_factors_ < core_->factorTimes().size()) {
        if (factors_file_)
            factors_file_ << std::setprecision(17) << core_->factorTimes()[logged_factors_] << "\n";
        ++logged_factors_;
    }
    if (factors_file_) factors_file_.flush();
}

void Backend::run()
{
    for (;;) {
        Event input;
        bool have_input = false, have_latest;
        backend::KeyframeEvent latest;
        std::size_t depth, dropped;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            wake_.wait_for(lock, std::chrono::milliseconds(100), [&] { return shutdown_ || !queue_.empty(); });
            if (shutdown_) break;
            if (!queue_.empty()) {
                input = std::move(queue_.front());
                queue_.pop_front();
                have_input = true;
            }
            latest = latest_;
            have_latest = have_latest_;
            depth = queue_.size();
            dropped = dropped_;
        }
        if (have_input) {
            try {
                if (input.is_gnss) core_->addGnss(input.gps);
                else {
                    core_->addKeyframe(input.key);
                    if (backend::validRemovals(input.key)) last_map_time_ = 0.0;
                }
                record(input.is_gnss ? "gnss" : "keyframe",
                       input.is_gnss ? input.gps.measurement_time : input.key.measurement_time);
            } catch (const std::exception& error) {
                ROS_ERROR("[online_backend] event processing failed: %s", error.what());
                // Never continue from a partially modified incremental graph.
                core_->disable();
            }
        }
        const double now = ros::Time::now().toSec();
        // Do not declare loss while accepted events are still waiting to be processed.
        if (depth == 0) core_->tick(now);
        const bool local_healthy = have_latest && latest.quality.lidar_update_accepted &&
            latest.quality.lidar_matches >= config_.min_matches &&
            latest.quality.eigen_ratio >= config_.min_eigen_ratio &&
            std::isfinite(latest.quality.normalized_residual_rms) &&
            latest.quality.normalized_residual_rms <= config_.max_normalized_rms;
        try {
            publish(latest, now, local_healthy, depth, dropped);
        } catch (const std::exception& error) {
            core_->disable();
            ROS_ERROR("[online_backend] output failed, global output disabled: %s", error.what());
        }
    }
}

void Backend::publish(const backend::KeyframeEvent& latest, double now, bool local_healthy,
    std::size_t queue_depth, std::size_t dropped)
{
    const bool local_recent = latest.measurement_time > 0.0 &&
        now - latest.measurement_time <= config_.pending_max_lag;
    const bool valid = core_->ready() && !core_->failed() && local_recent && local_healthy;
    if (now > 0.0 && now >= last_status_time_ + 0.2) {
        last_status_time_ = now;
        diagnostic_msgs::DiagnosticArray message;
        message.header.stamp.fromSec(now);
        diagnostic_msgs::DiagnosticStatus status;
        status.name = "fast_livo/online_backend";
        status.hardware_id = "FAST-LIVO2-RTK";
        const std::string reason = !gps_enabled_ ? "GNSS_DISABLED" :
            core_->failed() ? "OPTIMIZER_FAILED" :
            !local_recent ? "LOCAL_ODOMETRY_MISSING" :
            !local_healthy ? "LOCAL_QUALITY_LOW" : core_->reason();
        status.level = core_->failed() || (core_->ready() && !valid) ? 2 :
            !core_->ready() || !core_->gnssLive() || reason != "NORMAL" ? 1 : 0;
        status.message = reason;
        auto value = [&](const std::string& key, const auto& data) {
            diagnostic_msgs::KeyValue kv;
            kv.key = key;
            std::ostringstream text;
            text << std::setprecision(12) << data;
            kv.value = text.str();
            status.values.push_back(kv);
        };
        value("alignment_ready", core_->ready());
        value("global_pose_valid", valid);
        value("gnss_live", core_->gnssLive());
        value("nodes", core_->nodes().size());
        value("gnss_factors", core_->gnssFactors());
        value("gnss_rejected", core_->rejectedGnss());
        value("updates", core_->updates());
        value("last_gnss_factor_time", core_->lastFactorTime());
        value("innovation_score", core_->lastInnovation());
        value("initial_alignment_rms", core_->initRms());
        value("queue_depth", queue_depth);
        value("queue_dropped", dropped);
        value("pending_gnss", core_->pending());
        value("cloud_cache_limit", config_.cloud_cache);
        message.status.push_back(status);
        status_pub_.publish(message);
    }
    if (!valid || latest.measurement_time <= last_pose_stamp_) return;
    last_pose_stamp_ = latest.measurement_time;
    const gtsam::Pose3 global = core_->correction().compose(gtsam::Pose3(latest.T_odom_imu.matrix()));
    const auto q = global.rotation().toQuaternion();
    const auto correction_q = core_->correction().rotation().toQuaternion();
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(core_->correction().x(), core_->correction().y(), core_->correction().z()));
    transform.setRotation(tf::Quaternion(correction_q.x(), correction_q.y(), correction_q.z(), correction_q.w()));
    const ros::Time stamp(latest.measurement_time);
    tf_.sendTransform(tf::StampedTransform(transform, stamp, config_.frames.map, config_.frames.odom));
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = config_.frames.map;
    odom.child_frame_id = config_.frames.imu;
    odom.pose.pose.position.x = global.x();
    odom.pose.pose.position.y = global.y();
    odom.pose.pose.position.z = global.z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    // Propagate the latest optimized anchor covariance through fixed local motion.
    const auto& anchor = core_->nodes().back();
    const auto relative = anchor.raw.between(gtsam::Pose3(latest.T_odom_imu.matrix()));
    gtsam::Matrix66 H;
    anchor.optimized.compose(relative, H, boost::none);
    gtsam::Matrix66 covariance = H * core_->latestCovariance() * H.transpose();
    const double dt = std::max(0.0, latest.measurement_time - anchor.time);
    covariance.diagonal().head<3>().array() += config_.odom_rotation_sigma * config_.odom_rotation_sigma * dt;
    covariance.diagonal().tail<3>().array() += config_.odom_position_sigma * config_.odom_position_sigma * dt;
    gtsam::Matrix66 A = gtsam::Matrix66::Zero();
    A.block<3, 3>(0, 3) = global.rotation().matrix();
    A.block<3, 3>(3, 0) = global.rotation().matrix();
    const gtsam::Matrix66 ros_covariance = A * covariance * A.transpose();
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c) odom.pose.covariance[r * 6 + c] = ros_covariance(r, c);
    const Eigen::Vector3d body_velocity = latest.T_odom_imu.linear().transpose() * latest.velocity_odom;
    odom.twist.twist.linear.x = body_velocity.x();
    odom.twist.twist.linear.y = body_velocity.y();
    odom.twist.twist.linear.z = body_velocity.z();
    for (int i = 0; i < 6; ++i) odom.twist.covariance[i * 7] = 1e6;
    odom_pub_.publish(odom);
    if (trajectory_file_) {
        trajectory_file_ << std::setprecision(17) << latest.measurement_time << " "
            << global.x() << " " << global.y() << " " << global.z() << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
        trajectory_file_.flush();
    }
    if (core_->updates() != last_path_update_) {
        last_path_update_ = core_->updates();
        nav_msgs::Path path;
        path.header = odom.header;
        for (const auto& node : core_->nodes()) {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = config_.frames.map;
            pose.header.stamp.fromSec(node.time);
            pose.pose.position.x = node.optimized.x();
            pose.pose.position.y = node.optimized.y();
            pose.pose.position.z = node.optimized.z();
            const auto rotation = node.optimized.rotation().toQuaternion();
            pose.pose.orientation.x = rotation.x(); pose.pose.orientation.y = rotation.y();
            pose.pose.orientation.z = rotation.z(); pose.pose.orientation.w = rotation.w();
            path.poses.push_back(pose);
        }
        path_pub_.publish(path);
    }
    if (map_pub_.getNumSubscribers() && now >= last_map_time_ + config_.map_period) {
        last_map_time_ = now;
        const auto cloud = core_->buildMap();
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header = odom.header;
        map_pub_.publish(msg);
    }
}

} // namespace online
