#include "LIVMapper.h"
#include <filesystem>
#include <iomanip>

void LIVMapper::initializeSafety(ros::NodeHandle& nh)
{
    safety::loadConfig(nh, safety_config);
    original_odom_frame = frames.odom;
    std::string backend_mode;
    nh.param<std::string>("backend/mode", backend_mode, "batch");
    safety_online_backend = backend_mode == "online";
    if (!safety_config.enabled) return;
    safety_machine.reset(new safety::Machine(safety_config));
    safety_relocalizer.reset(new safety::Relocalizer(safety_config));
    localization_status_pub = nh.advertise<fast_livo::LocalizationStatus>("/localization/status", 20, true);
    local_debug_pub = nh.advertise<nav_msgs::Odometry>("/odometry/local_debug", 20);
    reinitialize_service = nh.advertiseService("/localization/reinitialize", &LIVMapper::reinitialize, this);
    const std::string directory = save_directory.empty() ? std::string(ROOT_DIR) + "Log/" : save_directory + "/debug/";
    safety_file.open(directory + "localization_state.csv");
    segment_file.open(directory + "localization_segments.csv");
    if (!safety_file || !segment_file) throw std::runtime_error("cannot open localization safety logs");
    safety_file << "time,segment,state,reason,pose_valid,map_write,map_revision,imu_ready,recovery,submap_points,x,y,z,qx,qy,qz,qw\n";
    segment_file << "time,segment,odom_frame,reason\n";
    segment_file << "0,0," << frames.odom << ",INITIAL_SEGMENT\n";
}

bool LIVMapper::safetySensor(health::Sensor sensor) const
{
    const auto& s = input_monitor.inputs[static_cast<int>(sensor)];
    if (!s.config.enabled) return true;
    const double age = ros::Time::now().toSec() - s.last_time;
    const double limit = sensor == health::Sensor::CAMERA ?
        std::min(s.config.max_age, safety_config.max_prediction_time * 0.5) : s.config.max_age;
    return input_monitor.safe(sensor) && age <= limit && age >= -s.config.future_tolerance;
}

bool LIVMapper::trustedOutputAllowed() const
{
    return !safety_machine || (safety_machine->poseValid() &&
        ros::Time::now().toSec() - LidarMeasures.last_lio_update_time <= safety_config.max_prediction_time);
}

void LIVMapper::assessSafety()
{
    if (!safety_machine) return;
    safety::Evidence e;
    e.time = keyframe_time;
    e.finite = _state.pos_end.allFinite() && _state.rot_end.allFinite() &&
        _state.vel_end.allFinite() && _state.cov.allFinite() && _state.cov.diagonal().minCoeff() >= 0.0 &&
        _state.bias_a.allFinite() && _state.bias_g.allFinite() && _state.gravity.allFinite() &&
        std::isfinite(_state.inv_expo_time);
    e.imu = safety_imu_ready && safetySensor(health::Sensor::IMU);
    e.lidar = safetySensor(health::Sensor::LIDAR);
    e.initialized = !p_imu->imu_need_init;
    e.camera = !img_en || safetySensor(health::Sensor::CAMERA);
    e.gnss = !gps_enabled || (safetySensor(health::Sensor::GNSS) && gnss_monitor.last.stable);
    e.position_sigma = std::sqrt(std::max(0.0, _state.cov.block<3, 3>(3, 3).diagonal().maxCoeff()));
    e.rotation_sigma = std::sqrt(std::max(0.0, _state.cov.block<3, 3>(0, 0).diagonal().maxCoeff()));
    const auto& c = frontend_monitor.config;
    e.geometry = last_quality.lidar_matches >= c.min_matches &&
        last_quality.match_ratio >= c.min_match_ratio && last_quality.angular_coverage >= c.min_coverage &&
        std::isfinite(last_quality.normalized_residual_rms) && last_quality.normalized_residual_rms <= c.max_normalized_rms &&
        last_quality.eigen_ratio >= c.min_eigen_ratio &&
        last_quality.translation_correction <= c.max_translation_correction &&
        last_quality.rotation_correction <= c.max_rotation_correction;
    e.relocalized = recovery_candidate;
    const bool accepted = e.finite && e.imu && e.lidar && e.geometry;
    safety_machine->observe(e);
    if (!accepted) {
        _state = state_propagat;
        voxelmap_manager->state_ = _state;
        recovery_candidate = false;
    }
    // INITIALIZING/RECOVERING may evaluate a candidate, but never write trusted maps.
    lio_update_allowed = accepted;
    last_quality.lidar_update_accepted = accepted;
    last_quality.local_pose_valid = safety_machine->poseValid();
    if (safety_machine->poseValid()) {
        trusted_state = _state;
        trusted_state_time = keyframe_time;
        recovery_candidate = false;
        safety::NormalCloud::Ptr snapshot(new safety::NormalCloud);
        for (const auto& match : voxelmap_manager->ptpl_list_) {
            if (dynamic_filter_engine && !dynamic_filter_engine->permanentAllowed(match.point_w_)) continue;
            pcl::PointNormal p;
            p.x = match.point_w_.x(); p.y = match.point_w_.y(); p.z = match.point_w_.z();
            p.normal_x = match.normal_.x(); p.normal_y = match.normal_.y(); p.normal_z = match.normal_.z();
            snapshot->push_back(p);
        }
        safety_relocalizer->remember(snapshot, keyframe_time, _state.pos_end);
    }
    publishSafety(ros::Time::now().toSec());
}

void LIVMapper::recordSafety()
{
    if (!safety_machine) return;
    const Eigen::Quaterniond q(_state.rot_end);
    safety_file << std::setprecision(17) << keyframe_time << "," << safety_machine->segment() << ","
        << safety::name(safety_machine->state()) << "," << safety_machine->reason() << ","
        << safety_machine->poseValid() << "," << safety_machine->mapWritable() << ","
        << voxelmap_manager->map_revision_ << "," << safety_imu_ready
        << "," << recovery_reason << "," << safety_relocalizer->size() << ","
        << _state.pos_end.x() << "," << _state.pos_end.y() << "," << _state.pos_end.z() << ","
        << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << "\n";
    safety_file.flush();
}

void LIVMapper::publishSafety(double now)
{
    if (!safety_machine) return;
    safety_machine->tick(now, safetySensor(health::Sensor::IMU), safetySensor(health::Sensor::LIDAR));
    fast_livo::LocalizationStatus status;
    status.header.stamp.fromSec(now);
    status.header.frame_id = frames.odom;
    status.measurement_time.fromSec(std::max(0.0, LidarMeasures.last_lio_update_time));
    status.state = static_cast<std::uint8_t>(safety_machine->state());
    status.segment_id = safety_machine->segment();
    status.pose_valid = trustedOutputAllowed();
    status.map_write_allowed = safety_machine->mapWritable() && status.pose_valid;
    const double ps = std::sqrt(std::max(0.0, _state.cov.block<3, 3>(3, 3).diagonal().maxCoeff()));
    const double rs = std::sqrt(std::max(0.0, _state.cov.block<3, 3>(0, 0).diagonal().maxCoeff()));
    status.imu_prediction_allowed = safety_machine->predictionAllowed(now, ps, rs);
    status.camera_available = img_en && safetySensor(health::Sensor::CAMERA);
    status.gnss_available = gps_enabled && safetySensor(health::Sensor::GNSS) && gnss_monitor.last.stable;
    status.reason = safety_machine->reason();
    localization_status_pub.publish(status);
    if (_state.pos_end.allFinite() && _state.rot_end.allFinite()) {
        nav_msgs::Odometry debug;
        debug.header.stamp = status.measurement_time;
        debug.header.frame_id = frames.odom;
        debug.child_frame_id = frames.imu;
        debug.pose.pose.position.x = _state.pos_end.x();
        debug.pose.pose.position.y = _state.pos_end.y();
        debug.pose.pose.position.z = _state.pos_end.z();
        const Eigen::Quaterniond q(_state.rot_end);
        debug.pose.pose.orientation.x = q.x(); debug.pose.pose.orientation.y = q.y();
        debug.pose.pose.orientation.z = q.z(); debug.pose.pose.orientation.w = q.w();
        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> covariance(debug.pose.covariance.data());
        if (status.pose_valid)
            covariance = backend::poseCovarianceToRos(_state.rot_end, _state.cov.block<6, 6>(0, 0));
        else
            covariance = Eigen::Matrix<double, 6, 6>::Identity() * 1e6;
        local_debug_pub.publish(debug);
    }
}

bool LIVMapper::reinitialize(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response)
{
    // Calling this service is an operator acknowledgement of externally verified standstill.
    if (!safety_online_backend) { response.message = "segment reinitialization requires online backend"; return true; }
    if (!safety_machine || safety_machine->state() != safety::State::LOST ||
        stationary_imu.size() < 20 || !safetySensor(health::Sensor::IMU)) {
        response.message = "requires LOST and fresh IMU; externally confirm standstill before calling";
        return true;
    }
    V3D mean = V3D::Zero();
    for (const auto& imu : stationary_imu) {
        if (V3D(imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z).norm() > 0.08) {
            response.message = "IMU rotation is not stationary"; return true;
        }
        mean += V3D(imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);
    }
    mean /= stationary_imu.size();
    for (const auto& imu : stationary_imu)
        if ((V3D(imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z) - mean).norm() > 0.3) {
            response.message = "IMU acceleration is not stationary"; return true;
        }
    if (mean.norm() < 0.1 || stationary_imu.back().header.stamp.toSec() - stationary_imu.front().header.stamp.toSec() < 0.4) {
        response.message = "insufficient stationary IMU window"; return true;
    }
    const auto archive = std::filesystem::path(save_directory.empty() ? std::string(ROOT_DIR) + "Log" : save_directory) / "segments";
    std::filesystem::create_directories(archive);
    const std::string prefix = (archive / std::to_string(safety_machine->segment())).string();
    if (!pcl_wait_save->empty() && pcl::io::savePCDFileBinary(prefix + "_rgb.pcd", *pcl_wait_save) != 0) {
        response.message = "cannot archive colored segment map"; return true;
    }
    if (!pcl_wait_save_intensity->empty() && pcl::io::savePCDFileBinary(prefix + "_intensity.pcd", *pcl_wait_save_intensity) != 0) {
        response.message = "cannot archive intensity segment map"; return true;
    }
    safety_machine->newSegment();
    frames.odom = backend::segmentFrame(original_odom_frame, safety_machine->segment());
    voxelmap_manager->map_frame_ = frames.odom;
    for (auto& root : voxelmap_manager->voxel_map_) delete root.second;
    voxelmap_manager->voxel_map_.clear();
    voxelmap_manager->replaceTemporaryMap({});
    voxelmap_manager->estimation_mask_.clear();
    if (dynamic_filter_engine) dynamic_filter_engine.reset(new dynamic_filter::Filter(dynamic_config));
    vio_manager->static_point_allowed = [](const V3D&) { return false; };
    vio_manager->pruneNonStaticPoints();
    vio_manager->static_point_allowed = dynamic_filter_engine ?
        std::function<bool(const V3D&)>([this](const V3D& p) { return dynamic_filter_engine->permanentAllowed(p); }) :
        std::function<bool(const V3D&)>();
    safety_relocalizer->clear();
    _state = trusted_state = StatesGroup();
    p_imu->Reset();
    state_propagat = _state;
    voxelmap_manager->state_ = _state;
    LidarMeasures = LidarMeasureGroup();
    {
        std::lock_guard<std::mutex> lock(mtx_buffer);
        imu_buffer.clear(); img_buffer.clear(); img_time_buffer.clear();
        lid_raw_data_buffer.clear(); lid_header_time_buffer.clear();
    }
    {
        std::lock_guard<std::mutex> lock(mtx_buffer_imu_prop);
        prop_imu_buffer.clear(); ekf_finish_once = false; state_update_flg = false;
    }
    pcl_wait_pub->clear(); pcl_w_wait_pub->clear();
    pcl_wait_save->clear(); pcl_wait_save_intensity->clear();
    feats_undistort->clear();
    pending_map_removals.clear();
    path.poses.clear(); path.header.frame_id = frames.odom;
    lidar_map_inited = false; lidar_pushed = false; gravity_align_finished = false;
    recovery_candidate = false; trusted_state_time = safety_process_time = recovery_pose_time = -1.0;
    frontend_monitor.frames = 0;
    frame_num = 0;
    backend::KeyframeEvent event;
    event.sequence = keyframe_sequence++;
    event.measurement_time = ros::Time::now().toSec();
    event.segment_id = safety_machine->segment();
    event.segment_start = true;
    event.pose_frame = frames.odom;
    event.body_frame = frames.imu;
    event.cloud_imu.reset(new PointCloudXYZRGB());
    event.quality.local_pose_valid = event.quality.lidar_update_accepted = false;
    if (keyframe_callback) keyframe_callback(event);
    segment_file << std::setprecision(17) << event.measurement_time << "," << event.segment_id
                 << "," << frames.odom << ",OPERATOR_REINITIALIZATION\n";
    segment_file.flush();
    response.success = true;
    response.message = "new local segment " + frames.odom + "; global alignment remains unavailable until trusted GNSS initialization";
    publishSafety(ros::Time::now().toSec());
    return true;
}
