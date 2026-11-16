#include "LIVMapper.h"
#include <iomanip>
#include <sstream>

bool LIVMapper::checkInput(health::Sensor sensor, double time, bool finite, bool saturated)
{
  const double now = ros::Time::now().toSec();
  if (!input_monitor.observe(sensor, time, now, finite, saturated)) {
    ROS_WARN_THROTTLE(1.0, "[input/%s] invalid or non-increasing measurement rejected", health::name(sensor));
    return false;
  }
  if (enforce_input_quality && !input_monitor.safe(sensor)) {
    ++input_monitor.inputs[static_cast<int>(sensor)].rejected;
    return false;
  }
  if (safety_machine && sensor == health::Sensor::IMU && saturated) return false;
  return true;
}

void LIVMapper::collectFrontendQuality()
{
  last_quality = backend::Quality();
  last_quality.static_filter_available = dynamic_config.enabled;
  last_quality.pose_covariance = _state.cov.block<6, 6>(0, 0);
  last_quality.lidar_points = feats_down_size;
  last_quality.lidar_matches = voxelmap_manager->lidar_match_count_;
  last_quality.lidar_information = voxelmap_manager->lidar_information_;
  last_quality.lidar_residual_rms = voxelmap_manager->lidar_residual_rms_;
  last_quality.normalized_residual_rms = voxelmap_manager->lidar_normalized_rms_;
  last_quality.match_ratio = feats_down_size > 0 ?
      static_cast<double>(last_quality.lidar_matches) / feats_down_size : 0.0;
  last_quality.angular_coverage = voxelmap_manager->lidar_angular_coverage_;
  last_quality.translation_correction = (_state.pos_end - state_propagat.pos_end).norm();
  last_quality.rotation_correction = Eigen::AngleAxisd(state_propagat.rot_end.transpose() * _state.rot_end).angle();
  frontend_monitor.assess(last_quality);
  const bool finite = _state.pos_end.allFinite() && _state.rot_end.allFinite() &&
      _state.vel_end.allFinite() && _state.bias_g.allFinite() && _state.bias_a.allFinite() &&
      _state.gravity.allFinite() && std::isfinite(_state.inv_expo_time) && _state.cov.allFinite();
  lio_update_allowed = finite && (!enforce_frontend ||
      (frontend_monitor.health.state != health::State::DEGRADED &&
       input_monitor.safe(health::Sensor::IMU) && input_monitor.safe(health::Sensor::LIDAR)));
  last_quality.lidar_update_accepted = lio_update_allowed;
  if (!lio_update_allowed) {
    ++frontend_monitor.rejected;
    _state = state_propagat;
    voxelmap_manager->state_ = state_propagat;
    last_quality.pose_covariance = _state.cov.block<6, 6>(0, 0);
  }
  assessSafety();
  if (frontend_quality_file.is_open()) {
    frontend_quality_file << std::setprecision(17) << LidarMeasures.last_lio_update_time << ","
      << health::name(frontend_monitor.health.state) << "," << frontend_monitor.reason << ","
      << last_quality.lidar_points << "," << last_quality.lidar_matches << ","
      << last_quality.match_ratio << "," << last_quality.angular_coverage << ","
      << last_quality.lidar_residual_rms << "," << last_quality.normalized_residual_rms << ","
      << last_quality.translation_correction << "," << last_quality.rotation_correction << ","
      << last_quality.eigen_ratio << "," << lio_update_allowed << "," << dynamic_config.enabled;
    for (int i = 0; i < 6; ++i) frontend_quality_file << "," << last_quality.information_eigenvalues[i];
    for (int r = 0; r < 6; ++r)
      for (int c = 0; c < 6; ++c) frontend_quality_file << "," << last_quality.scaled_information(r, c);
    frontend_quality_file << "\n";
    frontend_quality_file.flush();
  }
}

void LIVMapper::recordGnssDiagnostic(const backend::GnssEvent& event, bool forwarded)
{
  if (!gnss_quality_file.is_open()) return;
  gnss_quality_file << std::setprecision(17) << event.sequence << "," << event.measurement_time
    << "," << event.raw_unix_time << "," << event.classification << ","
    << event.valid_fix << "," << static_cast<int>(event.carrier_solution) << ","
    << static_cast<int>(event.satellites) << "," << event.horizontal_accuracy << ","
    << event.vertical_accuracy << "," << event.observation_age << "," << event.innovation_score
    << "," << event.position_variance.x() << "," << event.position_variance.z() << ","
    << event.trusted_fix << "," << event.stable_fix << "," << forwarded << ","
    << event.position_enu.x() << "," << event.position_enu.y() << "," << event.position_enu.z() << "\n";
  gnss_quality_file.flush();
}

void LIVMapper::diagnosticsCallback(const ros::WallTimerEvent&)
{
  const double now = ros::Time::now().toSec();
  if (now <= 0.0 || now < last_diagnostic_time + 0.1) return;
  last_diagnostic_time = now;
  {
    std::lock_guard<std::mutex> lock(mtx_buffer);
    auto queue = [&](health::Sensor sensor, std::size_t count, double oldest) {
      auto& status = input_monitor.inputs[static_cast<int>(sensor)];
      status.queue = count;
      status.queue_age = count ? std::max(0.0, now - oldest) : 0.0;
    };
    queue(health::Sensor::IMU, imu_buffer.size(),
          imu_buffer.empty() ? now : imu_buffer.front()->header.stamp.toSec());
    queue(health::Sensor::LIDAR, lid_raw_data_buffer.size(),
          lid_header_time_buffer.empty() ? now : lid_header_time_buffer.front());
    queue(health::Sensor::CAMERA, img_buffer.size(),
          img_time_buffer.empty() ? now : img_time_buffer.front());
    // Accepted GNSS history is not a pending input queue.
    queue(health::Sensor::GNSS, rtk_buffer.size(), rtk_buffer.empty() ? now : rtk_buffer.front().timestamp);
  }
  {
    std::lock_guard<std::mutex> lock(mtx_buffer_imu_prop);
    auto& status = input_monitor.inputs[0];
    status.queue = std::max(status.queue, prop_imu_buffer.size());
    if (!prop_imu_buffer.empty())
      status.queue_age = std::max(status.queue_age, std::max(0.0, now - prop_imu_buffer.front().header.stamp.toSec()));
  }
  input_monitor.poll(now);
  publishSafety(now);
  const auto& gps_input = input_monitor.inputs[3];
  if (gps_input.seen) gnss_monitor.last.age = now - gps_input.last_time;
  if (gps_enabled && gps_input.flags & (health::MISSING | health::DELAYED | health::FUTURE))
    gnss_monitor.unavailable(health::reasons(gps_input.flags));

  diagnostic_msgs::DiagnosticArray message;
  message.header.stamp.fromSec(now);
  auto make = [&](const std::string& component, health::State state, const std::string& reason) {
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "fast_livo/" + component;
    status.hardware_id = "FAST-LIVO2-RTK";
    status.level = state == health::State::DEGRADED ? diagnostic_msgs::DiagnosticStatus::ERROR :
        (state == health::State::INITIALIZING || reason != "OK") ? diagnostic_msgs::DiagnosticStatus::WARN :
                                                               diagnostic_msgs::DiagnosticStatus::OK;
    status.message = std::string(health::name(state)) + ":" + reason;
    return status;
  };
  auto value = [](diagnostic_msgs::DiagnosticStatus& status, const std::string& key, const auto& data) {
    diagnostic_msgs::KeyValue kv;
    kv.key = key;
    std::ostringstream text;
    text << std::setprecision(12) << data;
    kv.value = text.str();
    status.values.push_back(kv);
  };
  for (int i = 0; i < 4; ++i) {
    const auto& input = input_monitor.inputs[i];
    if (!input.config.enabled) continue;
    auto status = make(std::string("input/") + health::name(static_cast<health::Sensor>(i)),
                       input.health.state, health::reasons(input.flags));
    value(status, "age", input.age);
    value(status, "queue_depth", input.queue);
    value(status, "queue_oldest_age", input.queue_age);
    value(status, "received", input.received);
    value(status, "rejected", input.rejected);
    value(status, "transitions", input.health.transitions);
    for (int bit = 0; bit < 8; ++bit)
      value(status, health::reasons(1u << bit) + "_count", input.counts[bit]);
    value(status, "enforce_input_quality", enforce_input_quality);
    message.status.push_back(status);
  }
  if (gps_enabled) {
    auto status = make("gnss_quality", gnss_monitor.health.state,
                       gnss_monitor.last.trusted ? "OK" : gnss_monitor.last.classification);
    value(status, "classification", gnss_monitor.last.classification);
    value(status, "age", gnss_monitor.last.age);
    value(status, "innovation_score", gnss_monitor.last.innovation_score);
    value(status, "forwarded", gnss_monitor.forwarded);
    value(status, "rejected", gnss_monitor.rejected);
    value(status, "stable_fix", gnss_monitor.last.stable);
    value(status, "enforce_fix", enforce_gnss_fix);
    value(status, "transitions", gnss_monitor.health.transitions);
    for (const auto& count : gnss_monitor.counts) value(status, count.first + "_count", count.second);
    message.status.push_back(status);
  }
  auto status = make("frontend", frontend_monitor.health.state, frontend_monitor.reason);
  value(status, "candidate_inliers", last_quality.lidar_matches);
  value(status, "static_filter_available", dynamic_config.enabled);
  value(status, "match_ratio", last_quality.match_ratio);
  value(status, "angular_coverage", last_quality.angular_coverage);
  value(status, "normalized_rms", last_quality.normalized_residual_rms);
  value(status, "translation_correction", last_quality.translation_correction);
  value(status, "rotation_correction", last_quality.rotation_correction);
  value(status, "eigen_ratio", last_quality.eigen_ratio);
  value(status, "transitions", frontend_monitor.health.transitions);
  value(status, "rejected", frontend_monitor.rejected);
  value(status, "enforce_updates", enforce_frontend);
  message.status.push_back(status);
  if (dynamic_filter_engine) {
    const auto& s = dynamic_filter_engine->stats();
    auto filter = make("dynamic_filter", dynamic_map_write_allowed ? health::State::NORMAL : health::State::INITIALIZING,
                       dynamic_map_write_allowed ? "OK" : "MAP_WRITE_FROZEN");
    value(filter, "map_write_allowed", dynamic_map_write_allowed);
    value(filter, "unknown", s.points[0]);
    value(filter, "tentative_static", s.points[1]);
    value(filter, "confirmed_static", s.points[2]);
    value(filter, "dynamic", s.points[3]);
    value(filter, "moving_clusters", s.moving_clusters);
    value(filter, "removed_cells", s.removed);
    value(filter, "history_cells", s.cells);
    value(filter, "capacity_rejected", s.capacity_rejected);
    value(filter, "temporary_roots", voxelmap_manager->temporary_map_.size());
    if (s.capacity_rejected) {
      filter.level = diagnostic_msgs::DiagnosticStatus::WARN;
      filter.message = "HISTORY_CAPACITY_REACHED";
    }
    message.status.push_back(filter);
  }
  if (resource_config.enabled) {
    auto resource = make("resources", resource_archive_failed ? health::State::DEGRADED : health::State::NORMAL,
        resource_archive_failed ? "ARCHIVE_FAILED" : "OK");
    value(resource, "map_roots", voxelmap_manager->voxel_map_.size());
    value(resource, "map_root_limit", resource_config.map_roots);
    value(resource, "map_rejected_points", voxelmap_manager->resource_rejected_points);
    value(resource, "visual_points", vio_manager->resource_point_count);
    value(resource, "path_poses", path.poses.size());
    value(resource, "imu_dropped", resource_dropped[0]);
    value(resource, "lidar_dropped", resource_dropped[1]);
    value(resource, "camera_dropped", resource_dropped[2]);
    value(resource, "pcd_chunks", resource_pcd_chunks);
    value(resource, "pcd_rejected_points", resource_pcd_rejected);
    if (!resource_archive_failed && (resource_dropped[0] || resource_dropped[1] || resource_dropped[2] ||
        voxelmap_manager->resource_rejected_points)) {
      resource.level = diagnostic_msgs::DiagnosticStatus::WARN;
      resource.message = "CAPACITY_REACHED";
    }
    message.status.push_back(resource);
  }
  diagnostics_pub.publish(message);
  for (const auto& item : message.status) {
    if (diagnostics_file.is_open()) {
      diagnostics_file << std::setprecision(17) << now << "," << item.name << ","
                       << static_cast<int>(item.level) << "," << item.message << ",";
      for (const auto& kv : item.values) diagnostics_file << kv.key << "=" << kv.value << ";";
      diagnostics_file << "\n";
    }
    if (item.level == diagnostic_msgs::DiagnosticStatus::ERROR)
      ROS_WARN_THROTTLE(2.0, "[health] %s %s", item.name.c_str(), item.message.c_str());
  }
  if (diagnostics_file.is_open()) diagnostics_file.flush();
}
