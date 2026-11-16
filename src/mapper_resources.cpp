#include "LIVMapper.h"
#include <filesystem>
#include <iomanip>

void LIVMapper::initializeResources(ros::NodeHandle& nh)
{
    nh.param("resources/enabled", resource_config.enabled, false);
    if (!resource_config.enabled) return;
    if (!safety_machine) throw std::invalid_argument("resources/enabled requires safety/enabled for overflow validity");
    if (!imu_en) throw std::invalid_argument("resource mode requires IMU-enabled LIO/LIVO; pure LO is unsupported");
    std::string mode;
    nh.param<std::string>("backend/mode", mode, "batch");
    if (mode != "online") throw std::invalid_argument("resource mode requires online backend; batch retains full offline data");
    if (rtk_en) throw std::invalid_argument("resource mode does not support the legacy local RTK history buffers");
    auto count = [&](const char* name, std::size_t& value) {
        int n = static_cast<int>(value);
        nh.param(std::string("resources/") + name, n, n);
        if (n < 1) throw std::invalid_argument(std::string("resources/") + name + " must be positive");
        value = n;
    };
    count("imu_queue", resource_config.imu_queue); count("lidar_queue", resource_config.lidar_queue);
    count("camera_queue", resource_config.camera_queue); count("lidar_points", resource_config.lidar_points);
    count("image_bytes", resource_config.image_bytes); count("map_roots", resource_config.map_roots);
    count("visual_points", resource_config.visual_points); count("path_poses", resource_config.path_poses);
    count("pcd_points", resource_config.pcd_points); count("scan_points", resource_config.scan_points);
    count("registration_points", resource_config.registration_points);
    nh.param("resources/map_radius", resource_config.map_radius, resource_config.map_radius);
    nh.param("resources/registration_period", resource_config.registration_period, resource_config.registration_period);
    if (!std::isfinite(resource_config.map_radius) || resource_config.map_radius <= 0)
        throw std::invalid_argument("resources/map_radius must be positive and finite");
    if (!std::isfinite(resource_config.registration_period) || resource_config.registration_period <= 0)
        throw std::invalid_argument("resources/registration_period must be positive and finite");
    voxelmap_manager->resource_max_roots = resource_config.map_roots;
    vio_manager->resource_max_points = resource_config.visual_points;
    nh.param("resources/visual_frame_window", vio_manager->resource_frame_window, 100);
    if (vio_manager->resource_frame_window < 1) throw std::invalid_argument("resources/visual_frame_window must be positive");
    const auto static_allowed = voxelmap_manager->permanent_point_allowed_;
    voxelmap_manager->permanent_point_allowed_ = [this, static_allowed](const V3D& p) {
        return p.allFinite() && (p - _state.pos_end).norm() <= resource_config.map_radius &&
            (!static_allowed || static_allowed(p));
    };
    const auto directory = std::filesystem::path(save_directory.empty() ? std::string(ROOT_DIR) + "Log" : save_directory + "/debug");
    std::filesystem::create_directories(directory);
    resource_file.open((directory / "resources.csv").string());
    timing_file.open((directory / "timing.csv").string());
    if (!resource_file || !timing_file) throw std::runtime_error("cannot open resource logs");
    resource_file << "time,segment,stage,map_roots,temporary_roots,visual_points,path_poses,pcd_points,scan_points,"
                     "imu_queue,lidar_queue,camera_queue,prop_queue,imu_dropped,lidar_dropped,camera_dropped,"
                     "map_rejected,pcd_chunks,pcd_rejected,archive_failed,registration_attempts,map_revision,pose_valid\n";
    timing_file << "time,stage,processing_ms,measurement_age_ms\n";
}

bool LIVMapper::resourceInput(health::Sensor sensor, std::size_t incoming)
{
    if (!resource_config.enabled) return true;
    const auto index = static_cast<int>(sensor);
    const std::size_t used = sensor == health::Sensor::IMU ? imu_buffer.size() :
        sensor == health::Sensor::LIDAR ? lid_raw_data_buffer.size() : img_buffer.size();
    const std::size_t limit = sensor == health::Sensor::IMU ? resource_config.imu_queue :
        sensor == health::Sensor::LIDAR ? resource_config.lidar_queue : resource_config.camera_queue;
    if (resources::fits(used, incoming, limit)) return true;
    ++resource_dropped[index];
    input_monitor.mark(sensor, health::BACKLOG, ros::Time::now().toSec());
    if (sensor == health::Sensor::CAMERA && incoming == 1 && !img_buffer.empty()) {
        img_buffer.pop_front();
        img_time_buffer.pop_front();
        return true;
    }
    if (sensor != health::Sensor::CAMERA) safety_machine->lose("RESOURCE_INPUT_OVERFLOW");
    ROS_WARN_THROTTLE(1.0, "[resources] %s input refused at capacity", health::name(sensor));
    return false;
}

bool LIVMapper::flushResourcePcd()
{
    if (!resource_config.enabled || (pcl_wait_save->empty() && pcl_wait_save_intensity->empty())) return true;
    if (resource_archive_failed) return false;
    try {
        const auto directory = std::filesystem::path(save_directory.empty() ? std::string(ROOT_DIR) + "Log" : save_directory) / "resource_pcd";
        std::filesystem::create_directories(directory);
        const std::string prefix = (directory / (std::to_string(safety_machine->segment()) + "_" +
            std::to_string(resource_pcd_chunks))).string();
        if ((!pcl_wait_save->empty() && pcl::io::savePCDFileBinary(prefix + "_rgb.pcd", *pcl_wait_save) != 0) ||
            (!pcl_wait_save_intensity->empty() && pcl::io::savePCDFileBinary(prefix + "_intensity.pcd", *pcl_wait_save_intensity) != 0))
            throw std::runtime_error("PCD write failed");
        ++resource_pcd_chunks;
        PointCloudXYZRGB().swap(*pcl_wait_save);
        PointCloudXYZI().swap(*pcl_wait_save_intensity);
        return true;
    } catch (const std::exception& error) {
        resource_archive_failed = true;
        ROS_ERROR("[resources] archive stopped, refusing further PCD accumulation: %s", error.what());
        return false;
    }
}

void LIVMapper::maintainResources()
{
    if (!resource_config.enabled) return;
    if (!resources::fits(pcl_wait_pub->size(), pcl_w_wait_pub->size(), resource_config.scan_points)) {
        PointCloudXYZI().swap(*pcl_wait_pub);
        if (pcl_w_wait_pub->size() > resource_config.scan_points) {
            PointCloudXYZI sampled;
            for (std::size_t i = 0; i < resource_config.scan_points; ++i)
                sampled.push_back((*pcl_w_wait_pub)[i * pcl_w_wait_pub->size() / resource_config.scan_points]);
            pcl_w_wait_pub->swap(sampled);
        }
    }
    if (!trustedOutputAllowed()) return;
    auto& map = voxelmap_manager->voxel_map_;
    for (auto it = map.begin(); LidarMeasures.lio_vio_flg == LIO && it != map.end();) {
        const auto* root = it->second;
        const V3D center(root->voxel_center_[0], root->voxel_center_[1], root->voxel_center_[2]);
        if ((center - _state.pos_end).norm() > resource_config.map_radius) {
            delete it->second;
            it = map.erase(it);
            ++voxelmap_manager->map_revision_;
        } else ++it;
    }
    const auto static_allowed = vio_manager->static_point_allowed;
    vio_manager->static_point_allowed = [this, static_allowed](const V3D& p) {
        return p.allFinite() && (p - _state.pos_end).norm() <= resource_config.map_radius &&
            (!static_allowed || static_allowed(p));
    };
    vio_manager->pruneNonStaticPoints();
    vio_manager->static_point_allowed = static_allowed;
    for (auto& entry : vio_manager->warp_map) delete entry.second;
    vio_manager->warp_map.clear();
}

void LIVMapper::recordResources(const char* stage, double milliseconds)
{
    if (!resource_config.enabled) return;
    std::size_t prop;
    { std::lock_guard<std::mutex> lock(mtx_buffer_imu_prop); prop = prop_imu_buffer.size(); }
    resource_file << std::setprecision(17) << LidarMeasures.last_lio_update_time << "," << safety_machine->segment()
        << "," << stage << "," << voxelmap_manager->voxel_map_.size() << "," << voxelmap_manager->temporary_map_.size()
        << "," << vio_manager->resource_point_count << "," << path.poses.size()
        << "," << pcl_wait_save->size() + pcl_wait_save_intensity->size()
        << "," << pcl_wait_pub->size() + pcl_w_wait_pub->size()
        << "," << imu_buffer.size() << "," << lid_raw_data_buffer.size() << "," << img_buffer.size() << "," << prop
        << "," << resource_dropped[0] << "," << resource_dropped[1] << "," << resource_dropped[2]
        << "," << voxelmap_manager->resource_rejected_points << "," << resource_pcd_chunks
        << "," << resource_pcd_rejected << "," << resource_archive_failed << "," << resource_registration_attempts
        << "," << voxelmap_manager->map_revision_ << "," << trustedOutputAllowed() << "\n";
    timing_file << std::setprecision(17) << LidarMeasures.last_lio_update_time << "," << stage << "," << milliseconds
        << "," << (ros::Time::now().toSec() - LidarMeasures.last_lio_update_time) * 1000.0 << "\n";
    resource_file.flush(); timing_file.flush();
}
