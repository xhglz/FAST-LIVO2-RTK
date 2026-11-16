#include "LIVMapper.h"
#include <iomanip>

bool LIVMapper::dynamicPoseReliable(const StatesGroup& state) const
{
    if (!state.pos_end.allFinite() || !state.rot_end.allFinite() || !state.cov.allFinite()) return false;
    // The initial gauge prior needs a temporary-only bootstrap to obtain matches.
    // lidar_map_inited is set before the update, so permanent writes use strict bounds.
    const double position_sigma = !lidar_map_inited ?
        dynamic_config.bootstrap_position_sigma : dynamic_config.max_position_sigma;
    const double rotation_sigma = !lidar_map_inited ?
        dynamic_config.bootstrap_rotation_sigma : dynamic_config.max_rotation_sigma;
    return state.cov.diagonal().minCoeff() >= 0.0 &&
           state.cov.block<3, 3>(3, 3).diagonal().maxCoeff() <=
               position_sigma * position_sigma &&
           state.cov.block<3, 3>(0, 0).diagonal().maxCoeff() <=
               rotation_sigma * rotation_sigma &&
           (!p_imu->imu_need_init && state.gravity.norm() > 1.0);
}

void LIVMapper::updateDynamicMap(double time)
{
    const bool bootstrap = voxelmap_manager->voxel_map_.empty() && frame_num < 15;
    const bool reliable = lio_update_allowed && dynamicPoseReliable(_state) &&
        (!safety_machine || !safety_machine->recovering()) &&
        last_quality.translation_correction <= dynamic_config.max_correction &&
        last_quality.rotation_correction <= dynamic_config.max_rotation_sigma &&
        (bootstrap || (last_quality.lidar_matches >= frontend_monitor.config.min_matches &&
                       std::isfinite(last_quality.normalized_residual_rms) &&
                       last_quality.normalized_residual_rms <= frontend_monitor.config.max_normalized_rms));
    dynamic_map_write_allowed = reliable && (!safety_machine || safety_machine->mapWritable());
    dynamic_filter_engine->commit(voxelmap_manager->pv_list_, dynamic_mask,
        _state.pos_end + _state.rot_end * extT, time, reliable);
    voxelmap_manager->replaceTemporaryMap(dynamic_filter_engine->temporaryPoints(time));
    if (!dynamic_filter_engine->revoked().empty()) {
        std::set<dynamic_filter::Key> removed;
        for (const auto& p : dynamic_filter_engine->revoked()) removed.insert(dynamic_filter_engine->key(p));
        pending_map_removals.insert(removed.begin(), removed.end());
        for (const auto& k : removed)
            dynamic_removal_file << std::setprecision(17) << time << "," << dynamic_config.cell_size
                << "," << k[0] << "," << k[1] << "," << k[2] << "\n";
        dynamic_removal_file.flush();
        voxelmap_manager->rebuildRoots(dynamic_filter_engine->revoked(), dynamic_filter_engine->confirmedPoints());
        // Accumulated colored buffers use the same world-cell confidence gate.
        auto prune = [this](auto& cloud) {
            cloud->points.erase(std::remove_if(cloud->points.begin(), cloud->points.end(), [this](const auto& p) {
                return !dynamic_filter_engine->permanentAllowed(V3D(p.x, p.y, p.z));
            }), cloud->points.end());
            cloud->width = cloud->size();
            cloud->height = 1;
        };
        prune(pcl_wait_pub);
        prune(pcl_wait_save);
        prune(pcl_wait_save_intensity);
    }
    if (dynamic_map_write_allowed) voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);
    const auto& s = dynamic_filter_engine->stats();
    dynamic_quality_file << std::setprecision(17) << time << "," << reliable;
    for (const auto n : s.points) dynamic_quality_file << "," << n;
    dynamic_quality_file << "," << s.cells << "," << s.moving_clusters << "," << s.removed
        << "," << s.capacity_rejected << "," << dynamic_filter_engine->temporaryPoints(time).size() << "\n";
    dynamic_quality_file.flush();
}
