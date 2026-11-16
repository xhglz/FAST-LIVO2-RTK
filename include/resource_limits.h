#pragma once

#include "common_lib.h"
#include <cstddef>
#include <deque>
#include <stdexcept>

namespace resources {
template <typename T>
std::size_t purgeExpired(std::deque<T>& messages, std::deque<double>& times, double cutoff)
{
    if (messages.size() != times.size()) throw std::logic_error("unpaired resource queue");
    std::size_t removed = 0;
    while (!times.empty() && times.front() <= cutoff) {
        messages.pop_front();
        times.pop_front();
        ++removed;
    }
    return removed;
}

template <typename Cloud, typename Flush>
std::size_t appendChunks(Cloud& destination, const Cloud& input, std::size_t limit, Flush flush)
{
    std::size_t offset = 0;
    while (offset < input.size()) {
        if (!limit || destination.size() > limit) return input.size() - offset;
        if (destination.size() == limit && !flush()) return input.size() - offset;
        const std::size_t n = std::min(limit - destination.size(), input.size() - offset);
        if (!n) return input.size() - offset;
        destination.insert(destination.end(), input.begin() + offset, input.begin() + offset + n);
        offset += n;
    }
    return 0;
}

struct Config {
    bool enabled = false;
    std::size_t imu_queue = 2000, lidar_queue = 10, camera_queue = 10;
    std::size_t lidar_points = 200000, image_bytes = 16000000;
    std::size_t map_roots = 40000, visual_points = 10000, path_poses = 2000;
    std::size_t pcd_points = 300000, scan_points = 300000;
    std::size_t registration_points = 5000;
    double map_radius = 40.0;
    double registration_period = 0.5;
};

inline bool fits(std::size_t used, std::size_t incoming, std::size_t limit)
{
    return used <= limit && incoming <= limit - used;
}

inline PointCloudXYZRGB::Ptr boundedCloud(const PointCloudXYZRGB::ConstPtr& cloud, std::size_t limit)
{
    PointCloudXYZRGB::Ptr result(new PointCloudXYZRGB);
    if (!cloud || !limit) return result;
    if (cloud->size() <= limit) { *result = *cloud; return result; }
    result->reserve(limit);
    for (std::size_t i = 0; i < limit; ++i)
        result->push_back((*cloud)[i * cloud->size() / limit]);
    return result;
}
} // namespace resources
