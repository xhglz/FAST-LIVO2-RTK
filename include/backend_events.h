#pragma once

#include "common_lib.h"
#include <gnss_comm/GnssPVTSolnMsg.h>
#include <GeographicLib/LocalCartesian.hpp>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <array>
#include <set>

namespace backend {

// T_AB maps coordinates from B to A. Pose covariance uses [rotation, position].
struct Frames
{
    std::string map = "map";
    std::string odom = "odom";
    std::string imu = "imu_link";
    std::string lidar = "lidar_link";
    std::string camera = "camera_link";
    std::string gnss = "gnss_link";
};

struct Quality
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix<double, 6, 6> pose_covariance = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> lidar_information = Eigen::Matrix<double, 6, 6>::Zero();
    std::size_t lidar_points = 0;
    std::size_t lidar_matches = 0;
    std::size_t visual_points = 0;
    double lidar_residual_rms = std::numeric_limits<double>::quiet_NaN();
    double normalized_residual_rms = std::numeric_limits<double>::quiet_NaN();
    double match_ratio = 0.0, angular_coverage = 0.0;
    double translation_correction = 0.0, rotation_correction = 0.0;
    Eigen::Matrix<double, 6, 6> scaled_information = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> information_eigenvalues = Eigen::Matrix<double, 6, 1>::Zero();
    double eigen_ratio = 0.0;
    bool static_filter_available = false;
    bool lidar_update_accepted = true;
    bool visual_update = false;
};

struct KeyframeEvent
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::uint64_t sequence = 0;
    double measurement_time = 0.0;
    std::string pose_frame = "odom";
    std::string body_frame = "imu_link";
    Eigen::Isometry3d T_odom_imu = Eigen::Isometry3d::Identity();
    Eigen::Vector3d velocity_odom = Eigen::Vector3d::Zero();
    Eigen::Vector3d gravity_odom = Eigen::Vector3d::Zero();
    PointCloudXYZRGB::ConstPtr cloud_imu;
    Quality quality;
    // Revocations are odom-grid cells, independent of optimized map coordinates.
    double removed_cell_size = 0.0;
    std::vector<std::array<int, 3>> removed_cells_odom;
};

inline bool validRemovals(const KeyframeEvent& event)
{
    return !event.removed_cells_odom.empty() && std::isfinite(event.removed_cell_size) &&
           event.removed_cell_size >= 0.01 && event.removed_cell_size <= 10.0 &&
           event.removed_cells_odom.size() <= 60000;
}

inline PointCloudXYZRGB::Ptr removeMapCells(const PointCloudXYZRGB::ConstPtr& cloud,
                                           const Eigen::Isometry3d& raw,
                                           const KeyframeEvent& event)
{
    PointCloudXYZRGB::Ptr retained(new PointCloudXYZRGB());
    if (!cloud) return retained;
    const std::set<std::array<int, 3>> cells(event.removed_cells_odom.begin(), event.removed_cells_odom.end());
    for (const auto& point : *cloud) {
        const Eigen::Vector3d p = raw * Eigen::Vector3d(point.x, point.y, point.z);
        if (!p.allFinite() || p.cwiseAbs().maxCoeff() > 1e6) continue;
        const std::array<int, 3> key{static_cast<int>(std::floor(p.x() / event.removed_cell_size)),
                                   static_cast<int>(std::floor(p.y() / event.removed_cell_size)),
                                   static_cast<int>(std::floor(p.z() / event.removed_cell_size))};
        if (!cells.count(key)) retained->push_back(point);
    }
    return retained;
}

struct GnssEvent
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::uint64_t sequence = 0;
    std::uint32_t gps_week = 0;
    double gps_tow = 0.0;
    double raw_unix_time = 0.0;
    double measurement_time = 0.0;
    double configured_offset = 0.0;
    std::string position_frame = "map";
    std::string body_frame = "gnss_link";
    Eigen::Vector3d position_enu = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_enu = Eigen::Vector3d::Zero();
    Eigen::Vector3d position_variance = Eigen::Vector3d::Zero();
    double horizontal_accuracy = 0.0;
    double vertical_accuracy = 0.0;
    double velocity_accuracy = 0.0;
    double pdop = 0.0;
    std::uint8_t fix_type = 0;
    std::uint8_t carrier_solution = 0;
    std::uint8_t satellites = 0;
    bool valid_fix = false;
    bool differential = false;
    bool quality_available = true;
    std::string classification = "UNCLASSIFIED";
    double observation_age = std::numeric_limits<double>::quiet_NaN();
    double innovation_score = std::numeric_limits<double>::quiet_NaN();
    bool trusted_fix = false, stable_fix = false;
};

using KeyframeCallback = std::function<void(const KeyframeEvent&)>;
using GnssCallback = std::function<void(const GnssEvent&)>;

inline Eigen::Matrix<double, 6, 6> poseCovarianceToRos(
    const Eigen::Matrix3d& R, const Eigen::Matrix<double, 6, 6>& native)
{
    Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Zero();
    A.block<3, 3>(0, 3).setIdentity();
    A.block<3, 3>(3, 0) = R;
    return A * native * A.transpose();
}

inline Eigen::Matrix<double, 6, 6> poseCovarianceFromRos(
    const Eigen::Matrix3d& R, const Eigen::Matrix<double, 6, 6>& covariance)
{
    Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Zero();
    A.block<3, 3>(0, 3).setIdentity();
    A.block<3, 3>(3, 0) = R;
    return A.transpose() * covariance * A;
}

inline bool validKeyframe(const KeyframeEvent& event)
{
    const auto& R = event.T_odom_imu.linear();
    return std::isfinite(event.measurement_time) && event.measurement_time > 0.0 &&
           event.measurement_time < 4294967296.0 &&
           event.T_odom_imu.matrix().allFinite() && event.velocity_odom.allFinite() &&
           (R.transpose() * R - Eigen::Matrix3d::Identity()).norm() < 1e-6 &&
           std::abs(R.determinant() - 1.0) < 1e-6 &&
           event.quality.pose_covariance.allFinite() && event.cloud_imu &&
           (!event.cloud_imu->empty() || validRemovals(event)) &&
           (event.removed_cells_odom.empty() || validRemovals(event)) &&
           !event.pose_frame.empty() && !event.body_frame.empty();
}

inline bool validGnss(const GnssEvent& event)
{
    return std::isfinite(event.raw_unix_time) && std::isfinite(event.measurement_time) &&
           event.measurement_time > 0.0 && event.measurement_time < 4294967296.0 &&
           event.position_enu.allFinite() &&
           event.velocity_enu.allFinite() && event.position_variance.allFinite() &&
           (event.position_variance.array() >= 0.0).all();
}

inline bool decodeGnss(const gnss_comm::GnssPVTSolnMsg& msg,
                       double offset, double leap_seconds,
                       GeographicLib::LocalCartesian& origin, bool& origin_initialized,
                       GnssEvent& event)
{
    const double raw_time = 315964800.0 + msg.time.week * 604800.0 + msg.time.tow - leap_seconds;
    const double time = raw_time - offset;
    if (!std::isfinite(msg.time.tow) || msg.time.tow < 0.0 || msg.time.tow >= 604800.0 ||
        !std::isfinite(time) || time <= 0.0 || time >= 4294967296.0 ||
        !std::isfinite(msg.latitude) || std::abs(msg.latitude) > 90.0 ||
        !std::isfinite(msg.longitude) || std::abs(msg.longitude) > 180.0 ||
        !std::isfinite(msg.altitude) || !std::isfinite(offset) || !std::isfinite(leap_seconds) ||
        !std::isfinite(msg.h_acc) || msg.h_acc < 0.0 ||
        !std::isfinite(msg.v_acc) || msg.v_acc < 0.0 ||
        !std::isfinite(msg.vel_e) || !std::isfinite(msg.vel_n) || !std::isfinite(msg.vel_d)) {
        return false;
    }
    if (!origin_initialized) {
        origin.Reset(msg.latitude, msg.longitude, msg.altitude);
        origin_initialized = true;
    }
    event.gps_week = msg.time.week;
    event.gps_tow = msg.time.tow;
    event.raw_unix_time = raw_time;
    event.configured_offset = offset;
    event.measurement_time = event.raw_unix_time - offset;
    origin.Forward(msg.latitude, msg.longitude, msg.altitude,
                   event.position_enu.x(), event.position_enu.y(), event.position_enu.z());
    event.velocity_enu << msg.vel_e, msg.vel_n, -msg.vel_d;
    event.horizontal_accuracy = msg.h_acc;
    event.vertical_accuracy = msg.v_acc;
    // Receiver accuracy estimates are not a full covariance; preserve them separately.
    event.position_variance << msg.h_acc * msg.h_acc, msg.h_acc * msg.h_acc, msg.v_acc * msg.v_acc;
    event.velocity_accuracy = msg.vel_acc;
    event.pdop = msg.p_dop;
    event.fix_type = msg.fix_type;
    event.carrier_solution = msg.carr_soln;
    event.satellites = msg.num_sv;
    event.valid_fix = msg.valid_fix;
    event.differential = msg.diff_soln;
    return validGnss(event);
}

} // namespace backend
