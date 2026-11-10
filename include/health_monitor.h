#pragma once

#include "backend_events.h"
#include <array>
#include <map>
#include <string>

namespace health {

enum class State { INITIALIZING, NORMAL, DEGRADED };
enum class Sensor { IMU, LIDAR, CAMERA, GNSS };
enum Fault : unsigned {
    NONE = 0, INVALID = 1, REVERSED = 2, GAP = 4, MISSING = 8,
    DELAYED = 16, FUTURE = 32, BACKLOG = 64, SATURATED = 128
};

const char* name(State state);
const char* name(Sensor sensor);
std::string reasons(unsigned flags);

struct Hysteresis
{
    State state = State::INITIALIZING;
    int bad = 0, good = 0;
    int bad_frames = 3, good_frames = 5;
    std::size_t transitions = 0;
    void update(bool healthy);
};

struct InputConfig
{
    bool enabled = true;
    double max_gap = 0.5;
    double max_age = 0.5;
    double future_tolerance = 0.2;
    double max_queue_age = 1.0;
    std::size_t max_queue = 100;
};

struct InputStatus
{
    InputConfig config;
    Hysteresis health;
    bool seen = false;
    double last_time = -1.0;
    double last_receipt_ros = 0.0;
    double last_fault_ros = -1.0;
    double age = std::numeric_limits<double>::quiet_NaN();
    double queue_age = 0.0;
    std::size_t queue = 0, received = 0, rejected = 0;
    unsigned flags = NONE, held_flags = NONE;
    std::array<std::size_t, 8> counts{};
};

class InputMonitor
{
public:
    std::array<InputStatus, 4> inputs;
    double startup_grace = 2.0, fault_hold = 0.25;
    double last_poll = -1.0, first_poll = -1.0;
    // Invalid values and non-increasing time always fail; quality flags are advisory.
    bool observe(Sensor sensor, double time, double now, bool finite, bool saturated = false);
    void mark(Sensor sensor, unsigned flags, double now);
    void poll(double now);
    bool safe(Sensor sensor) const;
};

struct GnssConfig
{
    int min_satellites = 12, acquire_frames = 5;
    double max_h_accuracy = 0.15, max_v_accuracy = 0.30;
    double max_age = 0.5, future_tolerance = 0.2;
    double sigma_xy_floor = 0.05, sigma_z_floor = 0.10;
    double h_accuracy_scale = 1.0, v_accuracy_scale = 1.0;
    double jump_min = 2.0, jump_sigma = 6.0, motion_sigma_per_second = 2.0;
    double continuity_timeout = 3.0, max_interpolation_gap = 1.0;
};

struct GnssDecision
{
    std::string classification = "UNAVAILABLE";
    bool trusted = false, stable = false;
    double age = std::numeric_limits<double>::quiet_NaN();
    double innovation_score = std::numeric_limits<double>::quiet_NaN();
    Eigen::Vector3d variance = Eigen::Vector3d::Zero();
};

Eigen::Vector3d gnssVariance(const backend::GnssEvent& event, const GnssConfig& config);

class GnssMonitor
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GnssConfig config;
    Hysteresis health;
    GnssDecision last;
    std::map<std::string, std::size_t> counts;
    std::size_t forwarded = 0, rejected = 0;
    bool have_anchor = false;
    int consecutive_fix = 0;
    backend::GnssEvent anchor;
    GnssDecision observe(const backend::GnssEvent& event, double now);
    void unavailable(const std::string& reason);
};

struct FrontendConfig
{
    std::size_t min_matches = 30;
    int warmup_frames = 20;
    double min_match_ratio = 0.05, min_coverage = 0.10;
    double max_normalized_rms = 5.0;
    double rotation_scale = 0.1, translation_scale = 1.0;
    double min_eigen_ratio = 1e-6;
    double max_translation_correction = 1.0, max_rotation_correction = 0.2;
};

class FrontendMonitor
{
public:
    FrontendConfig config;
    Hysteresis health;
    std::size_t frames = 0, rejected = 0;
    std::string reason = "WARMUP";
    bool assess(backend::Quality& quality);
};

void loadGnssConfig(ros::NodeHandle& nh, GnssConfig& config);
void loadConfig(ros::NodeHandle& nh, InputMonitor& inputs, GnssMonitor& gnss, FrontendMonitor& frontend);

} // namespace health
