#include "health_monitor.h"
#include <Eigen/Eigenvalues>
#include <algorithm>

namespace health {

const char* name(State state)
{
    switch (state) {
        case State::NORMAL: return "NORMAL";
        case State::DEGRADED: return "DEGRADED";
        default: return "INITIALIZING";
    }
}

const char* name(Sensor sensor)
{
    static const char* names[] = {"imu", "lidar", "camera", "gnss"};
    return names[static_cast<int>(sensor)];
}

std::string reasons(unsigned flags)
{
    static const char* labels[] = {"INVALID_VALUE", "TIME_REVERSED", "MEASUREMENT_GAP",
        "MISSING", "DELAYED", "FUTURE_TIME", "QUEUE_BACKLOG", "IMU_SATURATION"};
    std::string result;
    for (int i = 0; i < 8; ++i)
        if (flags & (1u << i)) result += (result.empty() ? "" : "|") + std::string(labels[i]);
    return result.empty() ? "OK" : result;
}

void Hysteresis::update(bool healthy)
{
    const State old = state;
    if (healthy) {
        bad = 0;
        good = std::min(good + 1, good_frames);
        if (good >= good_frames) state = State::NORMAL;
    } else {
        good = 0;
        bad = std::min(bad + 1, bad_frames);
        if (bad >= bad_frames) state = State::DEGRADED;
    }
    if (old != state) ++transitions;
}

void InputMonitor::mark(Sensor sensor, unsigned flags, double now)
{
    auto& input = inputs[static_cast<int>(sensor)];
    input.held_flags |= flags;
    input.flags |= flags;
    input.last_fault_ros = now;
    for (int i = 0; i < 8; ++i)
        if (flags & (1u << i)) ++input.counts[i];
}

bool InputMonitor::observe(Sensor sensor, double time, double now, bool finite, bool saturated)
{
    auto& input = inputs[static_cast<int>(sensor)];
    if (!input.config.enabled) return false;
    ++input.received;
    input.last_receipt_ros = now;
    unsigned flags = NONE;
    const bool valid_time = std::isfinite(time) && time > 0.0 && time < 4294967296.0;
    if (!finite || !valid_time) flags |= INVALID;
    if (input.seen && valid_time && time <= input.last_time) flags |= REVERSED;
    if (input.seen && valid_time && time > input.last_time + input.config.max_gap) flags |= GAP;
    if (saturated) flags |= SATURATED;
    if (valid_time && now > 0.0) {
        input.age = now - time;
        if (input.age > input.config.max_age) flags |= DELAYED;
        if (input.age < -input.config.future_tolerance) flags |= FUTURE;
    }
    if (flags) mark(sensor, flags, now);
    if (flags & (INVALID | REVERSED)) {
        ++input.rejected;
        return false;
    }
    input.seen = true;
    input.last_time = time;
    return true;
}

void InputMonitor::poll(double now)
{
    // Simulation pauses do not count as missing samples or hysteresis frames.
    if (!std::isfinite(now) || now <= 0.0 || now <= last_poll) return;
    if (first_poll < 0.0) first_poll = now;
    if (now < last_poll + 0.1) return;
    last_poll = now;
    for (auto& input : inputs) {
        if (!input.config.enabled) continue;
        unsigned flags = NONE;
        if (now - input.last_fault_ros <= fault_hold) flags |= input.held_flags;
        else input.held_flags = NONE;
        if (input.seen) {
            input.age = now - input.last_time;
            if (now - input.last_receipt_ros > input.config.max_gap) flags |= MISSING;
            if (input.age > input.config.max_age) flags |= DELAYED;
            if (input.age < -input.config.future_tolerance) flags |= FUTURE;
        } else if (now - first_poll > startup_grace) flags |= MISSING;
        if (input.queue >= input.config.max_queue || input.queue_age > input.config.max_queue_age)
            flags |= BACKLOG;
        // Count sustained conditions on their rising edge, not on every tick.
        const unsigned rising = flags & ~input.flags & (MISSING | DELAYED | FUTURE | BACKLOG);
        for (int i = 0; i < 8; ++i)
            if (rising & (1u << i)) ++input.counts[i];
        input.flags = flags;
        if (input.seen || now - first_poll > startup_grace) input.health.update(flags == NONE);
    }
}

bool InputMonitor::safe(Sensor sensor) const
{
    const auto& input = inputs[static_cast<int>(sensor)];
    return !input.config.enabled ||
           (input.seen && (input.flags | input.held_flags) == NONE && input.health.state != State::DEGRADED);
}

Eigen::Vector3d gnssVariance(const backend::GnssEvent& event, const GnssConfig& config)
{
    const double sx = std::max(config.sigma_xy_floor, event.horizontal_accuracy * config.h_accuracy_scale);
    const double sz = std::max(config.sigma_z_floor, event.vertical_accuracy * config.v_accuracy_scale);
    return Eigen::Vector3d(sx * sx, sx * sx, sz * sz);
}

GnssDecision GnssMonitor::observe(const backend::GnssEvent& event, double now)
{
    GnssDecision decision;
    decision.variance = gnssVariance(event, config);
    decision.age = now > 0.0 ? now - event.measurement_time :
                             std::numeric_limits<double>::quiet_NaN();
    if (!backend::validGnss(event) || !std::isfinite(event.horizontal_accuracy) ||
        !std::isfinite(event.vertical_accuracy) || event.horizontal_accuracy < 0.0 ||
        event.vertical_accuracy < 0.0 || !decision.variance.allFinite())
        decision.classification = "INVALID_VALUE";
    else if (!event.quality_available) decision.classification = "QUALITY_UNKNOWN";
    else if (!event.valid_fix || event.fix_type != 3) decision.classification = "NO_VALID_FIX";
    else if (event.carrier_solution == 1) decision.classification = "FLOAT";
    else if (event.carrier_solution != 2) decision.classification = "NON_RTK";
    else if (event.satellites < config.min_satellites) decision.classification = "LOW_SATELLITES";
    else if (event.horizontal_accuracy > config.max_h_accuracy ||
             event.vertical_accuracy > config.max_v_accuracy) decision.classification = "POOR_ACCURACY";
    else if (!std::isfinite(decision.age)) decision.classification = "CLOCK_UNAVAILABLE";
    else if (decision.age > config.max_age) decision.classification = "STALE";
    else if (decision.age < -config.future_tolerance) decision.classification = "FUTURE_TIME";
    else {
        decision.classification = "TRUSTED_FIX";
        if (have_anchor) {
            const double dt = event.measurement_time - anchor.measurement_time;
            if (dt <= 0.0) decision.classification = "TIME_REVERSED";
            else if (dt <= config.continuity_timeout) {
                const Eigen::Vector3d residual = event.position_enu - anchor.position_enu - anchor.velocity_enu * dt;
                const Eigen::Vector3d variance = decision.variance + anchor.position_variance +
                    Eigen::Vector3d::Constant(std::pow(config.motion_sigma_per_second * dt, 2));
                decision.innovation_score = (residual.array().square() / variance.array()).sum();
                if (residual.norm() > config.jump_min &&
                    decision.innovation_score > config.jump_sigma * config.jump_sigma)
                    decision.classification = "POSITION_JUMP";
            } else {
                consecutive_fix = 0;
            }
        }
    }
    decision.trusted = decision.classification == "TRUSTED_FIX";
    if (decision.trusted) {
        ++consecutive_fix;
        anchor = event;
        anchor.position_variance = decision.variance;
        have_anchor = true;
    } else consecutive_fix = 0;
    decision.stable = decision.trusted && consecutive_fix >= config.acquire_frames;
    health.update(decision.trusted);
    ++counts[decision.classification];
    last = decision;
    return decision;
}

void GnssMonitor::unavailable(const std::string& reason)
{
    if (last.classification != reason) ++counts[reason];
    consecutive_fix = 0;
    last.classification = reason;
    last.trusted = last.stable = false;
    health.update(false);
}

bool FrontendMonitor::assess(backend::Quality& quality)
{
    ++frames;
    Eigen::Matrix<double, 6, 6> S = Eigen::Matrix<double, 6, 6>::Zero();
    S.diagonal().head<3>().setConstant(config.rotation_scale);
    S.diagonal().tail<3>().setConstant(config.translation_scale);
    const Eigen::Matrix<double, 6, 6> H = 0.5 * (quality.lidar_information + quality.lidar_information.transpose());
    quality.scaled_information = S.transpose() * H * S / std::max<std::size_t>(1, quality.lidar_matches);
    if (quality.scaled_information.allFinite()) {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigen(quality.scaled_information);
        if (eigen.info() == Eigen::Success) {
            quality.information_eigenvalues = eigen.eigenvalues().cwiseMax(0.0);
            const double max_eigen = quality.information_eigenvalues.maxCoeff();
            quality.eigen_ratio = max_eigen > 0.0 ? quality.information_eigenvalues.minCoeff() / max_eigen : 0.0;
        }
    }
    reason.clear();
    auto add = [&](bool failed, const char* value) {
        if (failed) reason += (reason.empty() ? "" : "|") + std::string(value);
    };
    add(!quality.scaled_information.allFinite() || !std::isfinite(quality.translation_correction) ||
        !std::isfinite(quality.rotation_correction), "NONFINITE_ESTIMATE");
    add(quality.lidar_matches < config.min_matches, "LOW_INLIERS");
    add(quality.match_ratio < config.min_match_ratio, "LOW_MATCH_RATIO");
    add(quality.angular_coverage < config.min_coverage, "LOW_COVERAGE");
    add(!std::isfinite(quality.normalized_residual_rms) ||
        quality.normalized_residual_rms > config.max_normalized_rms, "HIGH_NORMALIZED_RESIDUAL");
    add(quality.eigen_ratio < config.min_eigen_ratio, "WEAK_GEOMETRY");
    add(quality.translation_correction > config.max_translation_correction ||
        quality.rotation_correction > config.max_rotation_correction, "LARGE_CORRECTION");
    if (frames <= static_cast<std::size_t>(config.warmup_frames) &&
        reason.find("NONFINITE_ESTIMATE") == std::string::npos) {
        reason = "WARMUP";
        return true;
    }
    const bool healthy = reason.empty();
    health.update(healthy);
    if (healthy) reason = "OK";
    return healthy;
}

namespace {
void positive(const std::string& key, double value)
{
    if (!std::isfinite(value) || value <= 0.0)
        throw std::invalid_argument(key + " must be finite and positive");
}

void parameter(ros::NodeHandle& nh, const std::string& key, double& value)
{
    nh.param<double>(key, value, value);
    positive(key, value);
}
} // namespace

void loadGnssConfig(ros::NodeHandle& nh, GnssConfig& c)
{
    const std::string prefix = "health/gnss/";
    nh.param<int>(prefix + "min_satellites", c.min_satellites, c.min_satellites);
    nh.param<int>(prefix + "acquire_frames", c.acquire_frames, c.acquire_frames);
    if (c.min_satellites < 1 || c.acquire_frames < 1) throw std::invalid_argument("invalid GNSS count limits");
    parameter(nh, prefix + "max_h_accuracy", c.max_h_accuracy);
    parameter(nh, prefix + "max_v_accuracy", c.max_v_accuracy);
    parameter(nh, prefix + "max_age", c.max_age);
    parameter(nh, prefix + "future_tolerance", c.future_tolerance);
    parameter(nh, prefix + "sigma_xy_floor", c.sigma_xy_floor);
    parameter(nh, prefix + "sigma_z_floor", c.sigma_z_floor);
    parameter(nh, prefix + "h_accuracy_scale", c.h_accuracy_scale);
    parameter(nh, prefix + "v_accuracy_scale", c.v_accuracy_scale);
    parameter(nh, prefix + "jump_min", c.jump_min);
    parameter(nh, prefix + "jump_sigma", c.jump_sigma);
    parameter(nh, prefix + "motion_sigma_per_second", c.motion_sigma_per_second);
    parameter(nh, prefix + "continuity_timeout", c.continuity_timeout);
    parameter(nh, prefix + "max_interpolation_gap", c.max_interpolation_gap);
}

void loadConfig(ros::NodeHandle& nh, InputMonitor& inputs, GnssMonitor& gnss, FrontendMonitor& frontend)
{
    inputs.inputs[0].config.max_gap = 0.10;
    inputs.inputs[0].config.max_queue = 2000;
    inputs.inputs[1].config.max_queue = 50;
    inputs.inputs[2].config.max_queue = 50;
    inputs.inputs[3].config.max_gap = 1.0;
    int bad = 3, good = 5;
    nh.param<int>("health/bad_frames", bad, bad);
    nh.param<int>("health/good_frames", good, good);
    if (bad < 1 || good < 1) throw std::invalid_argument("health frame counts must be positive");
    parameter(nh, "health/startup_grace", inputs.startup_grace);
    parameter(nh, "health/fault_hold", inputs.fault_hold);
    for (int i = 0; i < 4; ++i) {
        auto& c = inputs.inputs[i].config;
        const std::string prefix = std::string("health/input/") + name(static_cast<Sensor>(i)) + "/";
        parameter(nh, prefix + "max_gap", c.max_gap);
        parameter(nh, prefix + "max_age", c.max_age);
        parameter(nh, prefix + "future_tolerance", c.future_tolerance);
        parameter(nh, prefix + "max_queue_age", c.max_queue_age);
        int max_queue = c.max_queue;
        nh.param<int>(prefix + "max_queue", max_queue, max_queue);
        if (max_queue < 1) throw std::invalid_argument(prefix + "max_queue must be positive");
        c.max_queue = max_queue;
        inputs.inputs[i].health.bad_frames = bad;
        inputs.inputs[i].health.good_frames = good;
    }
    gnss.health.bad_frames = frontend.health.bad_frames = bad;
    gnss.health.good_frames = frontend.health.good_frames = good;
    loadGnssConfig(nh, gnss.config);
    auto& c = frontend.config;
    const std::string prefix = "health/frontend/";
    int min_matches = c.min_matches;
    nh.param<int>(prefix + "min_matches", min_matches, min_matches);
    nh.param<int>(prefix + "warmup_frames", c.warmup_frames, c.warmup_frames);
    if (min_matches < 1 || c.warmup_frames < 0) throw std::invalid_argument("invalid frontend count limits");
    c.min_matches = min_matches;
    parameter(nh, prefix + "min_match_ratio", c.min_match_ratio);
    parameter(nh, prefix + "min_coverage", c.min_coverage);
    parameter(nh, prefix + "max_normalized_rms", c.max_normalized_rms);
    parameter(nh, prefix + "rotation_scale", c.rotation_scale);
    parameter(nh, prefix + "translation_scale", c.translation_scale);
    parameter(nh, prefix + "min_eigen_ratio", c.min_eigen_ratio);
    parameter(nh, prefix + "max_translation_correction", c.max_translation_correction);
    parameter(nh, prefix + "max_rotation_correction", c.max_rotation_correction);
    if (c.min_match_ratio > 1.0 || c.min_coverage > 1.0 || c.min_eigen_ratio > 1.0)
        throw std::invalid_argument("health ratios must be at most one");
}

} // namespace health
