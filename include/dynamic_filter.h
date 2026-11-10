#pragma once

#include "common_lib.h"
#include <array>
#include <functional>
#include <map>
#include <set>

namespace dynamic_filter {

enum class State { UNKNOWN, TENTATIVE_STATIC, CONFIRMED_STATIC, DYNAMIC };
using Key = std::array<int, 3>;

struct Config {
    bool enabled = false;
    double cell_size = 0.3;
    double consistency_distance = 0.15;
    double confirmation_time = 0.8;
    int confirmation_frames = 6;
    int free_frames = 6;
    double temporary_ttl = 1.0;
    double dynamic_hold = 1.0;
    double track_distance = 1.5;
    double min_speed = 0.3;
    double max_speed = 15.0;
    int motion_frames = 3;
    int cluster_min_cells = 4;
    int cluster_max_cells = 250;
    double cluster_max_extent = 5.0;
    double max_range = 60.0;
    double max_position_sigma = 0.5;
    double max_rotation_sigma = 0.1;
    double bootstrap_position_sigma = 1.0;
    double bootstrap_rotation_sigma = 0.2;
    double max_correction = 0.25;
    int max_cells = 60000;
    int samples_per_cell = 12;
    int max_rays = 4000;
    int max_temporary_points = 20000;
};

struct Stats {
    std::array<std::size_t, 4> points{};
    std::size_t cells = 0, moving_clusters = 0, removed = 0, capacity_rejected = 0;
    bool reliable = false;
};

// Owned by the frontend thread. One prepare/commit pair per real LIO measurement.
class Filter {
public:
    explicit Filter(const Config& config = Config()) : config_(config) {}
    using Geometry = std::function<bool(const V3D&)>;
    std::vector<State> prepare(const std::vector<V3D>& world, double time,
                              bool reliable, const Geometry& consistent);
    void commit(const std::vector<pointWithVar>& points, const std::vector<State>& mask,
                const V3D& origin, double time, bool reliable);
    State state(const V3D& world) const;
    bool estimationAllowed(const V3D& world) const { return state(world) != State::DYNAMIC; }
    bool permanentAllowed(const V3D& world) const { return state(world) == State::CONFIRMED_STATIC; }
    std::vector<pointWithVar> temporaryPoints(double time) const;
    std::vector<pointWithVar> confirmedPoints() const;
    const std::vector<V3D>& revoked() const { return revoked_; }
    const Stats& stats() const { return stats_; }
    const Config& config() const { return config_; }
    Key key(const V3D& p) const;
private:
    struct Cell {
        State state = State::UNKNOWN;
        V3D center = V3D::Zero();
        double first = 0.0, last = 0.0, last_free = 0.0, dynamic_until = 0.0;
        int hits = 0, free = 0;
        std::vector<pointWithVar> samples;
    };
    struct Track {
        V3D center = V3D::Zero(), velocity = V3D::Zero();
        double time = 0.0;
        int streak = 0;
        std::size_t size = 0;
        std::vector<Key> keys;
    };
    void revoke(Cell& cell);
    Config config_;
    std::map<Key, Cell> cells_;
    std::vector<Track> tracks_;
    std::set<Key> moving_;
    std::vector<V3D> revoked_;
    Stats stats_;
    double prepared_time_ = -1.0, committed_time_ = -1.0;
};

void loadConfig(ros::NodeHandle& nh, Config& config);

} // namespace dynamic_filter
