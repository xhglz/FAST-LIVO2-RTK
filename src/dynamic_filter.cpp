#include "dynamic_filter.h"
#include <algorithm>
#include <cmath>
#include <queue>
#include <stdexcept>

namespace dynamic_filter {

Key Filter::key(const V3D& p) const
{
    return {static_cast<int>(std::floor(p.x() / config_.cell_size)),
            static_cast<int>(std::floor(p.y() / config_.cell_size)),
            static_cast<int>(std::floor(p.z() / config_.cell_size))};
}

State Filter::state(const V3D& world) const
{
    if (!world.allFinite() || world.cwiseAbs().maxCoeff() > 1e6) return State::UNKNOWN;
    const auto k = key(world);
    if (moving_.count(k)) return State::DYNAMIC;
    const auto it = cells_.find(k);
    if (it == cells_.end()) return State::UNKNOWN;
    if (it->second.state == State::DYNAMIC && prepared_time_ >= it->second.dynamic_until)
        return State::UNKNOWN;
    return it->second.state;
}

void Filter::revoke(Cell& cell)
{
    if (cell.state == State::CONFIRMED_STATIC) {
        for (const auto& sample : cell.samples) revoked_.push_back(sample.point_w);
        ++stats_.removed;
    }
    cell.samples.clear();
    cell.hits = cell.free = 0;
}

std::vector<State> Filter::prepare(const std::vector<V3D>& world, double time,
                                  bool reliable, const Geometry& consistent)
{
    stats_ = Stats();
    stats_.reliable = reliable;
    revoked_.clear();
    moving_.clear();
    if (!std::isfinite(time) || time <= prepared_time_) {
        stats_.reliable = false;
        return std::vector<State>(world.size(), State::UNKNOWN);
    }
    prepared_time_ = time;
    std::map<Key, std::pair<V3D, int>> candidates;
    if (reliable) {
        for (const auto& p : world) {
            if (!p.allFinite() || p.cwiseAbs().maxCoeff() > 1e6) continue;
            const auto it = cells_.find(key(p));
            const bool supported = it != cells_.end() &&
                it->second.state == State::CONFIRMED_STATIC &&
                (p - it->second.center).norm() <= config_.consistency_distance;
            if (supported || (consistent && consistent(p))) continue;
            auto inserted = candidates.emplace(key(p), std::make_pair(V3D::Zero(), 0));
            inserted.first->second.first += p;
            ++inserted.first->second.second;
        }
    }
    std::vector<Track> next;
    std::set<std::size_t> used;
    while (!candidates.empty()) {
        Track cluster;
        V3D low = candidates.begin()->second.first / candidates.begin()->second.second;
        V3D high = low;
        std::queue<Key> pending;
        pending.push(candidates.begin()->first);
        while (!pending.empty()) {
            const auto k = pending.front();
            pending.pop();
            auto it = candidates.find(k);
            if (it == candidates.end()) continue;
            const V3D p = it->second.first / it->second.second;
            cluster.center += p;
            low = low.cwiseMin(p);
            high = high.cwiseMax(p);
            cluster.keys.push_back(k);
            candidates.erase(it);
            for (int x = -1; x <= 1; ++x)
                for (int y = -1; y <= 1; ++y)
                    for (int z = -1; z <= 1; ++z) {
                        const Key neighbor{k[0] + x, k[1] + y, k[2] + z};
                        if (candidates.count(neighbor)) pending.push(neighbor);
                    }
        }
        cluster.size = cluster.keys.size();
        if (cluster.size < static_cast<std::size_t>(config_.cluster_min_cells) ||
            cluster.size > static_cast<std::size_t>(config_.cluster_max_cells) ||
            (high - low).maxCoeff() > config_.cluster_max_extent) continue;
        cluster.center /= cluster.size;
        cluster.time = time;
        std::size_t best = tracks_.size();
        double distance = config_.track_distance;
        for (std::size_t i = 0; i < tracks_.size(); ++i) {
            const auto& track = tracks_[i];
            const double dt = time - track.time;
            if (used.count(i) || dt <= 0.0 || dt > 0.5 ||
                cluster.size < track.size / 2 || cluster.size > track.size * 2) continue;
            const double d = (cluster.center - track.center).norm();
            if (d < distance) { best = i; distance = d; }
        }
        if (best != tracks_.size()) {
            used.insert(best);
            const auto& track = tracks_[best];
            cluster.velocity = (cluster.center - track.center) / (time - track.time);
            const double speed = cluster.velocity.norm();
            const bool direction = track.velocity.norm() < config_.min_speed ||
                cluster.velocity.normalized().dot(track.velocity.normalized()) > 0.7;
            cluster.streak = speed >= config_.min_speed && speed <= config_.max_speed && direction ?
                track.streak + 1 : 0;
            if (cluster.streak >= config_.motion_frames) {
                moving_.insert(cluster.keys.begin(), cluster.keys.end());
                moving_.insert(track.keys.begin(), track.keys.end());
                ++stats_.moving_clusters;
            }
        }
        next.push_back(std::move(cluster));
    }
    tracks_ = std::move(next);
    std::vector<State> result;
    result.reserve(world.size());
    for (const auto& p : world) {
        const auto s = state(p);
        result.push_back(s);
        ++stats_.points[static_cast<int>(s)];
    }
    stats_.cells = cells_.size();
    return result;
}

void Filter::commit(const std::vector<pointWithVar>& points, const std::vector<State>& mask,
                    const V3D& origin, double time, bool reliable)
{
    if (!reliable || !stats_.reliable) {
        tracks_.clear();
        moving_.clear();
        stats_.reliable = false;
        return;
    }
    if (mask.size() != points.size() || time != prepared_time_ ||
        time <= committed_time_ || !origin.allFinite()) return;
    committed_time_ = time;
    // Commit destructive evidence only after the current local update was validated.
    for (const auto& k : moving_) {
        auto it = cells_.find(k);
        if (it == cells_.end()) continue;
        revoke(it->second);
        it->second.state = State::DYNAMIC;
        it->second.dynamic_until = time + config_.dynamic_hold;
    }
    std::map<Key, std::vector<pointWithVar>> hits;
    for (std::size_t i = 0; i < points.size(); ++i) {
        const auto& p = points[i];
        if (!p.point_w.allFinite() || p.point_w.cwiseAbs().maxCoeff() > 1e6 ||
            (p.point_w - origin).norm() > config_.max_range) continue;
        const auto k = key(p.point_w);
        if (mask[i] == State::DYNAMIC) {
            if (!cells_.count(k) && cells_.size() >= static_cast<std::size_t>(config_.max_cells)) continue;
            auto& c = cells_[k];
            const bool new_motion = moving_.count(k) || c.state != State::DYNAMIC;
            revoke(c);
            c.center = p.point_w;
            c.last = time;
            c.state = State::DYNAMIC;
            if (new_motion) c.dynamic_until = time + config_.dynamic_hold;
            continue;
        }
        auto& samples = hits[k];
        if (samples.size() < static_cast<std::size_t>(config_.samples_per_cell)) samples.push_back(p);
    }
    std::set<Key> free;
    // Only measured rays crossing a stored centroid provide free-space evidence.
    // Missing returns, rays ending at an occluder, and unseen directions do not.
    const std::size_t stride = std::max<std::size_t>(1, points.size() / config_.max_rays + 1);
    for (std::size_t i = 0; i < points.size(); i += stride) {
        const V3D ray = points[i].point_w - origin;
        const double range = ray.norm();
        if (!ray.allFinite() || range <= 2 * config_.cell_size || range > config_.max_range) continue;
        const V3D direction = ray / range;
        for (double d = config_.cell_size; d < range - 1.5 * config_.cell_size;
             d += config_.cell_size * 0.5) {
            const auto k = key(origin + direction * d);
            auto it = cells_.find(k);
            if (it == cells_.end() || hits.count(k) || it->second.state == State::DYNAMIC) continue;
            const V3D delta = it->second.center - origin;
            const double along = delta.dot(direction);
            if (along > config_.cell_size && along < range - 1.5 * config_.cell_size &&
                (delta - along * direction).norm() < config_.cell_size * 0.25) free.insert(k);
        }
    }
    for (const auto& k : free) {
        auto& c = cells_.at(k);
        if (time - c.last_free > config_.temporary_ttl) c.free = 0;
        c.last_free = time;
        if (++c.free >= config_.free_frames) {
            revoke(c);
            c.state = State::DYNAMIC;
            c.dynamic_until = time + config_.dynamic_hold;
            c.last = time;
        }
    }
    for (const auto& hit : hits) {
        auto it = cells_.find(hit.first);
        if (it == cells_.end() && cells_.size() >= static_cast<std::size_t>(config_.max_cells)) {
            ++stats_.capacity_rejected;
            continue;
        }
        auto& c = cells_[hit.first];
        if (c.state == State::DYNAMIC && time < c.dynamic_until) continue;
        V3D center = V3D::Zero();
        for (const auto& p : hit.second) center += p.point_w;
        center /= hit.second.size();
        const bool agreement = c.hits > 0 && time - c.last <= config_.temporary_ttl &&
            (center - c.center).norm() <= config_.consistency_distance;
        if (c.state != State::CONFIRMED_STATIC) {
            if (!agreement) { c.hits = 0; c.first = time; }
            ++c.hits;
            c.state = c.hits >= config_.confirmation_frames && time - c.first >= config_.confirmation_time ?
                State::CONFIRMED_STATIC : State::TENTATIVE_STATIC;
        }
        c.center = center;
        c.last = time;
        c.free = 0;
        c.samples = hit.second;
    }
    for (auto it = cells_.begin(); it != cells_.end();) {
        if (it->second.state != State::CONFIRMED_STATIC &&
            time - it->second.last > std::max(config_.temporary_ttl, config_.dynamic_hold)) it = cells_.erase(it);
        else ++it;
    }
    stats_.cells = cells_.size();
}

std::vector<pointWithVar> Filter::temporaryPoints(double time) const
{
    std::vector<pointWithVar> out;
    for (const auto& entry : cells_) {
        const auto& c = entry.second;
        if (c.state != State::TENTATIVE_STATIC || time - c.last > config_.temporary_ttl) continue;
        for (const auto& p : c.samples) {
            if (out.size() >= static_cast<std::size_t>(config_.max_temporary_points)) return out;
            if (estimationAllowed(p.point_w)) out.push_back(p);
        }
    }
    return out;
}

std::vector<pointWithVar> Filter::confirmedPoints() const
{
    std::vector<pointWithVar> out;
    for (const auto& entry : cells_) if (entry.second.state == State::CONFIRMED_STATIC)
        out.insert(out.end(), entry.second.samples.begin(), entry.second.samples.end());
    return out;
}

void loadConfig(ros::NodeHandle& nh, Config& c)
{
    nh.param<bool>("dynamic_filter/enabled", c.enabled, c.enabled);
    auto positive = [&](const std::string& name, auto& value) {
        nh.param("dynamic_filter/" + name, value, value);
        if (!std::isfinite(static_cast<double>(value)) || value <= 0)
            throw std::invalid_argument("dynamic_filter/" + name + " must be positive and finite");
    };
    positive("cell_size", c.cell_size);
    positive("consistency_distance", c.consistency_distance);
    positive("confirmation_time", c.confirmation_time);
    positive("confirmation_frames", c.confirmation_frames);
    positive("free_frames", c.free_frames);
    positive("temporary_ttl", c.temporary_ttl);
    positive("dynamic_hold", c.dynamic_hold);
    positive("track_distance", c.track_distance);
    positive("min_speed", c.min_speed);
    positive("max_speed", c.max_speed);
    positive("motion_frames", c.motion_frames);
    positive("cluster_min_cells", c.cluster_min_cells);
    positive("cluster_max_cells", c.cluster_max_cells);
    positive("cluster_max_extent", c.cluster_max_extent);
    positive("max_range", c.max_range);
    positive("max_position_sigma", c.max_position_sigma);
    positive("max_rotation_sigma", c.max_rotation_sigma);
    positive("bootstrap_position_sigma", c.bootstrap_position_sigma);
    positive("bootstrap_rotation_sigma", c.bootstrap_rotation_sigma);
    positive("max_correction", c.max_correction);
    positive("max_cells", c.max_cells);
    positive("samples_per_cell", c.samples_per_cell);
    positive("max_rays", c.max_rays);
    positive("max_temporary_points", c.max_temporary_points);
    if (c.min_speed >= c.max_speed || c.cluster_min_cells > c.cluster_max_cells ||
        c.confirmation_frames <= c.motion_frames || c.temporary_ttl < c.confirmation_time ||
        c.cell_size < 0.05 || c.cell_size > 2.0 || c.max_range > 200.0 ||
        c.max_cells > 60000 || c.samples_per_cell > 100)
        throw std::invalid_argument("inconsistent dynamic_filter confirmation, tracking or TTL limits");
}
} // namespace dynamic_filter
