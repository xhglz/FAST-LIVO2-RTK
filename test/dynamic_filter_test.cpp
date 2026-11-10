#include "dynamic_filter.h"
#include "voxel_map.h"
#include "backend_events.h"
#include "vio.h"
#include <gtest/gtest.h>

namespace {
using dynamic_filter::Filter;
using dynamic_filter::State;

std::vector<pointWithVar> points(const std::vector<V3D>& p)
{
    std::vector<pointWithVar> out(p.size());
    for (std::size_t i = 0; i < p.size(); ++i) {
        out[i].point_w = out[i].point_b = p[i];
        out[i].var = M3D::Identity() * 0.001;
    }
    return out;
}
void observe(Filter& f, const std::vector<V3D>& p, double t, bool reliable = true)
{
    const auto mask = f.prepare(p, t, reliable, {});
    f.commit(points(p), mask, V3D::Zero(), t, reliable);
}
std::vector<V3D> object(double y)
{
    std::vector<V3D> p;
    for (int a = 0; a < 3; ++a)
        for (int b = 0; b < 3; ++b) p.emplace_back(5.05, y + a * 0.3, 1.05 + b * 0.3);
    return p;
}
VoxelMapConfig voxelConfig()
{
    VoxelMapConfig c{};
    c.max_voxel_size_ = 2.0;
    c.max_layer_ = 1;
    c.max_iterations_ = 3;
    c.layer_init_num_ = {5, 5};
    c.max_points_num_ = 100;
    c.planner_threshold_ = 0.01;
    c.sigma_num_ = 3.0;
    return c;
}
} // namespace

TEST(DynamicConfidence, UnknownNeedsMultipleFramesAndElapsedTime)
{
    Filter f;
    const auto p = object(0.05);
    auto mask = f.prepare(p, 1.0, true, {});
    EXPECT_EQ(mask.front(), State::UNKNOWN);
    f.commit(points(p), mask, V3D::Zero(), 1.0, true);
    EXPECT_EQ(f.state(p.front()), State::TENTATIVE_STATIC);
    for (int i = 1; i < 6; ++i) observe(f, p, 1.0 + i * 0.05);
    EXPECT_EQ(f.state(p.front()), State::TENTATIVE_STATIC);
    for (int i = 1; i <= 6; ++i) observe(f, p, 1.25 + i * 0.2);
    EXPECT_EQ(f.state(p.front()), State::CONFIRMED_STATIC);
    EXPECT_TRUE(f.permanentAllowed(p.front()));
}

TEST(DynamicConfidence, NewRegionUsesTemporaryLayerAndExpires)
{
    Filter f;
    observe(f, object(0.05), 1.0);
    EXPECT_FALSE(f.temporaryPoints(1.1).empty());
    EXPECT_TRUE(f.confirmedPoints().empty());
    EXPECT_TRUE(f.temporaryPoints(2.1).empty());
    observe(f, {}, 2.1);
    EXPECT_EQ(f.stats().cells, 0u);
}

TEST(DynamicMotion, ConsistentClusterMotionExcludedBeforeStaticPromotion)
{
    Filter f;
    for (int i = 0; i < 8; ++i) {
        const auto p = object(0.05 + i * 0.3);
        const auto mask = f.prepare(p, 1.0 + i * 0.1, true, {});
        if (i == 0) EXPECT_EQ(mask.front(), State::UNKNOWN);
        if (i >= 3) {
            EXPECT_EQ(mask.front(), State::DYNAMIC);
            EXPECT_FALSE(f.estimationAllowed(p.front()));
            EXPECT_GT(f.stats().moving_clusters, 0u);
        }
        f.commit(points(p), mask, V3D::Zero(), 1.0 + i * 0.1, true);
    }
    EXPECT_TRUE(f.confirmedPoints().empty());
}

TEST(DynamicMotion, StationaryWallRetainedWhileObjectMoves)
{
    Filter f;
    const std::vector<V3D> wall{V3D(10.05, 3.05, 1.05), V3D(10.05, 3.35, 1.05)};
    for (int i = 0; i < 7; ++i) observe(f, wall, 1.0 + i * 0.2);
    ASSERT_TRUE(f.permanentAllowed(wall.front()));
    for (int i = 0; i < 8; ++i) {
        auto scan = object(0.05 + i * 0.3);
        scan.insert(scan.end(), wall.begin(), wall.end());
        observe(f, scan, 2.4 + i * 0.1);
        EXPECT_TRUE(f.permanentAllowed(wall.front()));
    }
    EXPECT_EQ(f.confirmedPoints().size(), wall.size());
}

TEST(DynamicCleanup, OcclusionMissingAndSingleFreeRayNeverDelete)
{
    Filter f;
    const V3D old(5.05, 0.05, 0.05);
    for (int i = 0; i < 7; ++i) observe(f, {old}, 1.0 + i * 0.2);
    ASSERT_TRUE(f.permanentAllowed(old));
    for (int i = 0; i < 10; ++i) observe(f, {old * 0.5}, 2.4 + i * 0.1);
    EXPECT_TRUE(f.permanentAllowed(old));
    observe(f, {}, 3.5);
    EXPECT_TRUE(f.permanentAllowed(old));
    observe(f, {old * 2.0}, 3.6);
    EXPECT_TRUE(f.permanentAllowed(old));
    EXPECT_TRUE(f.revoked().empty());
}

TEST(DynamicCleanup, MultipleVisibleFreeRaysRevokeOnlyContradictedCells)
{
    Filter f;
    const V3D old(5.05, 0.05, 0.05), wall(10.05, 2.05, 1.05);
    for (int i = 0; i < 7; ++i) observe(f, {old, wall}, 1.0 + i * 0.2);
    for (int i = 0; i < 6; ++i) observe(f, {old * 2.0, wall}, 2.4 + i * 0.1);
    EXPECT_EQ(f.state(old), State::DYNAMIC);
    EXPECT_FALSE(f.permanentAllowed(old));
    EXPECT_FALSE(f.revoked().empty());
    EXPECT_TRUE(f.permanentAllowed(wall));
}

TEST(DynamicSafety, UnreliablePoseCannotPromoteOrDelete)
{
    Filter f;
    const V3D old(5.05, 0.05, 0.05), fresh(7.05, 3.05, 1.05);
    for (int i = 0; i < 7; ++i) observe(f, {old}, 1.0 + i * 0.2);
    for (int i = 0; i < 10; ++i) observe(f, {old * 2.0, fresh}, 2.4 + i * 0.1, false);
    EXPECT_TRUE(f.permanentAllowed(old));
    EXPECT_EQ(f.state(fresh), State::UNKNOWN);
    EXPECT_TRUE(f.revoked().empty());
}

TEST(DynamicConfidence, StoppedObjectMustReconfirmAfterDynamicHoldExpires)
{
    Filter f;
    for (int i = 0; i < 5; ++i) observe(f, object(0.05 + i * 0.3), 1.0 + i * 0.1);
    const auto stopped = object(1.25);
    ASSERT_EQ(f.state(stopped.front()), State::DYNAMIC);
    for (int i = 1; i <= 8; ++i) observe(f, stopped, 1.4 + i * 0.1);
    EXPECT_FALSE(f.permanentAllowed(stopped.front()));
    for (int i = 0; i < 15; ++i) observe(f, stopped, 2.5 + i * 0.1);
    EXPECT_TRUE(f.permanentAllowed(stopped.front()));
}

TEST(DynamicSafety, TimeRollbackAndDuplicateCannotAccumulateEvidence)
{
    Filter f;
    auto p = object(0.05);
    observe(f, p, 2.0);
    for (int i = 0; i < 20; ++i) observe(f, p, i % 2 ? 2.0 : 1.9);
    EXPECT_EQ(f.state(p.front()), State::TENTATIVE_STATIC);
    EXPECT_FALSE(f.stats().reliable);
}

TEST(DynamicSafety, CellAndTemporaryPointBudgetsAreBounded)
{
    dynamic_filter::Config c;
    c.max_cells = 3;
    c.max_temporary_points = 2;
    Filter f(c);
    observe(f, object(0.05), 1.0);
    EXPECT_LE(f.stats().cells, 3u);
    EXPECT_GT(f.stats().capacity_rejected, 0u);
    EXPECT_LE(f.temporaryPoints(1.0).size(), 2u);
}

TEST(DynamicMap, BuildAndUpdateGatesAndTemporaryResidualMask)
{
    auto config = voxelConfig();
    std::unordered_map<VOXEL_LOCATION, VoxelOctoTree*> empty;
    VoxelMapManager map(config, empty);
    map.extR_ = M3D::Identity();
    map.extT_ = V3D::Zero();
    std::vector<V3D> plane;
    for (int x = 1; x < 6; ++x)
        for (int y = 1; y < 6; ++y) plane.emplace_back(x * 0.2, y * 0.2, 0.5);
    const auto samples = points(plane);
    map.permanent_point_allowed_ = [](const V3D&) { return false; };
    for (const auto& p : plane) {
        PointType q;
        q.x = p.x(); q.y = p.y(); q.z = p.z();
        map.feats_down_body_->push_back(q);
        map.feats_down_world_->push_back(q);
    }
    map.BuildVoxelMap();
    map.UpdateVoxelMap(samples);
    EXPECT_TRUE(map.voxel_map_.empty());
    map.replaceTemporaryMap(samples);
    EXPECT_FALSE(map.temporary_map_.empty());
    auto input = samples;
    map.estimation_mask_.assign(samples.size(), 0);
    map.BuildResidualListOMP(input, map.ptpl_list_);
    EXPECT_TRUE(map.ptpl_list_.empty());
    map.estimation_mask_.assign(samples.size(), 1);
    map.BuildResidualListOMP(input, map.ptpl_list_);
    EXPECT_FALSE(map.ptpl_list_.empty());
    map.replaceTemporaryMap({});
    EXPECT_TRUE(map.temporary_map_.empty());
}

TEST(DynamicVisual, PrunesHistoricalPatchOwnersAndSubmapPointers)
{
    VIOManager manager;
    manager.visual_submap = new SubSparseMap;
    manager.total_points = 1;
    auto* voxel = new VOXEL_POINTS(1);
    auto* point = new VisualPoint(V3D(5, 0, 0));
    voxel->voxel_points.push_back(point);
    manager.feat_map[VOXEL_LOCATION()] = voxel;
    manager.visual_submap->voxel_points.push_back(point);
    manager.retrieve_voxel_points.push_back(point);
    manager.static_point_allowed = [](const V3D&) { return false; };
    manager.pruneNonStaticPoints();
    EXPECT_TRUE(manager.feat_map.empty());
    EXPECT_TRUE(manager.visual_submap->voxel_points.empty());
    EXPECT_EQ(manager.total_points, 0);
    EXPECT_EQ(manager.retrieve_voxel_points.front(), nullptr);
}

TEST(DynamicMap, RebuildPollutedRootFromRetainedWallSamples)
{
    auto config = voxelConfig();
    std::unordered_map<VOXEL_LOCATION, VoxelOctoTree*> empty;
    VoxelMapManager map(config, empty);
    std::vector<V3D> wall;
    for (int x = 1; x < 6; ++x)
        for (int y = 1; y < 6; ++y) wall.emplace_back(x * 0.2, y * 0.2, 0.5);
    map.UpdateVoxelMap(points(wall));
    const V3D pollution(0.1, 0.1, 1.5);
    map.UpdateVoxelMap(points({pollution}));
    map.rebuildRoots({pollution}, points(wall));
    ASSERT_EQ(map.voxel_map_.size(), 1u);
    EXPECT_TRUE(map.geometryConsistent(wall.front(), 0.02));
    EXPECT_FALSE(map.geometryConsistent(pollution, 0.02));
    for (auto& entry : map.voxel_map_) delete entry.second;
    map.voxel_map_.clear();
}

TEST(DynamicMap, BackendRevocationUsesRawOdomNotOptimizedPose)
{
    backend::KeyframeEvent removal;
    removal.removed_cell_size = 0.3;
    removal.removed_cells_odom = {{20, 0, 0}};
    Eigen::Isometry3d raw = Eigen::Isometry3d::Identity();
    raw.translation() = V3D(5, 0, 0);
    PointCloudXYZRGB::Ptr cloud(new PointCloudXYZRGB);
    PointTypeRGB p;
    p.x = 1.05; p.y = 0.05; p.z = 0.05;
    cloud->push_back(p);
    p.x = 2.05;
    cloud->push_back(p);
    const auto retained = backend::removeMapCells(cloud, raw, removal);
    ASSERT_EQ(retained->size(), 1u);
    EXPECT_FLOAT_EQ(retained->front().x, 2.05f);
    EXPECT_EQ(cloud->size(), 2u);
}

int main(int argc, char** argv)
{
    ros::Time::init();
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
