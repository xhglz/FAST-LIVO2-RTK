#include "resource_limits.h"
#include "voxel_map.h"
#include "vio.h"
#include <gtest/gtest.h>
#include <limits>

TEST(ResourceBudget, OverflowSafeAdmission)
{
    EXPECT_TRUE(resources::fits(9, 1, 10));
    EXPECT_FALSE(resources::fits(10, 1, 10));
    EXPECT_FALSE(resources::fits(std::numeric_limits<std::size_t>::max(), 1, 10));
}

TEST(ResourceBudget, CloudIsBoundedAndInputUnmodified)
{
    PointCloudXYZRGB::Ptr cloud(new PointCloudXYZRGB);
    for (int i = 0; i < 100; ++i) { PointTypeRGB p; p.x = i; cloud->push_back(p); }
    auto bounded = resources::boundedCloud(cloud, 10);
    ASSERT_EQ(bounded->size(), 10u);
    EXPECT_EQ(cloud->size(), 100u);
    EXPECT_EQ(bounded->front().x, 0);
    EXPECT_EQ(bounded->back().x, 90);
}

TEST(ResourceBudget, ExpiredCameraPairsLeaveRoomForFreshRecovery)
{
    std::deque<int> messages{1, 2, 3};
    std::deque<double> times{10, 20, 30};
    EXPECT_EQ(resources::purgeExpired(messages, times, 20), 2u);
    EXPECT_EQ(messages.size(), 1u);
    EXPECT_EQ(times.front(), 30);
    EXPECT_EQ(messages.front(), 3);
}

TEST(ResourceBudget, ArchiveChunksNeverExceedCapacity)
{
    std::vector<int> buffer, source(25, 1);
    int chunks = 0;
    auto rejected = resources::appendChunks(buffer, source, 10, [&] {
        EXPECT_EQ(buffer.size(), 10u); buffer.clear(); ++chunks; return true;
    });
    EXPECT_EQ(rejected, 0u);
    EXPECT_EQ(chunks, 2);
    EXPECT_EQ(buffer.size(), 5u);
}

TEST(ResourceBudget, ArchiveFailureRetainsOldDataAndRefusesGrowth)
{
    std::vector<int> buffer(10, 1), source(25, 2);
    EXPECT_EQ(resources::appendChunks(buffer, source, 10, [] { return false; }), 25u);
    EXPECT_EQ(buffer.size(), 10u);
    EXPECT_EQ(buffer.front(), 1);
}

namespace {
VoxelMapConfig mapConfig()
{
    VoxelMapConfig c{};
    c.max_voxel_size_ = 1.0; c.max_layer_ = 0; c.layer_init_num_ = {5};
    c.max_points_num_ = 50; c.planner_threshold_ = 0.01;
    c.beam_err_ = 0.05; c.dept_err_ = 0.05;
    return c;
}
}

TEST(ResourceMap, BuildAndUpdateEnforceRootLimit)
{
    auto c = mapConfig();
    std::unordered_map<VOXEL_LOCATION, VoxelOctoTree*> empty;
    VoxelMapManager map(c, empty);
    map.extR_ = M3D::Identity(); map.extT_ = V3D::Zero();
    map.resource_max_roots = 2;
    std::vector<pointWithVar> points;
    for (int i = 1; i < 20; ++i) {
        PointType q; q.x = i * 2; q.y = 0.1; q.z = 0.1;
        map.feats_down_body_->push_back(q); map.feats_down_world_->push_back(q);
        pointWithVar p{}; p.point_w = V3D(q.x, q.y, q.z); p.var = M3D::Identity();
        points.push_back(p);
    }
    map.BuildVoxelMap();
    EXPECT_EQ(map.voxel_map_.size(), 2u);
    const auto revision = map.map_revision_;
    map.UpdateVoxelMap(points);
    EXPECT_EQ(map.voxel_map_.size(), 2u);
    EXPECT_EQ(map.map_revision_, revision + 2);
    EXPECT_GT(map.resource_rejected_points, 0u);
}

TEST(ResourceVisual, CapAndPruningKeepOwnersAndReferencesConsistent)
{
    VIOManager manager;
    manager.visual_submap = new SubSparseMap;
    manager.resource_max_points = 2;
    for (int i = 0; i < 10; ++i) manager.insertPointIntoVoxelMap(new VisualPoint(V3D(i, 0, 0)));
    EXPECT_EQ(manager.resource_point_count, 2u);
    manager.static_point_allowed = [](const V3D&) { return false; };
    manager.pruneNonStaticPoints();
    EXPECT_EQ(manager.resource_point_count, 0u);
    EXPECT_TRUE(manager.feat_map.empty());
    manager.insertPointIntoVoxelMap(new VisualPoint(V3D(0, 0, 0)));
    EXPECT_EQ(manager.resource_point_count, 1u);
}

TEST(ResourceVisual, ExpiredPatchReleasesReferenceAndImageHistory)
{
    VIOManager manager;
    manager.visual_submap = new SubSparseMap;
    vk::PinholeCamera camera(8, 8, 1, 1, 1, 4, 4);
    manager.new_frame_.reset(new Frame(&camera, cv::Mat::zeros(8, 8, CV_8UC1)));
    manager.new_frame_->id_ = 1000;
    manager.resource_frame_window = 10;
    auto* point = new VisualPoint(V3D(1, 0, 0));
    for (int id : {980, 995}) {
        auto* feature = new Feature(point, new float[1], V2D::Zero(), V3D::UnitZ(), SE3(), 0);
        feature->id_ = id;
        point->addFrameRef(feature);
        if (id == 980) { point->ref_patch = feature; point->has_ref_patch_ = true; }
    }
    manager.insertPointIntoVoxelMap(point);
    manager.pruneNonStaticPoints();
    ASSERT_EQ(point->obs_.size(), 1u);
    EXPECT_FALSE(point->has_ref_patch_);
    EXPECT_EQ(point->ref_patch, nullptr);
    manager.new_frame_->id_ = 1010;
    manager.pruneNonStaticPoints();
    EXPECT_TRUE(manager.feat_map.empty());
}

int main(int argc, char** argv)
{
    ros::Time::init();
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
