#include "vio.h"
#include <gtest/gtest.h>

TEST(VisualReference, CachedMeansMatchFreshNormalization)
{
    VIOManager manager;
    manager.visual_submap = new SubSparseMap();
    manager.total_points = 1;
    manager.patch_size_total = 4;
    manager.update_flag = {1};
    VisualPoint point(Vector3d(0, 0, 1));
    point.is_normal_initialized_ = true;
    point.normal_ = Vector3d::UnitZ();
    for (int i = 0; i < 6; ++i) {
        auto* patch = new float[4];
        for (int j = 0; j < 4; ++j) patch[j] = 10.0f * (i + 1) + j;
        auto* feature = new Feature(&point, patch, Vector2d::Zero(),
            Vector3d::UnitZ(), SE3(), 0);
        feature->id_ = i;
        point.addFrameRef(feature);
    }
    manager.visual_submap->voxel_points.push_back(&point);
    const unordered_map<VOXEL_LOCATION, VoxelOctoTree*> empty_map;
    for (int pass = 0; pass < 3; ++pass) {
        manager.updateReferencePatch(empty_map);
        ASSERT_TRUE(point.has_ref_patch_);
        for (const auto* feature : point.obs_) {
            EXPECT_FLOAT_EQ(feature->mean_, 10.0f * (feature->id_ + 1) + 1.5f);
            EXPECT_NEAR(feature->score_, 2.0f, 1e-6);
        }
    }
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
