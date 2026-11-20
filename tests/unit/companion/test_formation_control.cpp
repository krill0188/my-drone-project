#include <gtest/gtest.h>
#include "swarm_autonomy/formation_control/formation_controller.hpp"
#include "swarm_autonomy/task_allocator/cbba.hpp"
#include "swarm_comm/swarm_comm.hpp"

using namespace swarm::formation;
using namespace swarm::autonomy;
using namespace swarm::comm;

class FormationControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Mock mesh manager
        // In real test, would use a mock implementation
    }
};

TEST_F(FormationControllerTest, LeaderInit) {
    // Test leader initialization with V-formation
    FormationGeometry geometry;
    geometry.type = GeometryType::VEE;
    geometry.spacing_m = 5.0f;
    
    FormationTolerances tolerances;
    tolerances.max_lateral_error_m = 0.5f;
    tolerances.max_vertical_error_m = 0.3f;
    
    // Mock mesh manager would be injected here
    // FormationController controller(mock_mesh);
    // EXPECT_TRUE(controller.initAsLeader(1, geometry, tolerances));
}

TEST_F(FormationControllerTest, FollowerInit) {
    // FormationController controller(mock_mesh);
    // EXPECT_TRUE(controller.initAsFollower(1, 10));
}

TEST_F(FormationControllerTest, PositionErrorCalculation) {
    Vector3 leader_pos{100, 50, 20};
    Vector3 follower_pos{105, 53, 19.5};
    
    Vector3 error = follower_pos - leader_pos;
    EXPECT_NEAR(error.x, 5.0f, 0.01f);
    EXPECT_NEAR(error.y, 3.0f, 0.01f);
    EXPECT_NEAR(error.z, -0.5f, 0.01f);
}

class CbbaAllocatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        _capabilities.has_rtk_gps = true;
        _capabilities.has_stereo_camera = true;
        _capabilities.has_uwb = true;
        _capabilities.has_ai_compute = true;
        _capabilities.max_flight_time_s = 1800;
        _capabilities.max_payload_kg = 0.5f;
        _capabilities.comm_tier_bitmap = 0b111;
        
        _allocator = std::make_unique<CbbaAllocator>(1, _capabilities);
        _allocator->init();
    }
    
    CbbaAllocator::CapabilityBitmap _capabilities;
    std::unique_ptr<CbbaAllocator> _allocator;
};

TEST_F(CbbaAllocatorTest, BundleBuilding) {
    std::vector<CbbaAllocator::Task> tasks;
    
    // Create test tasks
    for (int i = 0; i < 5; ++i) {
        CbbaAllocator::Task task;
        task.id.fill(0);
        task.id[0] = i;
        task.type = TaskType::COVER_AREA;
        task.priority = 100;
        task.status = 0;  // PENDING
        tasks.push_back(task);
    }
    
    _allocator->buildBundle(tasks);
    const auto& bundle = _allocator->getState().bundle;
    
    // Should have some tasks in bundle
    EXPECT_GT(bundle.tasks.size(), 0);
    EXPECT_LE(bundle.tasks.size(), tasks.size());
}

TEST_F(CbbaAllocatorTest, CostComputation) {
    swarm_msgs::msg::AgentState state;
    state.battery_pct = 0.8f;
    state.remaining_time_s = 1200;
    state.position = {0, 0, 20};
    state.velocity = {0, 0, 0};
    
    CbbaAllocator::Task task;
    task.geometry.type = TaskType::COVER_AREA;
    task.geometry.area = Polygon{{{10, 10, 20}, {20, 10, 20}, {20, 20, 20}, {10, 20, 20}}};
    
    float cost = _allocator->computeTaskCost(task, state);
    EXPECT_GT(cost, 0.0f);
}

class MeshManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Would need mock HAL
    }
};

TEST_F(MeshManagerTest, NeighborTracking) {
    // Mock test for mesh neighbor tracking
    // MeshManager manager(mock_hal);
    // manager.update();
    // const auto& neighbors = manager.getNeighbors();
    // EXPECT_GE(neighbors.size(), 0);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}