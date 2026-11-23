#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "swarm_safety_geofence_monitor/geofence_monitor.hpp"

using namespace swarm::safety;
using Eigen::Vector3d;

class GeofenceMonitorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize ROS
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<GeofenceMonitorNode>();
        
        // Set up test geofence: square inclusion zone (0,0) to (100,100)
        GeofencePolygon inclusion;
        inclusion.id = 1;
        inclusion.vertices = {
            {0, 0, 0}, {100, 0, 0}, {100, 100, 0}, {0, 100, 0}
        };
        inclusion.min_altitude_m = 0.0;
        inclusion.max_altitude_m = 100.0;
        inclusion.is_exclusion = false;
        
        // Exclusion zone at (200,200) radius ~20
        GeofencePolygon exclusion;
        exclusion.id = 2;
        for (int i = 0; i < 8; ++i) {
            double angle = i * 2 * M_PI / 8;
            exclusion.vertices.push_back({200 + 20 * cos(angle), 200 + 20 * sin(angle), 0});
        }
        exclusion.min_altitude_m = 0.0;
        exclusion.max_altitude_m = 50.0;
        exclusion.is_exclusion = true;
        
        std::lock_guard<std::mutex> lock(node_->config_mutex_);
        node_->config_.polygons = {inclusion, exclusion};
        node_->config_.buffer_distance_m = 10.0;
        node_->config_.enabled = true;
        node_->config_.trigger_rtl = true;
    }
    
    void TearDown() override {
        rclcpp::shutdown();
    }
    
    std::shared_ptr<GeofenceMonitorNode> node_;
};

// ============================================================================
// Point-in-Polygon Tests
// ============================================================================

TEST_F(GeofenceMonitorTest, PointInConvexPolygon_Inside) {
    Vector3d pt(50, 50, 50);
    GeofencePolygon poly;
    poly.vertices = {{0,0,0}, {100,0,0}, {100,100,0}, {0,100,0}};
    
    EXPECT_TRUE(node_->pointInPolygon(pt, poly));
}

TEST_F(GeofenceMonitorTest, PointInConvexPolygon_Outside) {
    Vector3d pt(150, 50, 50);
    GeofencePolygon poly;
    poly.vertices = {{0,0,0}, {100,0,0}, {100,100,0}, {0,100,0}};
    
    EXPECT_FALSE(node_->pointInPolygon(pt, poly));
}

TEST_F(GeofenceMonitorTest, PointInConcavePolygon_Inside) {
    Vector3d pt(200, 200, 25);
    GeofencePolygon poly;
    for (int i = 0; i < 8; ++i) {
        double angle = i * 2 * M_PI / 8;
        poly.vertices.push_back({200 + 20 * cos(angle), 200 + 20 * sin(angle), 0});
    }
    
    EXPECT_TRUE(node_->pointInPolygon(pt, poly));
}

TEST_F(GeofenceMonitorTest, PointInConcavePolygon_Outside) {
    Vector3d pt(250, 200, 25);
    GeofencePolygon poly;
    for (int i = 0; i < 8; ++i) {
        double angle = i * 2 * M_PI / 8;
        poly.vertices.push_back({200 + 20 * cos(angle), 200 + 20 * sin(angle), 0});
    }
    
    EXPECT_FALSE(node_->pointInPolygon(pt, poly));
}

// ============================================================================
// Distance to Polygon Tests
// ============================================================================

TEST_F(GeofenceMonitorTest, DistanceToPolygon_Inside) {
    Vector3d pt(10, 50, 0);  // 10m from left edge
    GeofencePolygon poly;
    poly.vertices = {{0,0,0}, {100,0,0}, {100,100,0}, {0,100,0}};
    
    double dist = node_->pointToPolygonDistance(pt, poly);
    EXPECT_NEAR(dist, 10.0, 0.01);
}

TEST_F(GeofenceMonitorTest, DistanceToPolygon_Outside) {
    Vector3d pt(-10, 50, 0);  // 10m outside left edge
    GeofencePolygon poly;
    poly.vertices = {{0,0,0}, {100,0,0}, {100,100,0}, {0,100,0}};
    
    double dist = node_->pointToPolygonDistance(pt, poly);
    EXPECT_NEAR(dist, 10.0, 0.01);
}

TEST_F(GeofenceMonitorTest, DistanceToPolygon_Corner) {
    Vector3d pt(-5, -5, 0);  // Near corner (0,0)
    GeofencePolygon poly;
    poly.vertices = {{0,0,0}, {100,0,0}, {100,100,0}, {0,100,0}};
    
    double dist = node_->pointToPolygonDistance(pt, poly);
    EXPECT_NEAR(dist, 5.0 * sqrt(2), 0.01);
}

TEST_F(GeofenceMonitorTest, DistanceToOctagon) {
    Vector3d center(200, 200, 0);
    GeofencePolygon poly;
    for (int i = 0; i < 8; ++i) {
        double angle = i * 2 * M_PI / 8;
        poly.vertices.push_back({200 + 20 * cos(angle), 200 + 20 * sin(angle), 0});
    }
    
    double dist = node_->pointToPolygonDistance(center, poly);
    double expected = 20.0 * cos(M_PI / 8);  // apothem
    EXPECT_NEAR(dist, expected, 0.1);
}

// ============================================================================
// Geofence Action Logic Tests
// ============================================================================

TEST_F(GeofenceMonitorTest, InclusionZoneInside_NoAction) {
    Vector3d pt(50, 50, 50);
    GeofenceAction action = node_->evaluatePosition(pt);
    EXPECT_EQ(action, GeofenceAction::NONE);
}

TEST_F(GeofenceMonitorTest, InclusionZoneOutsideBuffer_RTL) {
    Vector3d pt(-20, 50, 50);  // 20m outside, beyond 10m buffer
    GeofenceAction action = node_->evaluatePosition(pt);
    EXPECT_EQ(action, GeofenceAction::RTL);
}

TEST_F(GeofenceMonitorTest, InclusionZoneOutsideWithinBuffer_Warning) {
    Vector3d pt(-5, 50, 50);  // 5m outside, within 10m buffer
    GeofenceAction action = node_->evaluatePosition(pt);
    EXPECT_EQ(action, GeofenceAction::WARNING);
}

TEST_F(GeofenceMonitorTest, ExclusionZoneInside_Terminate) {
    Vector3d pt(200, 200, 25);  // Inside exclusion zone
    GeofenceAction action = node_->evaluatePosition(pt);
    EXPECT_EQ(action, GeofenceAction::TERMINATE);
}

TEST_F(GeofenceMonitorTest, ExclusionZoneApproach_Warning) {
    Vector3d pt(230, 200, 25);  // 10m from exclusion edge
    GeofenceAction action = node_->evaluatePosition(pt);
    EXPECT_EQ(action, GeofenceAction::WARNING);
}

TEST_F(GeofenceMonitorTest, ExclusionZoneAltitudeExempt) {
    Vector3d pt(200, 200, 100);  // Above exclusion max altitude (50m)
    GeofenceAction action = node_->evaluatePosition(pt);
    EXPECT_EQ(action, GeofenceAction::NONE);
}

TEST_F(GeofenceMonitorTest, MultiplePolygons_WorstAction) {
    // Position that triggers WARNING from both
    Vector3d pt(230, 200, 25);
    GeofenceAction action = node_->evaluatePosition(pt);
    EXPECT_EQ(action, GeofenceAction::WARNING);
    
    // Position that triggers TERMINATE (inside exclusion)
    Vector3d pt2(200, 200, 25);
    GeofenceAction action2 = node_->evaluatePosition(pt2);
    EXPECT_EQ(action2, GeofenceAction::TERMINATE);
}

TEST_F(GeofenceMonitorTest, DisabledGeofence_NoAction) {
    std::lock_guard<std::mutex> lock(node_->config_mutex_);
    node_->config_.enabled = false;
    
    Vector3d pt(-100, 50, 50);
    GeofenceAction action = node_->evaluatePosition(pt);
    EXPECT_EQ(action, GeofenceAction::NONE);
}

// ============================================================================
// Determinism Tests (DAL A requirement)
// ============================================================================

TEST_F(GeofenceMonitorTest, RepeatedCallsSameResult) {
    Vector3d pt(50, 50, 50);
    GeofenceAction a1 = node_->evaluatePosition(pt);
    GeofenceAction a2 = node_->evaluatePosition(pt);
    GeofenceAction a3 = node_->evaluatePosition(pt);
    
    EXPECT_EQ(a1, a2);
    EXPECT_EQ(a2, a3);
}

TEST_F(GeofenceMonitorTest, DistanceConsistent) {
    Vector3d pt(50, 50, 0);
    GeofencePolygon poly;
    poly.vertices = {{0,0,0}, {100,0,0}, {100,100,0}, {0,100,0}};
    
    double d1 = node_->pointToPolygonDistance(pt, poly);
    double d2 = node_->pointToPolygonDistance(pt, poly);
    double d3 = node_->pointToPolygonDistance(pt, poly);
    
    EXPECT_FLOAT_EQ(d1, d2);
    EXPECT_FLOAT_EQ(d2, d3);
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST(GeofenceEdgeCaseTest, EmptyPolygon) {
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<GeofenceMonitorNode>();
    
    GeofencePolygon empty;
    Vector3d pt(0, 0, 0);
    
    EXPECT_FALSE(node->pointInPolygon(pt, empty));
    EXPECT_FLOAT_EQ(node->pointToPolygonDistance(pt, empty), 1e9);
    
    rclcpp::shutdown();
}

TEST(GeofenceEdgeCaseTest, LargeCoordinates) {
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<GeofenceMonitorNode>();
    
    GeofencePolygon large;
    large.vertices = {
        {1e6, 1e6, 0}, {1e6+100, 1e6, 0}, {1e6+100, 1e6+100, 0}, {1e6, 1e6+100, 0}
    };
    
    Vector3d inside(1e6+50, 1e6+50, 0);
    Vector3d outside(1e6-50, 1e6+50, 0);
    
    EXPECT_TRUE(node->pointInPolygon(inside, large));
    EXPECT_FALSE(node->pointInPolygon(outside, large));
    
    rclcpp::shutdown();
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}