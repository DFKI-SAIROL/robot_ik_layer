#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm> // for std::max, std::min
#include <limits>    // for std::numeric_limits
#include <memory>    // for std::unique_ptr

#include <Eigen/Dense>

// Pinocchio Includes
#include <pinocchio/fwd.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/multibody/model.hpp>

// Assuming rclcpp and visualization_msgs/msg/marker.hpp are included via vis.hpp or required by init
#include "vis.hpp"

// ====================================================================================
// SAFETY LAYER CLASS
// ====================================================================================

/**
 * @brief Safety layer class to adjust desired Cartesian poses and limit velocity.
 */
class SafetyLayer
{
public:
    /**
     * @brief Constructor for the SafetyLayer.
     */
    SafetyLayer();

    void init(Eigen::Vector3d &initial_position, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub);

    /**
     * @brief Adjusts the desired Cartesian pose to a safe pose by applying
     * iterative collision avoidance and workspace clamping on the position component.
     */
    pinocchio::SE3 adjustToSafePose(const pinocchio::SE3& robot_pose, const pinocchio::SE3& desired_pose) const;

    double getMaxSafeVelocity(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity);

    WorkspaceVisualizer vis_;

    double current_distance_to_obstacle;
    double current_distance_to_obstacle_along_velocity_direction;

    // --- Other Obstacle Members ---
    bool other_robot_check = false;

    Eigen::VectorXd other_q_;
    std::string other_prefix_;
    pinocchio::Model other_model_;
    std::unique_ptr<pinocchio::Data> other_data_;
    pinocchio::FrameIndex other_ee_frame_id_;

    void init_other();
   
private:

    AABB workspace_, effective_workspace_; // AABB for outer workspace boundary remains
    
    // All obstacles are now OBBs
    std::vector<OBB> forbidden_obstacles_; 
    
    // Dynamic Obstacle Definitions
    std::vector<LinkBoundingBox> other_robot_link_boxes_; 
    std::vector<OBB> current_other_robot_obstacles_;
    
    // --- Private Helper Functions for Dynamic Obstacles ---
    OBB gen_fixed_obb(const Eigen::Vector3d& minl, const Eigen::Vector3d& maxl);
    void updateOtherRobotKinematics();
    void transformBoundingBoxes();
    Eigen::Vector3d closestPointToOBB(const OBB& box, const Eigen::Vector3d& query_point) const;

    Eigen::Vector3d initial_position_;

    double safety_distance_ = 0.10;
    double min_velocity_ = 0.02;
    double max_velocity_ = 1.0;
    double safety_stopping_acceleration_ = 5.0;

    /**
     * @brief Clamps the given position vector within the allowed AABB.
     */
    Eigen::Vector3d clampToAABB(const Eigen::Vector3d& position) const;

    /**
     * @brief Calculates the necessary push-off vector from the single most critical violation.
     */
    Eigen::Vector3d calculatePushOff(const Eigen::Vector3d& current_position, const Eigen::Vector3d& robot_position) const;
    
    double intersectRayOBB(const Eigen::Vector3d& ray_origin, const Eigen::Vector3d& ray_dir, const OBB& obb) const;
    double getShortestDistanceToSafetyBoundary(const Eigen::Vector3d& query_position) const;
    double getDistanceAlongVelocity(const Eigen::Vector3d& query_position, const Eigen::Vector3d& query_velocity) const;
    
};