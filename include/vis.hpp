#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <tuple>


/**
 * @brief Represents an Axis-Aligned Bounding Box (AABB) for the fixed workspace limits.
 * Kept simple for easy clamping.
 */
struct AABB
{
    Eigen::Vector3d min_limits;
    Eigen::Vector3d max_limits;
};


/**
 * @brief Represents an Oriented Bounding Box (OBB) in the world frame.
 * Used for all fixed and dynamic obstacles.
 */
struct OBB
{
    Eigen::Vector3d center;            // Center in world frame
    Eigen::Vector3d half_extents;      // Half size [hx, hy, hz]
    Eigen::Matrix3d rotation;          // Orientation matrix (World -> OBB Local)
};

/**
 * @brief Defines a Bounding Box relative to a link's local frame.
 */
struct LinkBoundingBox
{
    Eigen::Vector3d center_local;      // Center of the OBB in the link's frame
    Eigen::Vector3d half_extents;      // Half size [hx, hy, hz]
    std::string frame_name;            // Pinocchio frame name (e.g., "fr3_link3")
};


/**
 * @brief Handles visualization of the workspace and forbidden volumes using ROS 2 Markers.
 */
class WorkspaceVisualizer
{
public:
    WorkspaceVisualizer();

    void init( rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub, 
               AABB &workspace, 
               std::vector<OBB> &fixed_obstacles,
               std::vector<OBB> &dynamic_obstacles);

    void publish_markers();

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    
    AABB workspace_;
    
    // Updated to store OBBs for fixed obstacles
    std::vector<OBB> fixed_forbidden_obstacles_;
    
    // Pointer to dynamic obstacles, as they are updated outside the class.
    std::vector<OBB>* dynamic_forbidden_obstacles_ = nullptr; 

    int marker_id_counter_ = 0;
    
    const double PLANE_THICKNESS = 0.005;

    void publish_workspace_planes();
    void publish_workspace_edges();

    // New marker creation functions to handle OBBs and clear separation from AABBs
    visualization_msgs::msg::Marker create_obb_volume_marker(const OBB& obb, const std::string& ns);
    visualization_msgs::msg::Marker create_aabb_volume_marker(const AABB& aabb, const std::string& ns, int id_offset);
};
