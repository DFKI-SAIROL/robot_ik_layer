#include "../include/franka_safety_layer/vis.hpp"

// Assuming the OBB struct is defined in the header, e.g.:
/*
struct OBB
{
    Eigen::Vector3d center;
    Eigen::Vector3d half_extents;
    Eigen::Matrix3d rotation; 
};
*/

// Constant for the thickness of the workspace boundary planes
const double PLANE_THICKNESS = 0.005;


WorkspaceVisualizer::WorkspaceVisualizer() 
: marker_id_counter_(0) {} // Initialize counter

// ====================================================================================
// INITIALIZATION
// ====================================================================================

void WorkspaceVisualizer::init( rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub, 
                                AABB &workspace, std::vector<OBB> &fixed_obstacles,
                                std::vector<OBB> &dynamic_obstacles)
{
    marker_pub_ = marker_pub;
    workspace_ = workspace;
    
    // Store fixed obstacles
    fixed_forbidden_obstacles_ = fixed_obstacles; 
    
    // Store the reference to the dynamic obstacles (updated externally)
    dynamic_forbidden_obstacles_ = &dynamic_obstacles;
}

// ====================================================================================
// MARKER CREATION HELPERS
// ====================================================================================

/**
 * @brief Creates a CUBE marker for an Axis-Aligned Bounding Box (AABB).
 * Used for the workspace planes/edges which are always world-aligned.
 */
visualization_msgs::msg::Marker WorkspaceVisualizer::create_aabb_volume_marker(const AABB& aabb, const std::string& ns, int id_offset)
{
    visualization_msgs::msg::Marker marker;
    
    marker.header.frame_id = "base";
    marker.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();

    marker.id = marker_id_counter_ + id_offset; // Use offset for multiple markers in one call
    marker.ns = ns; 

    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    Eigen::Vector3d center = (aabb.min_limits + aabb.max_limits) / 2.0;
    Eigen::Vector3d scale = aabb.max_limits - aabb.min_limits;

    marker.pose.position.x = center.x();
    marker.pose.position.y = center.y();
    marker.pose.position.z = center.z();
    
    // Default AABB orientation (Identity)
    marker.pose.orientation.w = 1.0; 
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;

    marker.scale.x = scale.x();
    marker.scale.y = scale.y();
    marker.scale.z = scale.z();

    return marker;
}

/**
 * @brief Creates a CUBE marker for an Oriented Bounding Box (OBB).
 * Used for all forbidden obstacles (fixed and dynamic).
 */
visualization_msgs::msg::Marker WorkspaceVisualizer::create_obb_volume_marker(const OBB& obb, const std::string& ns)
{
    visualization_msgs::msg::Marker marker;
    
    marker.header.frame_id = "base";
    marker.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();

    marker.id = marker_id_counter_++;
    marker.ns = ns; 

    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 1. Position is the OBB center
    marker.pose.position.x = obb.center.x();
    marker.pose.position.y = obb.center.y();
    marker.pose.position.z = obb.center.z();

    // 2. Orientation (Rotation Matrix to Quaternion)
    Eigen::Quaterniond q(obb.rotation);
    marker.pose.orientation.w = q.w();
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();

    // 3. Scale is the OBB full extent (2 * half_extents)
    marker.scale.x = obb.half_extents.x() * 2.0;
    marker.scale.y = obb.half_extents.y() * 2.0;
    marker.scale.z = obb.half_extents.z() * 2.0;
    
    return marker;
}


// ====================================================================================
// WORKSPACE VISUALIZATION (Mostly Unchanged)
// ====================================================================================

void WorkspaceVisualizer::publish_workspace_planes()
{
    Eigen::Vector3d size = workspace_.max_limits - workspace_.min_limits;
    Eigen::Vector3d center = (workspace_.min_limits + workspace_.max_limits) / 2.0;

    std::vector<std::string> plane_namespaces = {
        "ws_plane_x_min", "ws_plane_x_max", 
        "ws_plane_y_min", "ws_plane_y_max", 
        "ws_plane_z_min", "ws_plane_z_max"
    };
    
    std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d>> plane_configs(6);

    // [0] X_min plane
    plane_configs[0] = {
        (Eigen::Vector3d() << PLANE_THICKNESS, size.y(), size.z()).finished(),
        (Eigen::Vector3d() << workspace_.min_limits.x() + PLANE_THICKNESS / 2.0, center.y(), center.z()).finished()
    };
    
    // [1] X_max plane
    plane_configs[1] = {
        (Eigen::Vector3d() << PLANE_THICKNESS, size.y(), size.z()).finished(),
        (Eigen::Vector3d() << workspace_.max_limits.x() - PLANE_THICKNESS / 2.0, center.y(), center.z()).finished()
    };

    // [2] Y_min plane
    plane_configs[2] = {
        (Eigen::Vector3d() << size.x(), PLANE_THICKNESS, size.z()).finished(),
        (Eigen::Vector3d() << center.x(), workspace_.min_limits.y() + PLANE_THICKNESS / 2.0, center.z()).finished()
    };

    // [3] Y_max plane
    plane_configs[3] = {
        (Eigen::Vector3d() << size.x(), PLANE_THICKNESS, size.z()).finished(),
        (Eigen::Vector3d() << center.x(), workspace_.max_limits.y() - PLANE_THICKNESS / 2.0, center.z()).finished()
    };

    // [4] Z_min plane
    plane_configs[4] = {
        (Eigen::Vector3d() << size.x(), size.y(), PLANE_THICKNESS).finished(),
        (Eigen::Vector3d() << center.x(), center.y(), workspace_.min_limits.z() + PLANE_THICKNESS / 2.0).finished()
    };

    // [5] Z_max plane
    plane_configs[5] = {
        (Eigen::Vector3d() << size.x(), size.y(), PLANE_THICKNESS).finished(),
        (Eigen::Vector3d() << center.x(), center.y(), workspace_.max_limits.z() - PLANE_THICKNESS / 2.0).finished()
    };

    for (size_t i = 0; i < plane_configs.size(); ++i)
    {
        visualization_msgs::msg::Marker marker;
        
        marker.header.frame_id = "base";
        marker.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
        marker.id = marker_id_counter_++;
        marker.ns = plane_namespaces[i];
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        const auto& scale = std::get<0>(plane_configs[i]);
        const auto& pos = std::get<1>(plane_configs[i]);

        marker.scale.x = scale.x();
        marker.scale.y = scale.y();
        marker.scale.z = scale.z();

        marker.pose.position.x = pos.x();
        marker.pose.position.y = pos.y();
        marker.pose.position.z = pos.z();
        marker.pose.orientation.w = 1.0; 

        marker.color.r = 0;
        marker.color.g = 0.8;
        marker.color.b = 0.8;
        marker.color.a = 0.5;

        marker_pub_->publish(marker);
    }
}

void WorkspaceVisualizer::publish_workspace_edges()
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base";
    marker.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();

    marker.id = marker_id_counter_++;
    marker.ns = "ws_edges_wireframe";

    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.orientation.w = 1.0;

    const double LINE_WIDTH = 0.02;
    marker.scale.x = LINE_WIDTH;

    marker.color.r = 0;
    marker.color.g = 0.5;
    marker.color.b = 0;
    marker.color.a = 1.0;

    auto add_line = [&](double x1, double y1, double z1, double x2, double y2, double z2) {
        geometry_msgs::msg::Point p1, p2;
        p1.x = x1; p1.y = y1; p1.z = z1;
        p2.x = x2; p2.y = y2; p2.z = z2;
        marker.points.push_back(p1);
        marker.points.push_back(p2);
    };

    const double& min_x = workspace_.min_limits.x();
    const double& min_y = workspace_.min_limits.y();
    const double& min_z = workspace_.min_limits.z();
    const double& max_x = workspace_.max_limits.x();
    const double& max_y = workspace_.max_limits.y();
    const double& max_z = workspace_.max_limits.z();

    add_line(min_x, min_y, min_z, max_x, min_y, min_z);
    add_line(min_x, max_y, min_z, max_x, max_y, min_z);
    add_line(min_x, min_y, max_z, max_x, min_y, max_z);
    add_line(min_x, max_y, max_z, max_x, max_y, max_z);

    add_line(min_x, min_y, min_z, min_x, max_y, min_z);
    add_line(max_x, min_y, min_z, max_x, max_y, min_z);
    add_line(min_x, min_y, max_z, min_x, max_y, max_z);
    add_line(max_x, min_y, max_z, max_x, max_y, max_z);

    add_line(min_x, min_y, min_z, min_x, min_y, max_z);
    add_line(max_x, min_y, min_z, max_x, min_y, max_z);
    add_line(min_x, max_y, min_z, min_x, max_y, max_z);
    add_line(max_x, max_y, min_z, max_x, max_y, max_z);

    marker_pub_->publish(marker);
}


// ====================================================================================
// MAIN PUBLISH FUNCTION (Updated)
// ====================================================================================

// periodically (1s) called from timer in franks_ijk
void WorkspaceVisualizer::publish_markers()
{
    marker_id_counter_ = 0; 
    
    // publish_workspace_planes(); // Optional but uses marker_id_counter_
    publish_workspace_edges(); // Uses marker_id_counter_

    // 1. Visualize Fixed Obstacles (Red, 50% opacity)
    for (const auto& obb : fixed_forbidden_obstacles_)
    {
        auto marker = create_obb_volume_marker(obb, "fixed_forbidden_volume");
        marker.color.r = 0.8;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.5; // Red/Transparent
        marker_pub_->publish(marker);
    }
    
    // 2. Visualize Dynamic Obstacles (Yellow/Orange, 70% opacity)
    if (dynamic_forbidden_obstacles_)
    {
        for (const auto& obb : *dynamic_forbidden_obstacles_)
        {
            auto marker = create_obb_volume_marker(obb, "dynamic_forbidden_volume");
            marker.color.r = 1.0;
            marker.color.g = 0.6;
            marker.color.b = 0.0;
            marker.color.a = 0.7; // Orange/Semi-solid
            marker_pub_->publish(marker);
        }
    }
}