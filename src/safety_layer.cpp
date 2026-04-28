#include "../include/franka_safety_layer/safety_layer.hpp"


SafetyLayer::SafetyLayer()
{
    // 1. Define the Allowed Workspace (AABB - remains AABB for simple clamping)
    workspace_ = {
        Eigen::Vector3d(-0.5, -0.9, 0.0), // min_limits [x_min, y_min, z_min]
        Eigen::Vector3d( 0.8,  0.9, 1.2)  // max_limits [x_max, y_max, z_max]
    };
    
    // 2. Define Forbidden Obstacles (Now all OBBs)    

    // Fixed Obstacles (AABBs converted to OBBs with Identity Rotation)
    forbidden_obstacles_.push_back(gen_fixed_obb(Eigen::Vector3d(-0.5, -0.2, 0.0), Eigen::Vector3d(-0.1, 0.2, 0.7))); // center cam
    forbidden_obstacles_.push_back(gen_fixed_obb(Eigen::Vector3d( 0.3, -0.9, 0.0), Eigen::Vector3d( 0.8, -0.65, 0.7))); // right cam
    forbidden_obstacles_.push_back(gen_fixed_obb(Eigen::Vector3d( 0.3, 0.65, 0.0), Eigen::Vector3d( 0.8, 0.9, 0.7))); // left cam 
    forbidden_obstacles_.push_back(gen_fixed_obb(Eigen::Vector3d( 0.5, -0.2, 0.0), Eigen::Vector3d( 0.8, 0.2, 0.6))); // drawer

    // --- NEW: Define Other Robot Link Boxes (Franka FR3 Example) ---
    // NOTE: These are relative to the link frames and must be tuned!
    other_robot_link_boxes_ = 
    {
        // Link13 (lower Arm) - Approximate as a capsule-like OBB
        { Eigen::Vector3d(0.0, 0.0, -0.15), Eigen::Vector3d(0.07, 0.07, 0.15), "fr3_link1" },
        // Link 3 (Upper Arm) - Approximate as a capsule-like OBB
        { Eigen::Vector3d(0.0, 0.0, -0.15), Eigen::Vector3d(0.07, 0.07, 0.15), "fr3_link3" },
        // Link 5 (Forearm)
        { Eigen::Vector3d(0.0, 0.0, -0.25), Eigen::Vector3d(0.06, 0.06, 0.25), "fr3_link5" },
        // Link 7 (Wrist)
        { Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.05, 0.05, 0.1), "fr3_link7" },
        // ... add more critical links as needed ...
    };
    current_other_robot_obstacles_.resize(other_robot_link_boxes_.size());


    effective_workspace_ = {
        workspace_.min_limits + Eigen::Vector3d::Constant(safety_distance_),
        workspace_.max_limits - Eigen::Vector3d::Constant(safety_distance_)
    };


    std::cout << "SafetyLayer initialized with " << forbidden_obstacles_.size() << " fixed OBBs, "
        << other_robot_link_boxes_.size() << " other_robot_link_boxes_, "
        << "and safety distance " << safety_distance_ << " m." << std::endl;
}


void SafetyLayer::init(Eigen::Vector3d &initial_position, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub)
{
    initial_position_ = initial_position;
    // NOTE: vis_.init would need to be updated to accept a list of OBBs
    vis_.init(marker_pub, workspace_, forbidden_obstacles_, current_other_robot_obstacles_); 
}


OBB SafetyLayer::gen_fixed_obb(const Eigen::Vector3d& minl, const Eigen::Vector3d& maxl) 
{
    OBB obb;
    obb.center = (minl + maxl) / 2.0;
    obb.half_extents = (maxl - minl) / 2.0;
    obb.rotation = Eigen::Matrix3d::Identity();
    return obb;
};


// ====================================================================================
// KINEMATICS AND TRANSFORMATION
// ====================================================================================

/**
 * @brief Performs Pinocchio FK and updates the world poses of all links.
 */
void SafetyLayer::updateOtherRobotKinematics()
{
    // Assuming 'other_q_' is updated by a ROS 2 callback (not shown here)
    pinocchio::forwardKinematics(other_model_, *other_data_, other_q_);
    pinocchio::updateFramePlacements(other_model_, *other_data_);
}

/**
 * @brief Transforms all local LinkBoundingBoxes into world-frame OBBs.
 */
void SafetyLayer::transformBoundingBoxes()
{
    // 1. Update Kinematics
    updateOtherRobotKinematics(); 

    // 2. Transform each bounding box
    for (size_t i = 0; i < other_robot_link_boxes_.size(); ++i)
    {
        const auto& local_box = other_robot_link_boxes_[i];
        OBB& world_box = current_other_robot_obstacles_[i];
        
        // Get the transformation (SE3) for the link's frame
        if (!other_model_.existFrame(other_prefix_+"_"+local_box.frame_name)) {
            std::cout << other_prefix_+"_"+local_box.frame_name << " not found" << std::endl;
            continue;
        }
        pinocchio::FrameIndex frame_id = other_model_.getFrameId(other_prefix_+"_"+local_box.frame_name);
        const pinocchio::SE3& link_pose_world = other_data_->oMf[frame_id];

        // The center of the OBB in the world frame
        world_box.center = link_pose_world.translation() + link_pose_world.rotation() * local_box.center_local;
        
        // The half extents remain constant (they define the size)
        world_box.half_extents = local_box.half_extents;
        
        // The rotation matrix from the OBB's local frame to the world frame
        world_box.rotation = link_pose_world.rotation();
    }
}


/**
 * @brief Finds the closest point on the surface of an OBB to a query point.
 * This is required for finding the shortest distance for safety checks.
 */
Eigen::Vector3d SafetyLayer::closestPointToOBB(const OBB& box, const Eigen::Vector3d& query_point) const
{
    // 1. Transform the query point into the OBB's local frame
    Eigen::Vector3d q_local = box.rotation.transpose() * (query_point - box.center);

    // 2. Clamp the local query point to the local AABB limits ([-h, +h])
    Eigen::Vector3d closest_local;
    for (int i = 0; i < 3; ++i)
    {
        closest_local[i] = std::max(-box.half_extents[i], std::min(box.half_extents[i], q_local[i]));
    }

    // 3. Transform the closest local point back to the world frame
    Eigen::Vector3d closest_world = box.rotation * closest_local + box.center;
    
    return closest_world;
}


// ====================================================================================
// CONSTRUCTOR AND INITIALIZATION
// ====================================================================================



// ====================================================================================
// CLAMPING AND PUSH-OFF (Core Logic)
// ====================================================================================

// NOTE: closestPointToAABB is no longer needed since we use closestPointToOBB

Eigen::Vector3d SafetyLayer::clampToAABB(const Eigen::Vector3d& position) const
{
    Eigen::Vector3d clamped_pos;
    for (int i = 0; i < 3; ++i)
    {
        // Clamping uses the effective_workspace_ (AABB)
        clamped_pos[i] = std::max(effective_workspace_.min_limits[i], std::min(effective_workspace_.max_limits[i], position[i]));
    }           
    return clamped_pos;
}


Eigen::Vector3d SafetyLayer::calculatePushOff(const Eigen::Vector3d& current_position, const Eigen::Vector3d& robot_position) const
{
    double max_required_push_magnitude = 0.0;
    Eigen::Vector3d best_push_direction = Eigen::Vector3d::Zero();

    // Vector from current position to robot
    Eigen::Vector3d position_to_robot = robot_position - current_position;
    
    Eigen::Vector3d normalized_dir = Eigen::Vector3d::Zero();
    if (position_to_robot.norm() > 1e-9) {
        normalized_dir = position_to_robot.normalized();
    } else {
        normalized_dir = Eigen::Vector3d(0.0, 0.0, 1.0); // Fallback: +Z
    }
    
    // Combine fixed and dynamic obstacles
    std::vector<const OBB*> all_obstacles;
    
    // Add fixed obstacles
    for (const auto& obb : forbidden_obstacles_) {
        all_obstacles.push_back(&obb);
    }
    // Add dynamic obstacles
    if(other_robot_check)
    {
        for (const auto& obb : current_other_robot_obstacles_) {
            all_obstacles.push_back(&obb);
        }
    }

    // Iterate over all obstacles (fixed AABB converted to OBBs, and dynamic OBBs)
    for (const auto* obb_ptr : all_obstacles)
    {
        const OBB& obb = *obb_ptr;
        
        // 1. Find the closest point on the OBB's surface
        Eigen::Vector3d closest_on_obb = closestPointToOBB(obb, current_position);

        // 2. Calculate vector from closest point to current position
        Eigen::Vector3d vector_from_obb = current_position - closest_on_obb;
        double distance_to_obb_surface = vector_from_obb.cwiseAbs().maxCoeff();
        
        // 3. Determine if the current position is inside the INFLATED OBB
        if (distance_to_obb_surface < safety_distance_ - 1e-5)
        {
            // The required magnitude to exit the safety boundary
            double required_magnitude_to_boundary = safety_distance_ - distance_to_obb_surface;

            // --- Critical Face Logic (Adapted to OBB Local Frame) ---

            // 4. Define the inflated OBB boundaries in its LOCAL frame
            const Eigen::Vector3d local_min_inflated = -obb.half_extents - Eigen::Vector3d::Constant(safety_distance_);
            const Eigen::Vector3d local_max_inflated =  obb.half_extents + Eigen::Vector3d::Constant(safety_distance_);

            // 5. Transform the necessary vectors to the OBB's LOCAL frame
            // OBB.rotation is R_world_to_local (or R_local_to_world, depending on your setup)
            // Assuming OBB.rotation is the matrix that rotates the local basis vectors INTO the world frame (R_local_to_world).
            // The inverse (transpose) transforms WORLD vectors INTO the LOCAL frame (R_world_to_local).
            Eigen::Matrix3d R_W_L = obb.rotation.transpose(); // World to Local Rotation

            // Transform current position and normalized direction into the local OBB frame
            Eigen::Vector3d current_position_local = R_W_L * (current_position - obb.center);
            Eigen::Vector3d normalized_dir_local = R_W_L * normalized_dir;

            double current_required_magnitude = std::numeric_limits<double>::infinity();
            double current_dist_by_norm_direction = std::numeric_limits<double>::infinity();
            Eigen::Vector3d local_push_direction = Eigen::Vector3d::Zero();
            
            // Find the critical face that position_to_robot 'wants' to cross (in local frame)
            for (int i = 0; i < 3; ++i)
            {
                if (std::abs(normalized_dir_local[i]) > 1e-6)
                {
                    double magnitude_i;
                    double dist_by_norm_direction;
                    
                    if (normalized_dir_local[i] > 0.0)
                    {
                        // Exiting via MAX-face. Push direction is +1 on this local axis.
                        magnitude_i = local_max_inflated[i] - current_position_local[i];
                        dist_by_norm_direction = std::abs(magnitude_i / normalized_dir_local[i]);
                        local_push_direction = Eigen::Vector3d::Zero();
                        local_push_direction[i] = 1.0; 
                    }
                    else // normalized_dir_local[i] < 0.0
                    {
                        // Exiting via MIN-face. Push direction is -1 on this local axis.
                        magnitude_i = current_position_local[i] - local_min_inflated[i];
                        dist_by_norm_direction = std::abs(magnitude_i / normalized_dir_local[i]);
                        local_push_direction = Eigen::Vector3d::Zero();
                        local_push_direction[i] = -1.0;
                    }

                    // Track the axis with the smallest distance (most critical face)
                    if (dist_by_norm_direction < current_dist_by_norm_direction)
                    {
                        current_dist_by_norm_direction = dist_by_norm_direction;
                        current_required_magnitude = magnitude_i;
                        best_push_direction = local_push_direction; // Temporarily storing local push direction
                    }
                }
            }
            
            // 6. Transform the push direction back to the world frame
            if (current_required_magnitude < std::numeric_limits<double>::infinity())
            {
                // Push magnitude is the calculated distance to the face, scaled by the push direction.
                Eigen::Vector3d final_push_direction_world = obb.rotation * best_push_direction;

                // The final magnitude required to exit the face (current_required_magnitude) must be 
                // compared to the max_required_push_magnitude.

                if (current_required_magnitude > max_required_push_magnitude)
                {
                    max_required_push_magnitude = current_required_magnitude;
                    best_push_direction = final_push_direction_world;
                }
            }
        }
    }

    // Return the push vector (direction * magnitude)
    return best_push_direction * max_required_push_magnitude;
}


pinocchio::SE3 SafetyLayer::adjustToSafePose(const pinocchio::SE3& robot_pose, const pinocchio::SE3& desired_pose) const
{
    // --- NEW: Update Dynamic Obstacles ---
    // Update the other robot's configuration and OBBs
    if(other_robot_check)
    {
        const_cast<SafetyLayer*>(this)->transformBoundingBoxes(); 
    }

    // 1. Extract desired position and orientation
    Eigen::Vector3d desired_position = desired_pose.translation();
    
    // initial clamp to not work unnessesarry with invalid positions.
    Eigen::Vector3d safe_position = clampToAABB(desired_position);

    std::cout << "init : " << safe_position.transpose() << std::endl;

    const int MAX_ITERATIONS = 20;
    Eigen::Vector3d total_push_vector = Eigen::Vector3d::Zero();
    int push_iterations = 0;

    // --- Safety Check 1: Forbidden Blocks Avoidance (Iterative) ---
    for (int iter = 0; iter < MAX_ITERATIONS; ++iter)
    {
        // This call checks against fixed (now OBB) and dynamic OBBs
        Eigen::Vector3d current_push_off = calculatePushOff(safe_position, robot_pose.translation());

        
        if (current_push_off.norm() < 1e-6)
        {
            // Position is now safe from all blocks
            push_iterations = iter;
            break; 
        }
        else
        {
            std::cout << "iter " << iter << ", pos : " << safe_position.transpose() << std::endl;
            std::cout << "iter " << iter << ", push : " << current_push_off.transpose() << std::endl;
            std::cout << "iter " << iter << ", pos : " << (safe_position+current_push_off).transpose() << std::endl;
        }

        safe_position += current_push_off;
        total_push_vector += current_push_off;
        
        if (iter == MAX_ITERATIONS - 1) {
            std::cerr << "Warning: Maximum safety layer iterations (" << MAX_ITERATIONS 
                      << ") reached. Position still unsafe from blocks." << std::endl;
            push_iterations = MAX_ITERATIONS;
            // TODO safe fallback
        }
    }

    if(push_iterations > 0) {
        std::cout << "final : " << safe_position.transpose() << std::endl;
        std::cout << "dir " << (robot_pose.translation()- safe_position).transpose() << std::endl;

    }

    std::cout << "----------- " << std::endl;

    // 2. Allowed Workspace Clamping done (again) at the end to ensure the final position is within bounds, 
    Eigen::Vector3d final_safe_position = clampToAABB(safe_position);

    // 4. Return the new SE3 pose with the original orientation and the safe position
    return pinocchio::SE3(desired_pose.rotation(), final_safe_position);
}


double SafetyLayer::getShortestDistanceToSafetyBoundary(const Eigen::Vector3d& query_position) const
{
    // A positive 's' means we are safe. A negative 's' means we have penetrated the safety zone.
    double min_distance_remaining = 10;
    
    // Combine fixed and dynamic obstacles
    std::vector<const OBB*> all_obstacles;
    for (const auto& obb : forbidden_obstacles_) {
        all_obstacles.push_back(&obb);
    }
    if(other_robot_check)
    {
        for (const auto& obb : current_other_robot_obstacles_) {
            all_obstacles.push_back(&obb);
        }
    }


    // 1. Check distance to all forbidden OBBs (Fixed and Dynamic)
    for (const auto* obb_ptr : all_obstacles)
    {
        const OBB& obb = *obb_ptr;
        
        // Find the closest point on the OBB surface
        Eigen::Vector3d closest_on_obb = closestPointToOBB(obb, query_position);
        
        // Actual distance to the OBB surface
        double distance_to_obb = (query_position - closest_on_obb).norm();

        // Distance remaining until the safety boundary (distance to OBB minus safety_distance_)
        double remaining_distance_s = distance_to_obb - safety_distance_;
        
        min_distance_remaining = std::min(min_distance_remaining, remaining_distance_s);
    }
    
    // 2. Consider the distance to the Allowed AABB boundary (Workspace limits)
    // NOTE: This remains AABB since the outer workspace boundary is fixed and world-aligned.
    Eigen::Vector3d effective_min = workspace_.min_limits + Eigen::Vector3d::Constant(safety_distance_);
    Eigen::Vector3d effective_max = workspace_.max_limits - Eigen::Vector3d::Constant(safety_distance_);

    // Check if position is outside the effective safe workspace
    if (query_position.x() < effective_min.x() || query_position.x() > effective_max.x() ||
        query_position.y() < effective_min.y() || query_position.y() > effective_max.y() ||
        query_position.z() < effective_min.z() || query_position.z() > effective_max.z())
    {
        // If outside or on the boundary, distance 's' is 0 or negative.
        min_distance_remaining = std::min(min_distance_remaining, 0.0);
    }
    else
    {
        // If inside the effective safe AABB, find the shortest distance to the boundary.
        double min_dist_to_workspace_face = std::numeric_limits<double>::infinity();
        for (int i = 0; i < 3; ++i)
        {
            // Distance to min effective face
            min_dist_to_workspace_face = std::min(min_dist_to_workspace_face, query_position[i] - effective_min[i]);
            
            // Distance to max effective face
            min_dist_to_workspace_face = std::min(min_dist_to_workspace_face, effective_max[i] - query_position[i]);
        }
        
        min_distance_remaining = std::min(min_distance_remaining, min_dist_to_workspace_face);
    }

    return min_distance_remaining;
}

// Ray-OBB intersection using slab method in OBB local space
// Returns +inf if no intersection
double SafetyLayer::intersectRayOBB(const Eigen::Vector3d& ray_origin, const Eigen::Vector3d& ray_dir, const OBB& obb) const
{
    // Transform ray into OBB-local coordinates
    // rotation is world->local rotation
    Eigen::Vector3d local_origin = obb.rotation * (ray_origin - obb.center);
    Eigen::Vector3d local_dir    = obb.rotation * ray_dir;

    const Eigen::Vector3d& h = obb.half_extents;

    double t_min = 0.0;
    double t_max = std::numeric_limits<double>::infinity();

    for (int i = 0; i < 3; ++i) {
        if (std::abs(local_dir[i]) < 1e-8) {
            // Ray parallel to slab: must be within slab
            if (local_origin[i] < -h[i] || local_origin[i] > h[i])
                return std::numeric_limits<double>::infinity();
        } else {
            double t1 = (-h[i] - local_origin[i]) / local_dir[i];
            double t2 = ( h[i] - local_origin[i]) / local_dir[i];

            if (t1 > t2) std::swap(t1, t2);

            t_min = std::max(t_min, t1);
            t_max = std::min(t_max, t2);

            if (t_min > t_max)
                return std::numeric_limits<double>::infinity();
        }
    }

    // Hit
    return (t_min >= 0.0 ? t_min : t_max);
}


double SafetyLayer::getDistanceAlongVelocity(const Eigen::Vector3d& query_position, const Eigen::Vector3d& query_velocity) const
{
    // WARNING: This function still uses the complex Ray-AABB slab method and is not 
    // compatible with OBBs. To be fully safe and consistent, the fixed AABBs should be 
    // used here, and Ray-OBB intersection would be required for the dynamic blocks, 
    // which is geometrically much more complex.

    // For now, we revert to checking ONLY the fixed AABBs using the slab method, 
    // and use the simpler distance measure for the dynamic blocks if velocity is non-zero.
    
    // Die kleinste Distanz entlang des Geschwindigkeitsvektors bis zum Auftreffen auf eine Grenze
    double min_distance_t = 10;

    double velocity_norm = query_velocity.norm();
    
    if (velocity_norm < 1e-9) {
        return getShortestDistanceToSafetyBoundary(query_position); 
    }
    
    // Der Richtungsvektor des Strahls (Einheitsvektor)
    const Eigen::Vector3d ray_direction = query_velocity / velocity_norm;
    
    // --- 1. Distance to Forbidden OBBs (now fully supporting rotation) ---
    for (const auto& obb : forbidden_obstacles_)
    {
        double t_hit = intersectRayOBB(query_position, ray_direction, obb);

        if (t_hit >= 0.0)
            min_distance_t = std::min(min_distance_t, t_hit);
    }
    
    // --- 2. Distanz zur äußeren Grenze (Workspace Limits - AABB) ---
    // ... (Existing Slab Method for Workspace remains here - unchanged) ...
    
    double workspace_t_min = std::numeric_limits<double>::lowest();
    double workspace_t_max = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 3; ++i)
    {
        if (std::abs(ray_direction[i]) < 1e-6)
        {
            if (query_position[i] < workspace_.min_limits[i] || query_position[i] > workspace_.max_limits[i])
            {
                workspace_t_max = 0.0;
                break; 
            }
        }
        else
        {
            double t1 = (workspace_.min_limits[i] - query_position[i]) / ray_direction[i];
            double t2 = (workspace_.max_limits[i] - query_position[i]) / ray_direction[i];

            if (t1 > t2) std::swap(t1, t2);

            workspace_t_min = std::max(workspace_t_min, t1);
            workspace_t_max = std::min(workspace_t_max, t2);
        }
    }

    if (workspace_t_min <= workspace_t_max && workspace_t_max >= 0.0)
    {
        double t_exit = workspace_t_max;
        if (workspace_t_min > 0.0) {
            t_exit = 0.0;
        }
        min_distance_t = std::min(min_distance_t, t_exit);
    }
    
    // --- 3. Dynamic Obstacles (Approximation for Velocity Limit) ---
    // If we can't do Ray-OBB, we must approximate the distance for the velocity limit.
    // The safest approximation is to use the shortest distance calculated by 
    // getShortestDistanceToSafetyBoundary, as the full collision time logic is not available.

    if (min_distance_t == 10) {
        // If the ray didn't hit any fixed obstacle or the boundary, 
        // we use the point distance to the closest OBB (fixed or dynamic).
        double point_distance_s = getShortestDistanceToSafetyBoundary(query_position);
        
        // This is a conservative fallback: It uses distance (s) instead of collision time (t).
        min_distance_t = std::max(0.0, point_distance_s);
    }


    // Der Rückgabewert ist die minimale positive Distanz $t$
    return min_distance_t;
}


double SafetyLayer::getMaxSafeVelocity(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity) 
{
    // The closest distance 's' is the remaining distance to the safety boundary.
    current_distance_to_obstacle = getShortestDistanceToSafetyBoundary(position);
    current_distance_to_obstacle_along_velocity_direction = getDistanceAlongVelocity(position, velocity);
   
    // safe_velocity = min(max_velocity, sqrt(2 * a * max(s, 0.0)))
    double safe_velocity = std::min(max_velocity_, min_velocity_ + std::sqrt(2 * safety_stopping_acceleration_ * std::max(current_distance_to_obstacle_along_velocity_direction, 0.0)));
    
    return safe_velocity;
}