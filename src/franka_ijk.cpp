#include "../include/franka_safety_layer/franka_ijk.hpp"

Franka_IJK::Franka_IJK() : Node("franka_ijk") 
{

  // Setup Publisher and Subscriber
  target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("target_pose", 1, std::bind(&Franka_IJK::targetPoseCallback, this, std::placeholders::_1));
  target_dq_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("target_dq", 1, std::bind(&Franka_IJK::targetDQCallback, this, std::placeholders::_1));
  joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 1, std::bind(&Franka_IJK::jointStateCallback, this, std::placeholders::_1));
        
  target_joint_pub = this->create_publisher<sensor_msgs::msg::JointState>("target_joint", 1);
  debug_pub_ = this->create_publisher<franka_custom_msgs::msg::FIJKDebug>("fijk_debug", 1);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("safety_vis", 10);
  safe_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("safe_target_pose", 1);

  // TF Setup (for current pose access)
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // Parameters for arm prefix
  this->declare_parameter("arm_prefix", "franka_undefined");
  arm_prefix_ = this->get_parameter("arm_prefix").as_string();
  if(arm_prefix_ != "") 
  {
    arm_prefix_ += "_";
  }

  std::string other_ns;
  if(arm_prefix_ == "franka_left_") {
    other_ns = "franka_right";
  }
  else if(arm_prefix_ == "franka_right_") {
    other_ns = "franka_left";
  }
  else {
    other_ns = "franka_undefined";
  }
  safety_layer_.other_prefix_ = other_ns;

  std::cout << arm_prefix_ << " " << other_ns << std::endl;

  other_joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("/"+other_ns+"/joint_states", 1, std::bind(&Franka_IJK::otherJointStateCallback, this, std::placeholders::_1));

  // Parameters for Initial Position
  this->declare_parameter("init_joint_position", std::vector<double>(7, 0.0));
  std::vector<double> init_joint_position_vec = this->get_parameter("init_joint_position").as_double_array();

  this->declare_parameter("bypass_safety", false);
  bypass_safety_ = this->get_parameter("bypass_safety").as_bool();
  if (init_joint_position_vec.size() != 7 || std::all_of(init_joint_position_vec.begin(), init_joint_position_vec.end(), [](double x){ return std::abs(x) < 1e-6; }))
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid init joint positions (wrong size or all zeros). Shutting down.");
    rclcpp::shutdown();
    return;
  }
  q_init_ = Eigen::Map<Eigen::VectorXd>(init_joint_position_vec.data(), 7);
  RCLCPP_INFO(this->get_logger(), "Initial joint position loaded for nullspace control.");

  // Parameter declarations for Tuneable Variables
  this->declare_parameter("velocity_gain", 5.0);
  this->declare_parameter("joint_velocity_limit", 1.0);
  this->declare_parameter("cartesian_velocity_limit", 1.0);
  this->declare_parameter("K_NULL", 0.1);
  this->declare_parameter("dls_lambda", 0.0001);
  this->declare_parameter("max_spring_stretch", 0.1);
  this->declare_parameter("use_target_interpolation", true);
  this->declare_parameter("target_dt", 0.02);
  this->declare_parameter("timer_dt", 0.005);

  velocity_gain_ = this->get_parameter("velocity_gain").as_double();
  joint_velocity_limit_ = this->get_parameter("joint_velocity_limit").as_double();
  cartesian_velocity_limit_ = this->get_parameter("cartesian_velocity_limit").as_double();
  K_NULL_ = this->get_parameter("K_NULL").as_double();
  dls_lambda_ = this->get_parameter("dls_lambda").as_double();
  max_spring_stretch_ = this->get_parameter("max_spring_stretch").as_double();
  use_target_interpolation_ = this->get_parameter("use_target_interpolation").as_bool();
  target_dt_ = this->get_parameter("target_dt").as_double();
  timer_dt_ = this->get_parameter("timer_dt").as_double();

  RCLCPP_INFO(this->get_logger(), "Loaded tuneable parameters: velocity_gain=%f, joint_velocity_limit=%f, cartesian_velocity_limit=%f, K_NULL=%f, dls_lambda=%f, max_spring_stretch=%f, use_target_interpolation=%d, target_dt=%f, timer_dt=%f",
              velocity_gain_, joint_velocity_limit_, cartesian_velocity_limit_, K_NULL_, dls_lambda_, max_spring_stretch_, use_target_interpolation_, target_dt_, timer_dt_);

  param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&Franka_IJK::parametersCallback, this, std::placeholders::_1));

  // Load Pinocchio Model
  if (!loadPinocchioModel()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to load Pinocchio model. Shutting down.");
    rclcpp::shutdown(); 
    return;
  }

  // Load Pinocchio Model
  if(safety_layer_.other_robot_check && !bypass_safety_)
  {
    if (!loadOtherPinocchioModel(other_ns)) {
      RCLCPP_FATAL(this->get_logger(), "Failed to load other Pinocchio model. Shutting down.");
      rclcpp::shutdown(); 
      return;
    }
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Not loading other Pinocchio model.");
  }
  
  if (!bypass_safety_) {
    auto init_cartesian = computeForwardKinematic(q_init_).translation();
    safety_layer_.init(init_cartesian, marker_pub_);
  }

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(timer_dt_), std::bind(&Franka_IJK::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "franka_ijk initialized.");
}


bool Franka_IJK::loadPinocchioModel()
{
  // Create parameter client to get robot_description
  auto param_client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");

  RCLCPP_INFO(this->get_logger(), "Waiting for robot_state_publisher parameter server...");
  while (!param_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for parameter service.");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for robot_state_publisher to be available...");
  }

  // Wait until the parameter exists
  while (!param_client->has_parameter("robot_description")) {
    RCLCPP_INFO(this->get_logger(), "Waiting for 'robot_description' parameter...");
    rclcpp::sleep_for(500ms);
  }

  std::string urdf_string = param_client->get_parameter<std::string>("robot_description");
  if (urdf_string.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Received empty robot_description from robot_state_publisher.");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Successfully fetched robot_description parameter from robot_state_publisher");

  // Load Pinocchio model from URDF string
  try {
    pinocchio::urdf::buildModelFromXML(urdf_string, model_);
    data_ = std::make_unique<pinocchio::Data>(model_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load URDF from parameter: %s", e.what());
    return false;
  }

  std::string end_effector_link_ = arm_prefix_ + "rh_p12_rn_grasp_point";

  if (!model_.existFrame(end_effector_link_)) {
    RCLCPP_ERROR(this->get_logger(), "End effector link '%s' not found in model.", end_effector_link_.c_str());
    return false;
  }

  ee_frame_id_ = model_.getFrameId(end_effector_link_);
  return true;
}


void Franka_IJK::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  RCLCPP_INFO_ONCE(this->get_logger(), "Got joint state");
  if (q_.size() != model_.nv) {
    q_ = Eigen::VectorXd::Zero(model_.nv);
  }
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (model_.existJointName(msg->name[i])) {
      int idx_q = model_.joints[model_.getJointId(msg->name[i])].idx_q();
      if (idx_q >= 0 && idx_q < model_.nq) {
        q_(idx_q) = msg->position[i];
      }
    }
  }
}


void Franka_IJK::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  try {
    tf_buffer_->transform(*msg, target_pose_stamped_, target_frame_);
  } 
  catch (tf2::TransformException & ex) 
  {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Could not transform target pose from '%s' to '%s': %s",
                  msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
    return; 
  }

  target_se3_.translation() << target_pose_stamped_.pose.position.x, target_pose_stamped_.pose.position.y, target_pose_stamped_.pose.position.z;

  Eigen::Quaterniond q_rot( target_pose_stamped_.pose.orientation.w, 
                            target_pose_stamped_.pose.orientation.x, 
                            target_pose_stamped_.pose.orientation.y, 
                            target_pose_stamped_.pose.orientation.z);
  target_se3_.rotation() = q_rot.toRotationMatrix();

  RCLCPP_INFO_ONCE(this->get_logger(), "Got and transformed target state");
  
  // As soon as a cartesian target is received, we switch mode to cartesian control.
  if (is_policy_active_) {
    RCLCPP_INFO(this->get_logger(), "Cartesian target received. Preempting policy control.");
    is_policy_active_ = false;
  }
}


void Franka_IJK::targetDQCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // 1. Ensure we have enough data (Franka needs at least 7 joints)
  if (msg->position.size() < 7) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
        "Policy action too small! Got %zu, expected at least 7", msg->position.size());
    return;
  }

  // 2. Claim control and reset watchdog
  if (!is_policy_active_) {
      RCLCPP_INFO(this->get_logger(), "Engaging policy control.");
      is_policy_active_ = true;
  }
  last_policy_msg_time_ = this->get_clock()->now();

  target_dq = Eigen::VectorXd::Zero(model_.nv);
  
  // 3. MAP THE ARRAY TO THE COMMAND VECTOR
  if (msg->name.empty()) {
      // Fallback: Python sent no names. Map by index directly to the first 7 joints.
      // (This assumes the network output perfectly matches the URDF joint order).
      for (int i = 0; i < 7; ++i) {
          target_dq[i] = msg->position[i];
      }
  } 
  else {
      // Safe Mapping: Python sent names. Map them dynamically to Pinocchio's velocity vector.
      for (size_t i = 0; i < msg->name.size(); ++i) {
        if (model_.existJointName(msg->name[i])) {
          int idx_v = model_.joints[model_.getJointId(msg->name[i])].idx_v();
          if (idx_v >= 0 && idx_v < model_.nv) {
            target_dq[idx_v] = msg->position[i];
          }
        }
      }
  }

  // 4. Apply Velocity Limits Safely
  double max_dq = target_dq.array().abs().maxCoeff();
  
  // Ensure joint_velocity_limit_ is > 0 so we don't divide by zero or mute actions
  if (max_dq > joint_velocity_limit_ && joint_velocity_limit_ > 0.0) {
    target_dq *= (joint_velocity_limit_ / max_dq);
  }
}

// other robot
void Franka_IJK::otherJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  RCLCPP_INFO_ONCE(this->get_logger(), "Other joint cb");
  if (!bypass_safety_) {
    if (safety_layer_.other_q_.size() != safety_layer_.other_model_.nv) {
      safety_layer_.other_q_ = Eigen::VectorXd::Zero(safety_layer_.other_model_.nv);
    }
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (safety_layer_.other_model_.existJointName(msg->name[i])) {
        int idx_q = safety_layer_.other_model_.joints[safety_layer_.other_model_.getJointId(msg->name[i])].idx_q();
        if (idx_q >= 0 && idx_q < safety_layer_.other_model_.nq) {
          safety_layer_.other_q_(idx_q) = msg->position[i];
        }
      }
    }
  }
}


bool Franka_IJK::loadOtherPinocchioModel(std::string other_ns)
{
  // Create parameter client to get robot_description
  auto param_client = std::make_shared<rclcpp::SyncParametersClient>(this, "/"+other_ns+"/robot_state_publisher");

  RCLCPP_INFO(this->get_logger(), "Waiting for other robot_state_publisher parameter server...");
  while (!param_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for parameter service.");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for other robot_state_publisher to be available...");
  }

  // Wait until the parameter exists
  while (!param_client->has_parameter("robot_description")) {
    RCLCPP_INFO(this->get_logger(), "Waiting for other 'robot_description' parameter...");
    rclcpp::sleep_for(500ms);
  }

  std::string urdf_string = param_client->get_parameter<std::string>("robot_description");
  if (urdf_string.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Received empty robot_description from other robot_state_publisher.");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Successfully fetched other robot_description parameter from other robot_state_publisher");

  // Load Pinocchio model from URDF string
  try {
    pinocchio::urdf::buildModelFromXML(urdf_string, safety_layer_.other_model_);
    safety_layer_.other_data_ = std::make_unique<pinocchio::Data>(safety_layer_.other_model_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load URDF from parameter: %s", e.what());
    return false;
  }

  std::string other_end_effector_link_ = other_ns + "_fr3_link8";

  if (!safety_layer_.other_model_.existFrame(other_end_effector_link_)) {
    RCLCPP_ERROR(this->get_logger(), "End effector link '%s' not found in model.", other_end_effector_link_.c_str());
    return false;
  }

  safety_layer_.other_ee_frame_id_ = safety_layer_.other_model_.getFrameId(other_end_effector_link_);
  return true;
}


bool Franka_IJK::tfLookup(std::string frame_from, std::string frame_to, pinocchio::SE3 &result)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_->lookupTransform(
      frame_from, frame_to, tf2::TimePointZero, std::chrono::milliseconds(100));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Could not transform: %s", ex.what());
    return false;
  }

  result.translation() << transform_stamped.transform.translation.x,
                                transform_stamped.transform.translation.y,
                                transform_stamped.transform.translation.z;

  Eigen::Quaterniond q_rot(transform_stamped.transform.rotation.w,
                            transform_stamped.transform.rotation.x,
                            transform_stamped.transform.rotation.y,
                            transform_stamped.transform.rotation.z);
  result.rotation() = q_rot.toRotationMatrix();

  return true;
}


geometry_msgs::msg::Pose Franka_IJK::convert(pinocchio::SE3 se3)
{
  geometry_msgs::msg::Pose pose_msg;

  pose_msg.position.x = se3.translation()(0);
  pose_msg.position.y = se3.translation()(1);
  pose_msg.position.z = se3.translation()(2);

  Eigen::Quaterniond q(se3.rotation());
  pose_msg.orientation.x = q.x();
  pose_msg.orientation.y = q.y();
  pose_msg.orientation.z = q.z();
  pose_msg.orientation.w = q.w();

  return pose_msg;
}


geometry_msgs::msg::Twist Franka_IJK::convert(Eigen::VectorXd v)
{
  geometry_msgs::msg::Twist twist_msg;

  if (v.size() != 6) {
    return twist_msg;
  }

  twist_msg.linear.x = v(0);
  twist_msg.linear.y = v(1);
  twist_msg.linear.z = v(2);

  twist_msg.angular.x = v(3);
  twist_msg.angular.y = v(4);
  twist_msg.angular.z = v(5);

  return twist_msg;
}


pinocchio::SE3 Franka_IJK::computeForwardKinematic(Eigen::VectorXd q)
{  
  pinocchio::forwardKinematics(model_, *data_, q);
  pinocchio::updateFramePlacements(model_, *data_);
  return data_->oMf[ee_frame_id_]; 
}


double Franka_IJK::computeCartesianVelocity(
  const pinocchio::SE3 &current_se3, const pinocchio::SE3 &target_se3, Eigen::VectorXd &desired_cartesian_velocity)
{

  // Linear: Compute Error in World Frame
  Eigen::Vector3d lin_world_error = target_se3.translation() - current_se3.translation();
  
  // Angular 
  // 1. Calculate the relative rotation matrix R_err = R_current^-1 * R_target
  pinocchio::SE3 relative_pose = current_se3.inverse() * target_se3;
  Eigen::Matrix3d R_error = relative_pose.rotation();
  Eigen::Vector3d angular_error_local = pinocchio::log3(R_error);

  // 2. Rotate the Angular Error Vector from the Current End-Effector Frame to the World Frame.
  Eigen::Matrix3d R_current = current_se3.rotation();
  Eigen::Vector3d angular_world_error = R_current * angular_error_local;
  
  // Build the 6D desired velocity V = K * e
  desired_cartesian_velocity = Eigen::VectorXd::Zero(6);
  desired_cartesian_velocity.head<3>() = velocity_gain_ * lin_world_error; // / dt_;
  desired_cartesian_velocity.tail<3>() = velocity_gain_ * angular_world_error; // / dt_;

  double target_reachable_factor = 1;

  // Limit velocity
  if (desired_cartesian_velocity.norm() > cartesian_velocity_limit_) 
  {
    double reduction = cartesian_velocity_limit_ / desired_cartesian_velocity.norm();
    target_reachable_factor = 1 / reduction;
    desired_cartesian_velocity *= reduction;
  }

  if (!bypass_safety_)
  {
    double max_safe_v = safety_layer_.getMaxSafeVelocity(current_se3.translation(), desired_cartesian_velocity);
    if (desired_cartesian_velocity.norm() > max_safe_v)
    {
      double reduction = max_safe_v / desired_cartesian_velocity.norm();
      target_reachable_factor = 1;
      desired_cartesian_velocity *= reduction;
    } 
  }

  // Apply tolerance to stop movement near target
  if (desired_cartesian_velocity.norm() < 1e-3) {
    desired_cartesian_velocity.setZero();
  }
  
  return target_reachable_factor;
}


Eigen::VectorXd Franka_IJK::runJacobianNullspaceControl(const Eigen::VectorXd& desired_cartesian_velocity)
{

  // 1. Update Kinematics (needed for Jacobian calculation)
  pinocchio::computeAllTerms(model_, *data_, q_cmd_, Eigen::VectorXd::Zero(model_.nv));

  // 2. Compute the Jacobian matrix (6xN, N=DOF)
  Eigen::MatrixXd J(6, model_.nv);
  J.setZero();
  pinocchio::getFrameJacobian(model_, *data_, ee_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J);

  // 3. Compute the Damped Least Squares Pseudo-Inverse (J_dagger)
  const double lambda = dls_lambda_; // Damping factor for DLS
  Eigen::MatrixXd J_dagger = J.transpose() * (J * J.transpose() + lambda * Eigen::MatrixXd::Identity(6, 6)).inverse();

  // 4. Primary Task: Cartesian Velocity
  Eigen::VectorXd dq_prim = J_dagger * desired_cartesian_velocity;

  // Check for inaccuracies in inverse kinematics
  Eigen::VectorXd achieved_cartesian_velocity = J * dq_prim;

  // 5. Secondary Task: Nullspace Posture Control
  // 5.2. Compute the Nullspace Projector: N = I - J_dagger * J
  Eigen::MatrixXd N = Eigen::MatrixXd::Identity(model_.nv, model_.nv) - J_dagger * J;

  // 5.3. Secondary Task Velocity (Posture Control): dq_null_task = K_NULL_ * (q_init - q_curr)
  Eigen::VectorXd joint_error_posture = q_init_ - q_cmd_;
  Eigen::VectorXd dq_null_task = K_NULL_ * joint_error_posture;

  // 5.4. Projected Nullspace Command: dq_null = N * dq_null_task
  Eigen::VectorXd dq_null = N * dq_null_task;

  // 5.6. Combine Commands: dq = dq_prim + dq_null
  Eigen::VectorXd dq = dq_prim + dq_null;

  return dq;
}

void Franka_IJK::controlLoop()
{
  // 1. Safety Checks: Do we have valid joint states yet?
  if (q_.size() == 0 || !q_.allFinite()) {
    return;
  }

  if (q_cmd_.size() == 0) {
    q_cmd_ = q_;
  }

  // 2. Time management
  auto current_time = this->get_clock()->now();
  if (last_time_.nanoseconds() == 0) {
    last_time_ = current_time;
    return; // Skip the first tick to get a clean dt_ next time
  }

  dt_ = (current_time - last_time_).seconds();
  last_time_ = current_time;

  // Clamp dt_ to prevent math explosions if the node stalls
  if (dt_ <= 0.0 || dt_ > target_dt_ * 2) {
    dt_ = target_dt_; // Fallback to 200Hz
  }

  if (is_policy_active_) {
    // Timeout: No policy msg in 250ms -> fall back to Cartesian hold
    if ((current_time - last_policy_msg_time_).seconds() > 0.25) {
      RCLCPP_WARN(this->get_logger(), "Policy stream stopped. Reverting to Cartesian hold.");
      is_policy_active_ = false;
      target_se3_ = computeForwardKinematic(q_cmd_); 
    } else {
      // POLICY IS HEALTHY: Execute the stored network velocity at 200Hz!
      publishCommand(target_dq);
      
      // (Optional) Publish your debug info here to record the policy execution
      // publishDebugInfos(...);
      
      return; // Skip the Cartesian IK math
    }
  }
  
  // 3. Check if we have received a target yet
  if (target_pose_stamped_.header.stamp.sec == 0)
  {
    q_cmd_ = q_;

    // Keeping your original hold-position logic
    pinocchio::SE3 current_se3 = computeForwardKinematic(q_cmd_);
    Eigen::VectorXd desired_cartesian_velocity;
    computeCartesianVelocity(current_se3, current_se3, desired_cartesian_velocity);
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(model_.nv);

    pinocchio::SE3 tf_se3 = pinocchio::SE3::Identity();
    tfLookup(target_frame_, arm_prefix_ + "rh_p12_rn_grasp_point", tf_se3);

    publishDebugInfos(current_se3, current_se3, current_se3, tf_se3, desired_cartesian_velocity, dq);
    if (!bypass_safety_) {
        safety_layer_.vis_.publish_markers();
    }
    return;
  }

  // --- Normal Execution ---
  pinocchio::SE3 current_se3 = computeForwardKinematic(q_cmd_);

  pinocchio::SE3 tf_se3 = pinocchio::SE3::Identity();
  tfLookup(target_frame_, arm_prefix_ + "rh_p12_rn_grasp_point", tf_se3);

  pinocchio::SE3 active_target_se3 = getInterpolatedTarget(target_se3_, dt_);

  pinocchio::SE3 safe_target_se3;
  if (bypass_safety_)
  {
      safe_target_se3 = active_target_se3;
  }
  else
  {
      safe_target_se3 = safety_layer_.adjustToSafePose(current_se3, active_target_se3);
  }

  // --- Compute Primary Task (Cartesian Error and Velocity) ---
  Eigen::VectorXd desired_cartesian_velocity;
  double target_reachable_factor = computeCartesianVelocity(current_se3, safe_target_se3, desired_cartesian_velocity);
  
  Eigen::VectorXd dq = runJacobianNullspaceControl(desired_cartesian_velocity);

  // --- Velocity Limiting ---
  double max_dq = dq.array().abs().maxCoeff();
  if (max_dq > joint_velocity_limit_) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Joint velocity Limit Active");
    dq *= (joint_velocity_limit_ / max_dq);
  }

  // --- Publish Command ---
  publishCommand(dq);

  publishDebugInfos(current_se3, target_se3_, safe_target_se3, tf_se3, desired_cartesian_velocity, dq);

  if (!bypass_safety_) {
      safety_layer_.vis_.publish_markers();
  }
}

void Franka_IJK::publishCommand(const Eigen::VectorXd& dq)
{
  sensor_msgs::msg::JointState target_joint;
  
  target_joint.header.stamp = this->get_clock()->now();
  
  std::vector<std::string> target_names;
  for (int i = 1; i <= 7; ++i) {
    target_names.push_back(arm_prefix_ + "fr3_joint" + std::to_string(i));
  }
  target_joint.name = target_names;
  target_joint.position.resize(7);
  target_joint.velocity.resize(7);

  for (int i = 0; i < model_.nv; ++i) {
    q_cmd_[i] += dq[i] * dt_;
    // WARNING: Do NOT clamp q_cmd_ to q_ here. 
    // Muting the offset between desired and actual strictly prevents the 
    // impedance controller from building up spring force, leaving the arm stuck 
    // and causing algebraic loops (jiggeling) from sensor noise.
    // 2. The Leash: Calculate how far the virtual arm has pulled ahead
    double tracking_error = q_cmd_[i] - q_[i];
    double max_spring_stretch = max_spring_stretch_; // max allowed wind-up
    
    // 3. Prevent dangerous drift
    if (tracking_error > max_spring_stretch) {
        q_cmd_[i] = q_[i] + max_spring_stretch; // Cap the forward pull
    } else if (tracking_error < -max_spring_stretch) {
        q_cmd_[i] = q_[i] - max_spring_stretch; // Cap the backward pull
    }
  }

  for (int i = 0; i < 7; ++i) {
    if (model_.existJointName(target_names[i])) {
      int idx_q = model_.joints[model_.getJointId(target_names[i])].idx_q();
      target_joint.position[i] = q_cmd_[idx_q];
      
      // If velocity is enabled:
      int idx_v = model_.joints[model_.getJointId(target_names[i])].idx_v();
      target_joint.velocity[i] = dq[idx_v];
    }
  }

  target_joint_pub->publish(target_joint);
}


void Franka_IJK::publishDebugInfos(pinocchio::SE3 &current_se3, pinocchio::SE3 &target_se3, pinocchio::SE3 &safe_target_se3, pinocchio::SE3 &tf_se3, Eigen::VectorXd &desired_cartesian_velocity, Eigen::VectorXd &dq)
{
  franka_custom_msgs::msg::FIJKDebug debug_msg;
  debug_msg.header.stamp = this->get_clock()->now();
  debug_msg.header.frame_id = target_frame_;

  debug_msg.actual_pose = convert(computeForwardKinematic(q_));
  debug_msg.target_pose = convert(target_se3);
  debug_msg.safe_target_pose = convert(safe_target_se3);
  debug_msg.tf_pose = convert(tf_se3);
  debug_msg.cmd_pose = convert(computeForwardKinematic(q_cmd_));
  debug_msg.cmd_final_pose = convert(computeForwardKinematic(q_cmd_));
  debug_msg.cartesian_velocity = convert(desired_cartesian_velocity);

  debug_msg.safety_distance = safety_layer_.current_distance_to_obstacle;
  debug_msg.safety_distance_along_velocity = safety_layer_.current_distance_to_obstacle_along_velocity_direction;

  debug_msg.q_actual.resize(7);
  debug_msg.dq_commanded.resize(7);
  debug_msg.q_target.resize(7);

  for (int i = 0; i < 7; ++i) {
    // We need to map the 7 robot joints correctly from the Pinocchio model
    std::string joint_name = arm_prefix_ + "fr3_joint" + std::to_string(i + 1);
    
    if (model_.existJointName(joint_name)) {
      int idx_q = model_.joints[model_.getJointId(joint_name)].idx_q();
      int idx_v = model_.joints[model_.getJointId(joint_name)].idx_v();

      // 1. The State: Exactly where the robot is physically right now
      debug_msg.q_actual[i] = q_[idx_q];

      // 2. The Velocity Action: The limited velocity the expert commanded
      debug_msg.dq_commanded[i] = dq[idx_v];

      // 3. The Position Action: The final leashed virtual target sent to the controller
      debug_msg.q_target[i] = q_cmd_[idx_q]; 
    }
  }

  debug_pub_->publish(std::move(debug_msg));
}

rcl_interfaces::msg::SetParametersResult Franka_IJK::parametersCallback(
  const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto &param : parameters)
  {
    if (param.get_name() == "velocity_gain" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      velocity_gain_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated velocity_gain to %f", velocity_gain_);
    } else if (param.get_name() == "joint_velocity_limit" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      joint_velocity_limit_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated joint_velocity_limit to %f", joint_velocity_limit_);
    } else if (param.get_name() == "cartesian_velocity_limit" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      cartesian_velocity_limit_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated cartesian_velocity_limit to %f", cartesian_velocity_limit_);
    } else if (param.get_name() == "K_NULL" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      K_NULL_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated K_NULL to %f", K_NULL_);
    } else if (param.get_name() == "dls_lambda" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      dls_lambda_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated dls_lambda to %f", dls_lambda_);
    } else if (param.get_name() == "max_spring_stretch" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      max_spring_stretch_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated max_spring_stretch to %f", max_spring_stretch_);
    } else if (param.get_name() == "use_target_interpolation" && param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      use_target_interpolation_ = param.as_bool();
      RCLCPP_INFO(this->get_logger(), "Updated use_target_interpolation to %d", use_target_interpolation_);
    } else if (param.get_name() == "target_dt" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      target_dt_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated target_dt to %f", target_dt_);
    } else if (param.get_name() == "timer_dt" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      timer_dt_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated timer_dt to %f", timer_dt_);
      if (timer_) timer_->cancel();
      timer_ = this->create_wall_timer(std::chrono::duration<double>(timer_dt_), std::bind(&Franka_IJK::controlLoop, this));
    }
  }

  return result;
}

pinocchio::SE3 Franka_IJK::getInterpolatedTarget(const pinocchio::SE3& raw_target, double dt)
{
  // 1. Pass-Through if disabled
  if (!use_target_interpolation_) {
    return raw_target;
  }

  // 2. Snap to target on the very first frame to prevent sweeping across the room
  if (!first_target_received_) {
    smooth_target_se3_ = raw_target;
    first_target_received_ = true;
    return smooth_target_se3_;
  }

  // 3. Calculate interpolation step size
  double interpolation_factor = dt / target_dt_; 
  if (interpolation_factor > 1.0) interpolation_factor = 1.0; // Prevent overshooting

  // 4. Lerp Translation
  Eigen::Vector3d trans_error = raw_target.translation() - smooth_target_se3_.translation();
  smooth_target_se3_.translation() += trans_error * interpolation_factor;

  // 5. Slerp Rotation (Lie Algebra)
  pinocchio::SE3 relative_rot = smooth_target_se3_.inverse() * raw_target;
  Eigen::Vector3d angular_error_local = pinocchio::log3(relative_rot.rotation());
  
  Eigen::Matrix3d R_step = pinocchio::exp3(angular_error_local * interpolation_factor);
  smooth_target_se3_.rotation() = smooth_target_se3_.rotation() * R_step; 

  return smooth_target_se3_;
}

// =================================================================================
// Main Function
// =================================================================================

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting Cartesian Motion Controller Node...");
  auto node = std::make_shared<Franka_IJK>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}