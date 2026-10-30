#include "../include/franka_ijk.hpp"
#include "model_utils.hpp"

Franka_IJK::Franka_IJK() : Node("franka_ijk")
{
  // Setup Publisher and Subscriber
  // Initialize Publisher for dq
  target_dq_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("target_dq", 1);
  safe_target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "safe_target_pose", 1, std::bind(&Franka_IJK::safeTargetPoseCallback, this, std::placeholders::_1));
  joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 1, std::bind(&Franka_IJK::jointStateCallback, this, std::placeholders::_1));

  // TF Setup (for current pose access)
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // Parameters for arm prefix
  this->declare_parameter("arm_prefix", "franka_undefined");
  arm_prefix_ = this->get_parameter("arm_prefix").as_string();
  if (arm_prefix_ != "")
  {
    arm_prefix_ += "_";
  }

  std::string other_ns;
  if (arm_prefix_ == "franka_left_")
  {
    other_ns = "franka_right";
  }
  else if (arm_prefix_ == "franka_right_")
  {
    other_ns = "franka_left";
  }
  else
  {
    other_ns = "franka_undefined";
  }

  // Parameters for Initial Position
  this->declare_parameter("init_joint_position", std::vector<double>(7, 0.0));
  std::vector<double> init_joint_position_vec = this->get_parameter("init_joint_position").as_double_array();
  if (init_joint_position_vec.size() != 7 || std::all_of(init_joint_position_vec.begin(), init_joint_position_vec.end(),
                                                         [](double x) { return std::abs(x) < 1e-6; }))
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid init joint positions (wrong size or all zeros). Shutting down.");
    rclcpp::shutdown();
    return;
  }

  // Parameter declarations for Tuneable Variables
  this->declare_parameter("velocity_gain", 5.0);
  this->declare_parameter("joint_velocity_limit", 1.0);
  this->declare_parameter("cartesian_velocity_limit", 1.0);
  this->declare_parameter("K_NULL", 0.1);
  this->declare_parameter("dls_lambda", 0.0001);
  this->declare_parameter("dls_lambda_max", 0.05);
  this->declare_parameter("singularity_threshold", 0.1);
  this->declare_parameter("joint_limit_buffer", 0.15);
  this->declare_parameter("joint_limit_gain", 5.0);
  this->declare_parameter("singularity_sigma_stop", 0.04);
  this->declare_parameter("singularity_gain", 2.0);
  this->declare_parameter("max_spring_stretch", 0.1);
  this->declare_parameter("use_target_interpolation", true);
  this->declare_parameter("target_dt", 0.02);
  this->declare_parameter("timer_dt", 0.005);

  velocity_gain_ = this->get_parameter("velocity_gain").as_double();
  joint_velocity_limit_ = this->get_parameter("joint_velocity_limit").as_double();
  cartesian_velocity_limit_ = this->get_parameter("cartesian_velocity_limit").as_double();
  K_NULL_ = this->get_parameter("K_NULL").as_double();
  dls_lambda_ = this->get_parameter("dls_lambda").as_double();
  dls_lambda_max_ = this->get_parameter("dls_lambda_max").as_double();
  singularity_threshold_ = this->get_parameter("singularity_threshold").as_double();
  joint_limit_buffer_ = this->get_parameter("joint_limit_buffer").as_double();
  joint_limit_gain_ = this->get_parameter("joint_limit_gain").as_double();
  singularity_sigma_stop_ = this->get_parameter("singularity_sigma_stop").as_double();
  singularity_gain_ = this->get_parameter("singularity_gain").as_double();
  max_spring_stretch_ = this->get_parameter("max_spring_stretch").as_double();
  use_target_interpolation_ = this->get_parameter("use_target_interpolation").as_bool();
  target_dt_ = this->get_parameter("target_dt").as_double();
  timer_dt_ = this->get_parameter("timer_dt").as_double();

  param_callback_handle_ =
      this->add_on_set_parameters_callback(std::bind(&Franka_IJK::parametersCallback, this, std::placeholders::_1));

  this->declare_parameter("end_effector_frame", "");
  end_effector_frame_ = this->get_parameter("end_effector_frame").as_string();

  // Load Pinocchio Model
  if (!loadPinocchioModel())
  {
    RCLCPP_FATAL(this->get_logger(), "Failed to load Pinocchio model. Shutting down.");
    rclcpp::shutdown();
    return;
  }

  // Size q_init_ to model_.nv now that the model is loaded.
  // The parameter holds 7 arm joint values; extra DOFs (e.g. gripper) stay at 0.
  q_init_ = Eigen::VectorXd::Zero(model_.nv);
  for (int i = 0; i < 7; ++i)
  {
    std::string name = arm_prefix_ + "fr3_joint" + std::to_string(i + 1);
    if (model_.existJointName(name))
    {
      int idx = model_.joints[model_.getJointId(name)].idx_v();
      q_init_(idx) = init_joint_position_vec[i];
    }
  }
  RCLCPP_INFO(this->get_logger(), "Initial joint position loaded for nullspace control. model_.nv=%d", model_.nv);

  timer_ = this->create_wall_timer(std::chrono::duration<double>(timer_dt_), std::bind(&Franka_IJK::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "franka_ijk initialized.");
}

bool Franka_IJK::loadPinocchioModel()
{
  if (!robot_safety_layer::ModelLoader::loadPinocchioModel(this, model_, data_))
  {
    return false;
  }

  if (end_effector_frame_.empty())
  {
    end_effector_frame_ = arm_prefix_ + "rh_p12_rn_grasp_point";
    RCLCPP_WARN(this->get_logger(), "end_effector_frame parameter not set. Falling back to default: %s",
                end_effector_frame_.c_str());
  }

  if (!model_.existFrame(end_effector_frame_))
  {
    RCLCPP_ERROR(this->get_logger(), "End effector link '%s' not found in model.", end_effector_frame_.c_str());
    return false;
  }

  ee_frame_id_ = model_.getFrameId(end_effector_frame_);
  return true;
}

void Franka_IJK::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  RCLCPP_INFO_ONCE(this->get_logger(), "Got joint state");
  if (q_.size() != model_.nv)
  {
    q_ = Eigen::VectorXd::Zero(model_.nv);
  }
  for (size_t i = 0; i < msg->name.size(); ++i)
  {
    if (model_.existJointName(msg->name[i]))
    {
      int idx_q = model_.joints[model_.getJointId(msg->name[i])].idx_q();
      if (idx_q >= 0 && idx_q < model_.nq)
      {
        q_(idx_q) = msg->position[i];
      }
    }
  }
}

void Franka_IJK::safeTargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  processTargetPose(*msg);
}

bool Franka_IJK::processTargetPose(const geometry_msgs::msg::PoseStamped& msg)
{
  try
  {
    tf_buffer_->transform(msg, target_pose_stamped_, target_frame_);
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "Could not transform target pose from '%s' to '%s': %s", msg.header.frame_id.c_str(),
                          target_frame_.c_str(), ex.what());
    return false;
  }

  target_se3_.translation() << target_pose_stamped_.pose.position.x, target_pose_stamped_.pose.position.y,
      target_pose_stamped_.pose.position.z;

  Eigen::Quaterniond q_rot(target_pose_stamped_.pose.orientation.w, target_pose_stamped_.pose.orientation.x,
                           target_pose_stamped_.pose.orientation.y, target_pose_stamped_.pose.orientation.z);
  target_se3_.rotation() = q_rot.toRotationMatrix();

  return true;
}

pinocchio::SE3 Franka_IJK::computeForwardKinematic(Eigen::VectorXd q)
{
  pinocchio::forwardKinematics(model_, *data_, q);
  pinocchio::updateFramePlacements(model_, *data_);
  return data_->oMf[ee_frame_id_];
}

double Franka_IJK::computeCartesianVelocity(const pinocchio::SE3& current_se3, const pinocchio::SE3& target_se3,
                                            Eigen::VectorXd& desired_cartesian_velocity)
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
  desired_cartesian_velocity.head<3>() = velocity_gain_ * lin_world_error;      // / dt_;
  desired_cartesian_velocity.tail<3>() = velocity_gain_ * angular_world_error;  // / dt_;

  double target_reachable_factor = 1;

  // Limit velocity
  if (desired_cartesian_velocity.norm() > cartesian_velocity_limit_)
  {
    double reduction = cartesian_velocity_limit_ / desired_cartesian_velocity.norm();
    target_reachable_factor = 1 / reduction;
    desired_cartesian_velocity *= reduction;
  }

  // Apply tolerance to stop movement near target
  if (desired_cartesian_velocity.norm() < 1e-3)
  {
    desired_cartesian_velocity.setZero();
  }

  return target_reachable_factor;
}

double Franka_IJK::computeSigmaMin(const Eigen::VectorXd& q)
{
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, model_.nv);
  pinocchio::computeFrameJacobian(model_, *data_, q, ee_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
  return svd.singularValues()(svd.singularValues().size() - 1);
}

Eigen::VectorXd Franka_IJK::runJacobianNullspaceControl(const Eigen::VectorXd& desired_cartesian_velocity)
{
  // 1. Update Kinematics (needed for Jacobian calculation)
  pinocchio::computeAllTerms(model_, *data_, q_, Eigen::VectorXd::Zero(model_.nv));

  // 2. Compute the Jacobian matrix (6xN, N=DOF)
  Eigen::MatrixXd J(6, model_.nv);
  J.setZero();
  pinocchio::getFrameJacobian(model_, *data_, ee_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J);

  // 3. Secondary Task Velocity (Posture Control): dq_null_task = K_NULL_ * (q_init - q_curr)
  Eigen::VectorXd joint_error_posture = q_init_ - q_;
  Eigen::VectorXd dq_null_task = K_NULL_ * joint_error_posture;

  // 4. Joint limit velocity bounds for the arm joints: a joint may approach (limit - buffer) only
  // exponentially (dq <= gain * remaining distance), so it slows down smoothly and never gets there.
  std::vector<int> arm_idx;
  for (int i = 0; i < 7; ++i)
  {
    arm_idx.push_back(model_.joints[model_.getJointId(arm_prefix_ + "fr3_joint" + std::to_string(i + 1))].idx_v());
  }
  Eigen::VectorXd ub = Eigen::VectorXd::Constant(model_.nv, std::numeric_limits<double>::infinity());
  Eigen::VectorXd lb = -ub;
  for (int i : arm_idx)
  {
    ub(i) = joint_limit_gain_ * (model_.upperPositionLimit(i) - joint_limit_buffer_ - q_(i));
    lb(i) = joint_limit_gain_ * (model_.lowerPositionLimit(i) + joint_limit_buffer_ - q_(i));
  }

  // 5. Solve with saturation in the null space: the joint violating its bound the most is fixed at
  // its bound, its contribution is removed from the task, and the rest of the arm re-solves the task.
  std::vector<bool> saturated(model_.nv, false);
  Eigen::VectorXd dq_fixed = Eigen::VectorXd::Zero(model_.nv);
  Eigen::VectorXd dq = Eigen::VectorXd::Zero(model_.nv);
  for (size_t iter = 0; iter <= arm_idx.size(); ++iter)
  {
    Eigen::MatrixXd J_active = J;
    Eigen::VectorXd null_task = dq_null_task;
    for (int i = 0; i < model_.nv; ++i)
    {
      if (saturated[i])
      {
        J_active.col(i).setZero();
        null_task(i) = 0.0;
      }
    }
    Eigen::VectorXd task_velocity = desired_cartesian_velocity - (J - J_active) * dq_fixed;

    // 5.1. Damped Least Squares with adaptive damping: dls_lambda_ is the base damping, raised
    // smoothly up to dls_lambda_ + dls_lambda_max_ as the smallest singular value drops below singularity_threshold_
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_active);
    double sigma_min = svd.singularValues()(svd.singularValues().size() - 1);
    double lambda = dls_lambda_;
    if (sigma_min < singularity_threshold_)
    {
      double ratio = sigma_min / singularity_threshold_;
      lambda += (1.0 - ratio * ratio) * dls_lambda_max_;
    }
    Eigen::MatrixXd J_dagger =
        J_active.transpose() * (J_active * J_active.transpose() + lambda * Eigen::MatrixXd::Identity(6, 6)).inverse();

    // 5.2. Primary Task (Cartesian velocity) + Nullspace Posture Control: N = I - J_dagger * J
    Eigen::MatrixXd N = Eigen::MatrixXd::Identity(model_.nv, model_.nv) - J_dagger * J_active;
    dq = J_dagger * task_velocity + N * null_task;
    for (int i = 0; i < model_.nv; ++i)
    {
      if (saturated[i])
        dq(i) = dq_fixed(i);
    }

    // 5.3. Saturate the worst offender, if any
    int worst = -1;
    double worst_violation = 1e-9;
    for (int i : arm_idx)
    {
      double violation = std::max(dq(i) - ub(i), lb(i) - dq(i));
      if (!saturated[i] && violation > worst_violation)
      {
        worst = i;
        worst_violation = violation;
      }
    }
    if (worst < 0)
      break;
    saturated[worst] = true;
    dq_fixed(worst) = std::clamp(dq(worst), lb(worst), ub(worst));
  }

  // Inside the buffer the bound turns negative; only require "not further out" there
  for (int i : arm_idx)
  {
    dq(i) = std::clamp(dq(i), std::min(lb(i), 0.0), std::max(ub(i), 0.0));
  }

  // 6. Singularity barrier: the smallest singular value may only decrease as fast as
  // singularity_gain * (sigma - singularity_sigma_stop). Moving towards a singularity slows the arm
  // down smoothly until it stops at sigma_stop; moving away from it is never slowed.
  double sigma = computeSigmaMin(q_);
  Eigen::VectorXd grad = Eigen::VectorXd::Zero(model_.nv);
  const double h = 1e-5;
  for (int i : arm_idx)
  {
    Eigen::VectorXd q_plus = q_, q_minus = q_;
    q_plus(i) += h;
    q_minus(i) -= h;
    grad(i) = (computeSigmaMin(q_plus) - computeSigmaMin(q_minus)) / (2.0 * h);
  }
  double sigma_rate = grad.dot(dq);
  double allowed_rate = -singularity_gain_ * (sigma - singularity_sigma_stop_);
  if (sigma_rate < 0.0 && sigma_rate < allowed_rate)
  {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Singularity slowdown active");
    dq *= std::clamp(allowed_rate / sigma_rate, 0.0, 1.0);
  }

  return dq;
}

void Franka_IJK::controlLoop()
{
  // 1. Safety Checks: Do we have valid joint states yet?
  if (q_.size() == 0 || !q_.allFinite())
  {
    return;
  }

  // 2. Time management
  auto current_time = this->get_clock()->now();
  if (last_time_.nanoseconds() == 0)
  {
    last_time_ = current_time;
    return;  // Skip the first tick to get a clean dt_ next time
  }

  dt_ = (current_time - last_time_).seconds();
  last_time_ = current_time;

  // Clamp dt_ to prevent math explosions if the node stalls
  if (dt_ <= 0.0 || dt_ > target_dt_ * 2)
  {
    dt_ = target_dt_;  // Fallback to 200Hz
  }

  // 3. Check if we have received a target yet
  if (target_pose_stamped_.header.stamp.sec == 0)
  {
    return;
  }

  // --- Normal Execution ---
  pinocchio::SE3 current_se3 = computeForwardKinematic(q_);

  pinocchio::SE3 active_target_se3 = getInterpolatedTarget(target_se3_, dt_);

  // --- Compute Primary Task (Cartesian Error and Velocity) ---
  Eigen::VectorXd desired_cartesian_velocity;
  computeCartesianVelocity(current_se3, active_target_se3, desired_cartesian_velocity);

  Eigen::VectorXd dq = runJacobianNullspaceControl(desired_cartesian_velocity);

  // --- Velocity Limiting ---
  double max_dq = dq.array().abs().maxCoeff();
  if (max_dq > joint_velocity_limit_)
  {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Joint velocity Limit Active");
    dq *= (joint_velocity_limit_ / max_dq);
  }

  // --- Publish dq instead of final joint state ---
  sensor_msgs::msg::JointState dq_msg;
  dq_msg.header.stamp = current_time;
  dq_msg.name.resize(7);
  dq_msg.velocity.resize(7);
  for (int i = 0; i < 7; ++i)
  {
    dq_msg.name[i] = arm_prefix_ + "fr3_joint" + std::to_string(i + 1);
    int idx_v = model_.joints[model_.getJointId(dq_msg.name[i])].idx_v();
    dq_msg.velocity[i] = dq[idx_v];
  }
  target_dq_pub_->publish(dq_msg);
}

rcl_interfaces::msg::SetParametersResult
Franka_IJK::parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto& param : parameters)
  {
    if (param.get_name() == "velocity_gain" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      velocity_gain_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated velocity_gain to %f", velocity_gain_);
    }
    else if (param.get_name() == "joint_velocity_limit" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      joint_velocity_limit_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated joint_velocity_limit to %f", joint_velocity_limit_);
    }
    else if (param.get_name() == "cartesian_velocity_limit" &&
             param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      cartesian_velocity_limit_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated cartesian_velocity_limit to %f", cartesian_velocity_limit_);
    }
    else if (param.get_name() == "K_NULL" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      K_NULL_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated K_NULL to %f", K_NULL_);
    }
    else if (param.get_name() == "dls_lambda" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      dls_lambda_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated dls_lambda to %f", dls_lambda_);
    }
    else if (param.get_name() == "dls_lambda_max" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      dls_lambda_max_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated dls_lambda_max to %f", dls_lambda_max_);
    }
    else if (param.get_name() == "singularity_threshold" &&
             param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      singularity_threshold_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated singularity_threshold to %f", singularity_threshold_);
    }
    else if (param.get_name() == "joint_limit_buffer" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      joint_limit_buffer_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated joint_limit_buffer to %f", joint_limit_buffer_);
    }
    else if (param.get_name() == "joint_limit_gain" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      joint_limit_gain_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated joint_limit_gain to %f", joint_limit_gain_);
    }
    else if (param.get_name() == "singularity_sigma_stop" &&
             param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      singularity_sigma_stop_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated singularity_sigma_stop to %f", singularity_sigma_stop_);
    }
    else if (param.get_name() == "singularity_gain" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      singularity_gain_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated singularity_gain to %f", singularity_gain_);
    }
    else if (param.get_name() == "max_spring_stretch" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      max_spring_stretch_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated max_spring_stretch to %f", max_spring_stretch_);
    }
    else if (param.get_name() == "use_target_interpolation" &&
             param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
    {
      use_target_interpolation_ = param.as_bool();
      RCLCPP_INFO(this->get_logger(), "Updated use_target_interpolation to %d", use_target_interpolation_);
    }
    else if (param.get_name() == "target_dt" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      target_dt_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated target_dt to %f", target_dt_);
    }
    else if (param.get_name() == "timer_dt" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      timer_dt_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated timer_dt to %f", timer_dt_);
      if (timer_)
        timer_->cancel();
      timer_ =
          this->create_wall_timer(std::chrono::duration<double>(timer_dt_), std::bind(&Franka_IJK::controlLoop, this));
    }
  }

  return result;
}

pinocchio::SE3 Franka_IJK::getInterpolatedTarget(const pinocchio::SE3& raw_target, double dt)
{
  // 1. Pass-Through if disabled
  if (!use_target_interpolation_)
  {
    return raw_target;
  }

  if (!first_target_received_)
  {
    smooth_target_se3_ = raw_target;
    first_target_received_ = true;
    return smooth_target_se3_;
  }

  // 3. Calculate interpolation step size
  double interpolation_factor = dt / target_dt_;
  if (interpolation_factor > 1.0)
    interpolation_factor = 1.0;  // Prevent overshooting

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

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting Cartesian Motion Controller Node...");
  auto node = std::make_shared<Franka_IJK>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}