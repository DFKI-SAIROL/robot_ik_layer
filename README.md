# robot_ik_layer

Cartesian IK controller for the Franka FR3. Receives a target end-effector pose, computes joint velocities via Jacobian pseudoinverse with nullspace posture control, and publishes them for the safety layer to integrate and forward to the robot.

## Node: `ik_node`

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Sub | `safe_target_pose` | `PoseStamped` | Safety-checked Cartesian target from `robot_safety_layer` |
| Sub | `joint_states` | `JointState` | Actual robot joint positions |
| Pub | `target_dq` | `JointState` (velocity field) | Desired joint velocities (rad/s) |

