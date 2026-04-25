# Mowbot: Slope-Slipping Issues & Resolution Strategies

This document summarizes the challenges and proposed strategies for handling steep terrains and wheel/track slipping for the Mowbot platform.

## 1. Monitoring & Detection
*   **Tilt Measurement (Pitch/Roll)**:
    - **Strategy**: Subscribe to `/odometry/local` or `/imu/data`. Convert orientation (Quaternions) to Euler angles.
    - **Use Case**: Real-time estimation of the robot's orientation in the world frame.
*   **Longitudinal Slip Detection (Traction Loss)**:
    - **Strategy**: Compare commanded velocity (`/cmd_vel`) against actual movement from fused odometry (`/odometry/local`) or raw wheel encoders.
    - **Detection Logic**: If `cmd_vel > 0.3 m/s` but `odom_vel < 0.05 m/s` for 1-2 seconds, the robot is likely stuck spinning in place.
*   **Lateral Slip Detection (Drifting)**:
    - **Strategy**: Monitor side-velocity (lateral movement) while the robot is attempting to drive straight on a side slope.

## 2. Adaptive Control (Velocity Scaling)
*   **Tilt-Regulated Velocity Scaling**:
    - **Strategy**: Implement a standalone `mowbot_tilt_limiter` node.
    - **Logic**: Apply a scaling factor $k \in [0.1, 1.0]$ to `/cmd_vel` based on the current pitch and roll.
    - **Action**: Lower speeds increase torque and reduce the rotational momentum that causes sliding on loose grass.
*   **Gravity Compensation (Rollback Prevention)**:
    - **Strategy**: Implement "hill-start" logic. Apply higher initial torque or a "pseudo-brake" when starting uphill.
*   **Steering Compensation**:
    - **Strategy**: Adjust the robot's heading to crab-walk uphill if lateral drift is detected.

## 3. Smart Path Planning (Nav2 Integration)
*   **Slope Constraints (Costmap Filters)**:
    - **Strategy**: Use costmap filters (e.g., Speed Limit Filter or Keepout Filter) based on an elevation map or slope analysis.
    - **Goal**: Proactively avoid slopes exceeding the physical limit (e.g., > 30°).
*   **Optimal Climbing Strategy**:
    - **Strategy**: Configure the global planner to prefer paths that go straight up or down rather than traversing across slopes (side-loading is the #1 cause of track slippage).

## 4. Safety & Recovery Actions
*   **Anti-Rollover (Critical Tilt-Stop)**:
    - **Strategy**: Independent watchdog node that monitors IMU/Odom.
    - **Action**: Immediate stop if tilt > 35°.
*   **Slipping Recovery Behavior**:
    - **Strategy**: Add a custom Nav2 recovery plugin.
    - **Action**: If slipping is detected for > 3 seconds, reverse briefly to reach stable ground and replan.
*   **Localization Stability**:
    - **Strategy**: Increase the trust (lower covariance) of GPS/IMU in the EKF (Extended Kalman Filter) when the robot is operating on steep dopes where wheel encoders become unreliable.

---
**Status**: Analysis complete. 
**Proposed Next Step**: Design and implement a standalone `mowbot_tilt_limiter` node for Reactive Velocity Scaling.
