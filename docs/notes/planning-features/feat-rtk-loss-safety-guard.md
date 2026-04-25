# Feature Specification: RTK Loss Safety Guard (GNSS Covariance Threshold)

**Document Status:** 📝 Draft
**Date:** 2026-04-10
**Epic:** Autonomous Navigation Safety & Reliability
**Target Release:** TBD
**Author:** R&D Team

---

## 1. Executive Summary
Currently, the Mowbot utilizes an Extended Kalman Filter (EKF) for localization, seamlessly falling back to dead reckoning (Odom + IMU integration) when the RTK/GNSS signal is heavily degraded or lost. While this guarantees smooth navigation without stuttering, prolonged dead reckoning leads to unbounded position drift (`position_covariance`).
This feature aims to introduce a hard safety threshold that guarantees the robot transitions to a secure, stationary state when navigation accuracy drops below acceptable physical limits.

---

## 2. Problem Statement & Motivation
* **Customer Inquiry Context:** Customers are accustomed to traditional AGVs functioning with strict hardware-like GPS locks. "When RTK is lost, does it stop or continue driving?" 
* **Business Need:** We must leverage our superior EKF-based seamless transition as a selling point while simultaneously guaranteeing a fail-safe limits. Operating with massive positional covariance carries the risk of the robot violating virtual costmaps or physically bumping into constraints prior to the VDA5050 controller noticing an anomaly.
* **Goal:** Terminate navigation gracefully (and legally, from a VDA5050 state perspective) if the robot navigates blindly for too long.

---

## 3. User Stories
* **As a Fleet Manager**, I want the robot to automatically halt when its positional uncertainty grows too high, so that it does not wander off the path or hit obstacles.
* **As a Customer Service Agent**, I want the robot to explicitly report a "GNSS Position Lost" warning via the VDA5050 standard, so that I don't confuse this safety stop with a random physical blockage.
* **As an Engineer**, I want the braking action to trigger safely from both the lower execution level and accurately sync with the upper mission bridge states.

---

## 4. Technical Architecture Candidates

To fulfill the requirements, we explored three major implementation paths.

### Option A: High-Level State Management (Mission Bridge Only)
* **Design:** The `DiagnosticsListener` monitors `position_covariance` from the GNSS/sensor diagnostics. If the covariance exceeds a config threshold, trigger `order_manager.pause_order()`.
* **Pros:** Cleanly synchronizes with VDA5050, resulting in proper state tracking.
* **Cons:** Network delays or node crashing could stall the emergency stop execution.

### Option B: Low-Level Mux Injection (Standalone Safety Watchdog)
* **Design:** A dedicated ROS 2 node subscribes to `/odometry/filtered`. If the covariance threshold breaches the limit, it publishes `cmd_vel = 0.0` at the highest priority to `twist_mux`.
* **Pros:** Guaranteed hard safety override directly overriding motors.
* **Cons:** Severe State Mismatch footprint. Nav2 enters aggressive recovery behaviors while the mission bridge remains falsely confident that it is `AUTO_DRIVING`.

### Option C: Hybrid Edge-to-Bridge Control 🏆 *(Recommended)*
1. **Low-Level Watchdog:** A dedicated `gnss_safety_watchdog` node continually publishes `cmd_vel = 0.0` to `twist_mux` upon breaching the threshold, guaranteeing the immediate physical stopping of hardware.
2. **System Messaging:** The watchdog simultaneously triggers a boolean flag via `/diagnostics` or a dedicated `/safety/status` topic.
3. **Bridge Interception:** The `DiagnosticsListener` picks up the safety flag, executes `pause_order()` to gracefully cancel the Nav2 goal, mitigating Nav2 panic-loops, and updates the VDA5050 status to `PAUSED` or `ERROR`.

---

## 5. Acceptance Criteria
- [ ] **Configurability:** The threshold parameters (`max_covariance_xy`, `max_rtk_loss_time_sec`) must be externally configurable via launch parameters.
- [ ] **Safety Stop Verification:** When the covariance limit is breached, the robot's physical velocity must drop to 0 m/s within 250 miliseconds.
- [ ] **Nav2 Cancellation:** Upon triggering the safety constraint, Nav2 action goals must be preempted/cancelled to avoid pointless recovery loops.
- [ ] **Fleet Monitoring Delivery:** The VDA5050 bridge must publish an `ERROR` or `WARNING` code reflecting the lost position.
- [ ] **Resumption:** If the robot successfully re-acquires RTK Fixed/Float status (covariance drops below the safe threshold), it may allow resuming via the standard VDA5050 play command.

---

## 6. Open Questions & Dependencies
| Task / Question | Owner / Notes |
|-----------------|---------------|
| What are the exact physical covariance numbers (in meters) to define as "critical drift"? | TBD directly via field testing |
| Ensure `twist_mux` priority levels are correctly set prior to node insertion. | Robotics Dev |
| Which VDA5050 standard error code should we use for GNSS failure? | Needs verification with API doc |
