# Validation Plan: RTK Failover & Dead Reckoning Performance

## 1. Overview
This document outlines the strategy for validating the Mowbot's localization performance during RTK correction loss. The goal is to empirically verify that the system can maintain a drift window of 20-30cm over 10-30 meters of travel without high-precision corrections.

## 2. Methodology: 3-Tier Shadowing
The validation utilizes a "Shadowing" technique, where three independent Extended Kalman Filter (EKF) instances run concurrently on the same hardware to compare localization tiers in real-time.

| Tier | Name | Sensor Configuration | Purpose |
| :--- | :--- | :--- | :--- |
| **Tier 1** | Baseline | IMU + Odom + RTK Fixed | Reference / Ground Truth |
| **Tier 2** | Degraded | IMU + Odom + GNSS (Single/DGPS) | Evaluates GNSS noise filtering |
| **Tier 3** | Outage | IMU + Odom (Pure Dead Reckoning) | Evaluates inertial drift rate |

## 3. Test Scenarios (Upcoming)
Tests will be conducted across multiple trials ($n$ iterations) in varied environments (Open field, building shadows, and tree canopies).

### Scenario A: RTCM Correction Loss
*   **Action**: Disable the NTRIP client / RTCM stream while the robot is navigating.
*   **Assessment**: Measure stability during the transition from centimeter-level to meter-level raw GNSS accuracy.

### Scenario B: Total GNSS Outage
*   **Action**: Physically disconnect or block the GNSS signal.
*   **Assessment**: Determine the maximum safe distance the robot can travel before cumulative drift triggers a safety abort.

## 4. Performance Metrics
To quantify the results, the following metrics will be logged for each trial:

*   **Max Lateral Drift (Cross-Track Error)**: The maximum deviation from the global plan during the failure window.
    * *Threshold: < 25cm*
*   **Recovery Displacement**: The distance "jump" between the Dead Reckoning estimate and the actual RTK-Fixed pose upon signal re-acquisition.
    * *Threshold: < 30cm*
*   **Recovery Smoothness**: Rate of re-alignment (Pose Jump / Time) to ensure no jerky movements.
    * *Threshold: < 0.5 m/s equivalent*

## 5. Hardware Configuration (Field Trials)
For objective benchmarking, the field trial robot will be equipped as follows:
*   **Dual-Receiver Setup**: Two independent GNSS modules running in parallel.
*   **RF Signal Splitters**: Signal from a single pair of antennas is split to both receivers.
    *   *Receiver 1*: Stays in RTK-Fixed mode as the standard.
    *   *Receiver B*: Injected with simulated faults (RTCM cut or GNSS cut).

## 6. Success Criteria
The validation is considered successful if:
1.  The robot maintains a trajectory within **25cm** of the Baseline for at least **15 meters** in Scenario B.
2.  All RTK-loss events are correctly flagged as `WARNING` diagnostics.
3.  VDA5050 error messages are correctly transmitted to the Master Controller with accurate error levels.
