# EECE 5554 Lab 4: IMU and GPS-Based Navigation with Dead Reckoning

## Project Overview

This project, completed for the EECE 5554 Robotics Sensing and Navigation course, implements a navigation stack utilizing data from Inertial Measurement Unit (IMU) and GPS sensors. The primary objectives were to calibrate the magnetometer, estimate yaw through sensor fusion, compute forward velocity, and perform dead reckoning to create a vehicle trajectory. Through sensor fusion and periodic GPS corrections, we aimed to develop a reliable path estimation technique that overcomes the drift issues inherent in IMU-based dead reckoning.

## Data Collection

Data was gathered in two main scenarios:

- **Donut Data (Circular Path)**: This data was collected by driving in a circular path, allowing for effective magnetometer calibration.
- **Driving Data (GPS and IMU)**: This dataset was recorded by driving through the Northeastern University area, starting and ending near Ruggles Station in Boston. The driving data provided a realistic environment for testing dead reckoning and comparing the IMU-derived path with GPS data.

## Project Components

The project involved the following key scripts and processes:

1. **Magnetometer Calibration** (`magnetometer_calibration.m`): This script applies hard-iron and soft-iron calibration on the magnetometer data collected in the circular path (donut data), transforming the elliptical raw data into a corrected circular pattern. Accurate magnetometer calibration is essential for reliable yaw estimation.
2. **Yaw Estimation Using Sensor Fusion** (`sensor_fusion.m`): To estimate the yaw angle, this script employs a complementary filter, combining gyroscope and magnetometer data to balance short-term responsiveness with long-term stability. This fused yaw estimate corrects for the drift inherent in gyroscope data, providing a stable heading reference for navigation.
3. **Forward Velocity Estimation** (`fwd_vel_est.m`): Forward velocity is derived by integrating the IMU’s accelerometer data along the forward axis. The script removes bias and applies filtering to minimize drift. GPS-based velocity serves as a benchmark for adjusting the IMU-based velocity.
4. **Dead Reckoning Path Estimation** (`deadrocking.m`): This script computes the vehicle’s path by integrating forward velocity and yaw estimates over time. Periodic corrections from GPS data are applied to mitigate drift, ensuring that the IMU-derived trajectory aligns with the GPS path for accurate dead reckoning.

## Collaborator Contributions

The data collected for this project and several essential resources were made available by **Dhyey Mistry** (GitLab username: `mistry.dhy`). Dhyey contributed by uploading the IMU drivers and ROSbag data files necessary for running the analyses on GitLab. His contributions provided a solid foundation for data collection, enabling the calibration and dead reckoning processes.

## Results

1. **Magnetometer Calibration**: The calibration steps corrected magnetic distortions, aligning the magnetometer data with the true magnetic field. This calibration was essential for accurate yaw estimation in later stages.
2. **Yaw Estimation**: The complementary filter effectively combined gyroscope and magnetometer readings to produce a stable yaw estimate, necessary for directional updates during dead reckoning.
3. **Forward Velocity Estimation**: By adjusting IMU-derived velocity with GPS data, we achieved a consistent forward velocity estimate that minimized drift.
4. **Dead Reckoning**: The dead reckoning path was computed by integrating forward velocity and yaw estimates. Periodic GPS corrections allowed the IMU-based path to maintain accuracy over time, aligning closely with the GPS trajectory.

## Files

- `magnetometer_calibration.m`: Script for calibrating magnetometer data.
- `sensor_fusion.m`: Script for combining magnetometer and gyroscope data to estimate yaw.
- `fwd_vel_est.m`: Script for forward velocity estimation using IMU and GPS data.
- `deadrocking.m`: Script for dead reckoning path estimation with periodic GPS corrections.