% Dead_Reckoning_Smooth_Alignment.m
% MATLAB script for Dead Reckoning using IMU and GPS with smooth drift correction

% Load IMU and GPS data
imuData = readtable('/MATLAB Drive/RSN_lab4/driving/imu_data.csv'); % Path to IMU data
gpsData = readtable('/MATLAB Drive/RSN_lab4/driving/gps_data.csv'); % Path to GPS data

% Extract IMU timestamps and forward acceleration (assuming 'linear_acceleration_x' as forward axis)
imu_timestamps = imuData.timestamp - imuData.timestamp(1); % Normalize IMU time
accel_x = imuData.linear_acceleration_x;

% Step 1: Calculate Forward Velocity from IMU
% Remove any initial bias from the acceleration
accel_bias = mean(accel_x(1:100)); % Adjust as needed
accel_x_corrected = accel_x - accel_bias;

% Integrate corrected acceleration to get forward velocity
velocity_imu = cumtrapz(imu_timestamps, accel_x_corrected);

% Apply a low-pass filter to reduce noise in velocity
Fs = 1 / mean(diff(imu_timestamps)); % Sampling frequency
cutoff = 0.1; % Low-pass cutoff frequency
[b, a] = butter(2, cutoff / (Fs / 2), 'low');
velocity_imu_filtered = filtfilt(b, a, velocity_imu);

% Detrend velocity to remove residual drift
velocity_imu_adjusted = detrend(velocity_imu_filtered);

% Step 2: Calculate Yaw from Quaternion Orientation Data
qx = imuData.orientation_x;
qy = imuData.orientation_y;
qz = imuData.orientation_z;
qw = imuData.orientation_w;
yaw = atan2(2 * (qw .* qz + qx .* qy), 1 - 2 * (qy.^2 + qz.^2));

% Step 3: Convert to Easting (ve) and Northing (vn) Components
scaling_factor = 1.05; % Adjust as necessary
ve = velocity_imu_adjusted .* cos(yaw) * scaling_factor;
vn = velocity_imu_adjusted .* sin(yaw) * scaling_factor;

% Step 4: Initialize Position Variables for Complementary Filter
xe = zeros(size(imu_timestamps)); % Easting position estimate
xn = zeros(size(imu_timestamps)); % Northing position estimate
alpha = 0.98; % Complementary filter weight (close to 1 means more reliance on IMU, close to 0 means more GPS)

% Extract GPS position for interpolation
gps_timestamps = gpsData.gpsTimeStamps - gpsData.gpsTimeStamps(1); % Normalize GPS time
gps_x = gpsData.utmEasting - gpsData.utmEasting(1); % Normalize GPS Easting start point
gps_y = gpsData.utmNorthing - gpsData.utmNorthing(1); % Normalize GPS Northing start point

% Remove any rows with NaN or infinite values in GPS data
valid_gps_indices = isfinite(gps_timestamps) & isfinite(gps_x) & isfinite(gps_y);
gps_timestamps = gps_timestamps(valid_gps_indices);
gps_x = gps_x(valid_gps_indices);
gps_y = gps_y(valid_gps_indices);

% Interpolate GPS position to match IMU timestamps for smoother correction
gps_x_interp = interp1(gps_timestamps, gps_x, imu_timestamps, 'linear', 'extrap');
gps_y_interp = interp1(gps_timestamps, gps_y, imu_timestamps, 'linear', 'extrap');

% Step 5: Apply Complementary Filter for Smoother Drift Correction
current_x = 0;
current_y = 0;

for i = 2:length(imu_timestamps)
    dt = imu_timestamps(i) - imu_timestamps(i - 1);
    
    % Update IMU-based position estimate
    current_x = current_x + ve(i) * dt;
    current_y = current_y + vn(i) * dt;
    
    % Blend with GPS position using complementary filter
    current_x = alpha * current_x + (1 - alpha) * gps_x_interp(i);
    current_y = alpha * current_y + (1 - alpha) * gps_y_interp(i);
    
    % Store position
    xe(i) = current_x;
    xn(i) = current_y;
end

% Step 6: Plot the Smoothly Corrected Dead Reckoning Path and GPS Path
figure;
plot(gps_x, gps_y, 'b', 'DisplayName', 'GPS Path');
hold on;
plot(xe, xn, 'r', 'DisplayName', 'Smooth Drift-Corrected Dead Reckoning Path');
title('Smooth Drift-Corrected Dead Reckoning Path vs. GPS Path');
xlabel('Easting (m)');
ylabel('Northing (m)');
legend;
grid on;

% Step 7: Calculate Displacement and Compare with GPS Displacement
displacement_imu = cumtrapz(imu_timestamps, velocity_imu_adjusted);

% Calculate GPS displacement for comparison
gps_displacement = sqrt((gps_x_interp - gps_x_interp(1)).^2 + (gps_y_interp - gps_y_interp(1)).^2);

% Plot IMU vs GPS Displacement
figure;
plot(imu_timestamps, displacement_imu, 'r', 'DisplayName', 'IMU Displacement');
hold on;
plot(imu_timestamps, gps_displacement, 'b', 'DisplayName', 'GPS Displacement');
xlabel('Time (s)');
ylabel('Displacement (m)');
title('Displacement Comparison: IMU vs GPS');
legend;
grid on;

% Step 8: Compute ω * x_dot and Compare with Corrected Forward Acceleration
omega = imuData.angular_velocity_z; % Yaw rate around Z-axis
x_dot = velocity_imu_adjusted; % Forward velocity as x_dot in this context

% Compute ω * x_dot for comparison
omega_x_dot = omega .* x_dot;

% Plot ω * x_dot and corrected forward acceleration (accel_x_corrected)
figure;
plot(imu_timestamps, omega_x_dot, 'r', 'DisplayName', 'ω * x_dot');
hold on;
plot(imu_timestamps, accel_x_corrected, 'b', 'DisplayName', 'Corrected Forward Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Comparison of ω * x_dot and Corrected Forward Acceleration');
legend;
grid on;

dead_reckoning_data = table(imu_timestamps, xe, xn, 'VariableNames', {'Time', 'Easting', 'Northing'});
writetable(dead_reckoning_data, 'smooth_drift_corrected_dead_reckoning_data.csv');
