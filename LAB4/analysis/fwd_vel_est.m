% Load IMU and GPS data
imuData = readtable('/MATLAB Drive/RSN_lab4/driving/imu_data.csv'); % Path to IMU data
gpsData = readtable('/MATLAB Drive/RSN_lab4/driving/gps_data.csv'); % Path to GPS data

% Extract IMU timestamps (normalized) and accelerometer data along the forward axis
imu_timestamps = imuData.timestamp - imuData.timestamp(1); % Normalize time
accel_x = imuData.linear_acceleration_x;

% Step 1: Remove Accelerometer Bias (initial bias estimate from stationary period)
accel_bias = mean(accel_x(1:100)); % Adjust based on stationary segment
accel_x_corrected = accel_x - accel_bias;

% Step 2: Integrate Acceleration to Estimate Unadjusted Velocity (IMU-Based Velocity Before Adjustment)
velocity_imu_unadjusted = cumtrapz(imu_timestamps, accel_x_corrected);

% Step 3: Calculate Velocity from GPS Data
% Extract GPS timestamps and UTM coordinates
gps_timestamps = gpsData.gpsTimeStamps - gpsData.gpsTimeStamps(1); % Normalize time
gps_x = gpsData.utmEasting;
gps_y = gpsData.utmNorthing;

% Calculate GPS-Based Velocity
gps_velocity = zeros(length(gps_x)-1, 1);
for i = 2:length(gps_x)
    distance = sqrt((gps_x(i) - gps_x(i-1))^2 + (gps_y(i) - gps_y(i-1))^2);
    time_interval = gps_timestamps(i) - gps_timestamps(i-1);
    gps_velocity(i-1) = distance / time_interval;
end

% Adjust GPS timestamps vector to match the reduced gps_velocity vector size
gps_timestamps_adjusted = gps_timestamps(1:end-1);

% Step 4: Align IMU and GPS-Based Velocity Estimates by Interpolating GPS Data
gps_velocity_interp = interp1(gps_timestamps_adjusted, gps_velocity, imu_timestamps, 'linear', 'extrap');

% Plot 1: Velocity Estimate from GPS with Unadjusted IMU Velocity
figure;
plot(imu_timestamps, velocity_imu_unadjusted, 'r', 'DisplayName', 'Unadjusted IMU-Based Velocity');
hold on;
plot(imu_timestamps, gps_velocity_interp, 'b', 'DisplayName', 'Interpolated GPS-Based Velocity');
title('Comparison of Forward Velocity Estimates (Unadjusted)');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend;
grid on;

% Step 5: Apply Low-Pass Filter to IMU Velocity to Reduce Noise
Fs = 1 / mean(diff(imu_timestamps)); % Sampling frequency
cutoff = 0.1; % Low-pass cutoff frequency (adjust as needed)
[b, a] = butter(2, cutoff / (Fs / 2), 'low');
velocity_imu_filtered = filtfilt(b, a, velocity_imu_unadjusted);

% Step 6: Detrend IMU Velocity to Minimize Drift (Adjusted IMU-Based Velocity)
velocity_imu_adjusted = detrend(velocity_imu_filtered);

% Plot 2: Velocity Estimate from GPS with Adjusted IMU Velocity
figure;
plot(imu_timestamps, velocity_imu_adjusted, 'r', 'DisplayName', 'Adjusted IMU-Based Velocity');
hold on;
plot(imu_timestamps, gps_velocity_interp, 'b', 'DisplayName', 'Interpolated GPS-Based Velocity');
title('Comparison of Forward Velocity Estimates (Adjusted)');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend;
grid on;
