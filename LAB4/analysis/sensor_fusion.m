% Load the IMU data
imuData = readtable('/MATLAB Drive/RSN_lab4/driving/imu_data.csv');

% Extract specific columns
timestamps = imuData.timestamp - imuData.timestamp(1); % Normalize time
mag_x_vals = imuData.magnetic_field_x;
mag_y_vals = imuData.magnetic_field_y;
gyro_yaw_rate = imuData.angular_velocity_z;

% Convert quaternion to yaw (if quaternion data is available in the IMU data)
qx = imuData.orientation_x;
qy = imuData.orientation_y;
qz = imuData.orientation_z;
qw = imuData.orientation_w;

imu_yaw_angle = atan2(2 * (qw .* qz + qx .* qy), 1 - 2 * (qy.^2 + qz.^2));

% Step 1: Hard-Iron Correction
hard_iron_x = (max(mag_x_vals) + min(mag_x_vals)) / 2;
hard_iron_y = (max(mag_y_vals) + min(mag_y_vals)) / 2;
mag_x_adj = mag_x_vals - hard_iron_x;
mag_y_adj = mag_y_vals - hard_iron_y;

% Step 2: Soft-Iron Correction with Least Squares Fitting
ellipse_residual = @(params, x, y) ...
    ((x - params(1)) * cos(params(5)) + (y - params(2)) * sin(params(5))) / params(3).^2 + ...
    (-(x - params(1)) * sin(params(5)) + (y - params(2)) * cos(params(5))) / params(4).^2 - 1;

initial_params = [mean(mag_x_adj), mean(mag_y_adj), std(mag_x_adj), std(mag_y_adj), 0];
fitted_params = lsqnonlin(@(p) ellipse_residual(p, mag_x_adj, mag_y_adj), initial_params);

% Extract fitted parameters
x_c = fitted_params(1);
y_c = fitted_params(2);
a_axis = fitted_params(3);
b_axis = fitted_params(4);
angle = fitted_params(5);

% Apply soft-iron correction
cos_angle = cos(angle);
sin_angle = sin(angle);
mag_x_final = (mag_x_adj * cos_angle - mag_y_adj * sin_angle) / a_axis;
mag_y_final = (mag_x_adj * sin_angle + mag_y_adj * cos_angle) / b_axis;

% Step 3: Calculate Magnetometer Yaw with Smoothing
calibrated_mag_yaw = atan2(mag_y_final, mag_x_final);
calibrated_mag_yaw = smooth(calibrated_mag_yaw, 5); % Apply moving average filter

% Step 4: Plot Time Series of Magnetometer Yaw Data Before and After Correction
raw_mag_yaw = atan2(mag_y_vals, mag_x_vals);

figure;
plot(timestamps, raw_mag_yaw, 'b', 'DisplayName', 'Raw Magnetometer Yaw');
hold on;
plot(timestamps, calibrated_mag_yaw, 'r', 'DisplayName', 'Corrected Magnetometer Yaw');
title('Time Series of Magnetometer Yaw Data (Before and After Correction)');
xlabel('Time (s)');
ylabel('Yaw Angle (radians)');
legend;
grid on;

% Step 5: Gyro Bias Correction and Integration
gyro_bias = mean(gyro_yaw_rate(1:100)); % Adjust based on stationary period
gyro_yaw_rate_corrected = gyro_yaw_rate - gyro_bias;
yaw_angle_from_gyro = cumtrapz(timestamps, gyro_yaw_rate_corrected);

% Step 6: Plot yaw estimates from magnetometer and gyro integration
figure;
plot(timestamps, calibrated_mag_yaw, 'b', 'DisplayName', 'Magnetometer Yaw');
hold on;
plot(timestamps, yaw_angle_from_gyro, 'Color', [1, 0.5, 0], 'DisplayName', 'Integrated Gyro Yaw'); % Orange color in RGB
title('Yaw Angle from Magnetometer vs. Integrated Gyro');
xlabel('Time (s)');
ylabel('Yaw (rad)');
legend;

% Step 7: Apply Complementary Filter with Adjusted Parameters
alpha_lpf = 0.02; % Lower for more emphasis on magnetometer
alpha_hpf = 0.1;  % High-pass filter parameter
cf_weight = 0.85; % Reduced to further emphasize magnetometer for long-term accuracy

% Low-Pass Filter for Magnetometer Yaw
filtered_mag_yaw = calibrated_mag_yaw;
for i = 2:length(filtered_mag_yaw)
    filtered_mag_yaw(i) = alpha_lpf * calibrated_mag_yaw(i) + (1 - alpha_lpf) * filtered_mag_yaw(i - 1);
end

% High-Pass Filter for Gyro Yaw Rate
filtered_gyro_yaw_rate = zeros(size(gyro_yaw_rate_corrected));
for i = 2:length(filtered_gyro_yaw_rate)
    filtered_gyro_yaw_rate(i) = gyro_yaw_rate_corrected(i) - (1 - alpha_hpf) * filtered_gyro_yaw_rate(i - 1);
end

% Complementary Filter for Yaw Angle
complementary_yaw = zeros(size(yaw_angle_from_gyro));
for i = 2:length(complementary_yaw)
    complementary_yaw(i) = cf_weight * (complementary_yaw(i - 1) + filtered_gyro_yaw_rate(i) * (timestamps(i) - timestamps(i - 1))) + ...
                           (1 - cf_weight) * filtered_mag_yaw(i);
end

% Step 8: Plot complementary filter and individual yaw estimates
figure;
plot(timestamps, filtered_mag_yaw, 'g', 'DisplayName', 'LPF Magnetometer Yaw');
hold on;
plot(timestamps, filtered_gyro_yaw_rate, 'Color', [1, 0.5, 0], 'DisplayName', 'HPF Gyro Yaw Rate'); % Orange color in RGB
plot(timestamps, complementary_yaw, 'r', 'DisplayName', 'Complementary Yaw');
title('Yaw Estimates with Complementary Filter');
xlabel('Time (s)');
ylabel('Yaw (rad)');
legend;

% Step 9: Compare Complementary Filter Yaw with IMU Yaw
figure;
plot(timestamps, complementary_yaw, 'r', 'DisplayName', 'Adjusted Complementary Filter Yaw');
hold on;
plot(timestamps, -yaw_angle_from_gyro, 'b', 'DisplayName', 'IMU Yaw');
title('Adjusted Complementary Filter Yaw vs. IMU Yaw');
xlabel('Time (s)');
ylabel('Yaw (rad)');
legend;
grid on;
