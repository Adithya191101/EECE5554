% Load IMU data
imuData = readtable('/MATLAB Drive/RSN_lab4/Donut/imu_data.csv');

% Extract magnetometer data
magX = imuData.magnetic_field_x;
magY = imuData.magnetic_field_y;

% Plot raw magnetometer data (X vs. Y)
figure;
plot(magX, magY, 'b.');
title('Raw Magnetometer Data');
xlabel('Magnetic Field X (Tesla)');
ylabel('Magnetic Field Y (Tesla)');
axis equal;
grid on;

% Hard-Iron Correction
% Calculate offsets based on the center of the data
offsetX = (max(magX) + min(magX)) / 2;
offsetY = (max(magY) + min(magY)) / 2;

% Apply hard-iron correction
magX_corrected = magX - offsetX;
magY_corrected = magY - offsetY;

% Soft-Iron Correction
% Calculate scaling factors to normalize the data range along each axis
scaleFactorX = (max(magX_corrected) - min(magX_corrected)) / 1.89;
scaleFactorY = (max(magY_corrected) - min(magY_corrected)) / 2;
avgScale = (scaleFactorX + scaleFactorY) / 2;

% Calculate individual scaling factors for each axis
scaleX = avgScale / scaleFactorX;
scaleY = avgScale / scaleFactorY;

% Apply soft-iron correction
magX_final = magX_corrected * scaleX;
magY_final = magY_corrected * scaleY;

% Plot corrected magnetometer data
figure;
plot(magX_final, magY_final, 'r.');
title('Corrected Magnetometer Data');
xlabel('Magnetic Field X (Tesla)');
ylabel('Magnetic Field Y (Tesla)');
axis equal;
grid on;

% Display results side-by-side for comparison
figure;
subplot(1, 2, 1);
plot(magX, magY, 'b.');
title('Raw Magnetometer Data');
xlabel('Magnetic Field X (Tesla)');
ylabel('Magnetic Field Y (Tesla)');
axis equal;
grid on;

subplot(1, 2, 2);
plot(magX_final, magY_final, 'r.');
title('Corrected Magnetometer Data');
xlabel('Magnetic Field X (Tesla)');
ylabel('Magnetic Field Y (Tesla)');
axis equal;
grid on;
