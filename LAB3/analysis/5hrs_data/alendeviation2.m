% Load the 5-hour IMU data from MATLAB Drive
data = readtable('/MATLAB Drive/RSN/imu_data_5hrs.csv');

% Extract time vector (assuming time is in seconds)
t = data.timestamp - data.timestamp(1);

% Sampling frequency (Hz)
fs = 1 / mean(diff(t));

% Define the sensors and their axes
sensors = {'Accelerometer', 'Gyroscope', 'Magnetometer'};
axes = {'X', 'Y', 'Z'};

for i = 1:length(sensors)
    figure('Name', [sensors{i} ' Allan Deviation'], 'Position', [100, 100, 1200, 400]);
    
    for j = 1:length(axes)
        subplot(1, 3, j);
        
        % Extract data for current sensor and axis
        if strcmp(sensors{i}, 'Accelerometer')
            sensor_data = data.(['linear_acceleration_' lower(axes{j})]);
            units = 'm/s^2';
        elseif strcmp(sensors{i}, 'Gyroscope')
            sensor_data = data.(['angular_velocity_' lower(axes{j})]);
            units = 'rad/s';
        else
            sensor_data = data.(['magnetic_field_' lower(axes{j})]);
            units = 'Tesla';
        end
        
        % Compute Allan deviation
        [avar, tau] = allanvar(sensor_data, 'octave', fs);
        adev = sqrt(avar);
        
        % Plot Allan deviation
        loglog(tau, adev, 'LineWidth', 2);
        hold on;
        
        % Find and plot key points
        [minAdev, minIdx] = min(adev);
        biasInstability = minAdev / sqrt(2/pi);
        plot(tau(minIdx), minAdev, 'ro', 'MarkerSize', 10);
        text(tau(minIdx), minAdev, '  Bias Instability', 'VerticalAlignment', 'bottom');
        
        slopeNegHalf = log10(adev(2:end) ./ adev(1:end-1)) ./ log10(tau(2:end) ./ tau(1:end-1));
        [~, arhIdx] = min(abs(slopeNegHalf - (-0.5)));
        arw = adev(arhIdx) * 60 / sqrt(tau(arhIdx));
        plot(tau(arhIdx), adev(arhIdx), 'go', 'MarkerSize', 10);
        text(tau(arhIdx), adev(arhIdx), '  Angle Random Walk', 'VerticalAlignment', 'top');
        
        [~, rrwIdx] = min(abs(slopeNegHalf - 0.5));
        rrw = adev(rrwIdx) / 60 * sqrt(tau(rrwIdx));
        plot(tau(rrwIdx), adev(rrwIdx), 'bo', 'MarkerSize', 10);
        text(tau(rrwIdx), adev(rrwIdx), '  Rate Random Walk', 'VerticalAlignment', 'bottom');
        
        % Customize plot
        xlabel('Averaging Time \tau (s)', 'FontSize', 10);
        ylabel(['Allan Deviation (' units ')'], 'FontSize', 10);
        title([sensors{i} ' ' axes{j} '-axis'], 'FontSize', 12);
        grid on;
        
        % Display noise parameters
        disp([sensors{i} ' ' axes{j} '-axis:']);
        disp(['  Angle Random Walk: ' num2str(arw, '%.2e') ' ' units '/√Hz']);
        disp(['  Bias Instability: ' num2str(biasInstability, '%.2e') ' ' units]);
        disp(['  Rate Random Walk: ' num2str(rrw, '%.2e') ' ' units '/√s']);
    end
    
    % Save figure
    saveas(gcf, [sensors{i} '_allan_deviation.png']);
end