import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import norm

def load_data(file_path):
    df = pd.read_csv(file_path)
    df['timestamp'] = pd.to_datetime(df['timestamp'], unit='s')
    df.set_index('timestamp', inplace=True)
    return df

def plot_time_series(df, columns, title):
    plt.figure(figsize=(12, 6))
    for column in columns:
        plt.plot(df.index.to_numpy(), df[column].to_numpy(), label=column)
    plt.title(title)
    plt.xlabel('Time')
    plt.ylabel('Value')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(f"{title.replace(' ', '_')}.png")
    plt.show()

def plot_histogram(df, column, title):
    plt.figure(figsize=(10, 6))
    df[column].hist(bins=50, density=True, alpha=0.7, color='b')
    
    mu, std = norm.fit(df[column])
    xmin, xmax = plt.xlim()
    x = np.linspace(xmin, xmax, 100)
    p = norm.pdf(x, mu, std)
    plt.plot(x, p, 'k', linewidth=2)
    
    plt.title(f"Histogram of {title}")
    plt.xlabel('Value')
    plt.ylabel('Frequency')
    plt.text(0.05, 0.95, f'μ = {mu:.6f}\nσ = {std:.6f}', 
             transform=plt.gca().transAxes, verticalalignment='top')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(f"Histogram_{title.replace(' ', '_')}.png")
    plt.show()

def main():
    df = load_data('imu_data.csv')
    
    # Accelerometer analysis
    plot_time_series(df, ['linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z'], 
                     'Accelerometer Data')
    for axis in ['x', 'y', 'z']:
        plot_histogram(df, f'linear_acceleration_{axis}', f'Accelerometer {axis.upper()}-axis')

    # Gyroscope analysis
    plot_time_series(df, ['angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z'], 
                     'Gyroscope Data')
    for axis in ['x', 'y', 'z']:
        plot_histogram(df, f'angular_velocity_{axis}', f'Gyroscope {axis.upper()}-axis')

    # Magnetometer analysis
    plot_time_series(df, ['magnetic_field_x', 'magnetic_field_y', 'magnetic_field_z'], 
                     'Magnetometer Data')
    for axis in ['x', 'y', 'z']:
        plot_histogram(df, f'magnetic_field_{axis}', f'Magnetometer {axis.upper()}-axis')

    # Calculate and print statistics
    stats = df.describe()
    print("Data statistics:")
    print(stats)
    stats.to_csv('imu_15min_statistics.csv')

if __name__ == "__main__":
    main()