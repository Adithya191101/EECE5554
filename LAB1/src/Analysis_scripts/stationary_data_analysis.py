import os
import sqlite3
import numpy as np
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from gps_interfaces.msg import GPSmsg  # Adjust the import as per your message structure

# Function to read GPS data from SQLite database
def read_gps_data(db_file):
    # Connect to the SQLite database
    conn = sqlite3.connect(db_file)
    cursor = conn.cursor()

    # Query to select the relevant data from the '/gps' topic
    cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = (SELECT id FROM topics WHERE name = '/gps')")

    # Fetch all results
    data = cursor.fetchall()

    # Close the database connection
    conn.close()

    # Deserialize the messages
    gps_data = []
    for timestamp, blob in data:
        message = deserialize_message(blob, GPSmsg)  # Adjust this according to your message type
        gps_data.append((timestamp, message.latitude, message.longitude, message.utm_easting, message.utm_northing, message.altitude))

    return gps_data

# Function to compute various statistical metrics
def compute_statistics(values, reference_value=None):
    mean_value = np.mean(values)
    variance = np.var(values)
    std_dev = np.std(values)
    
    if reference_value is None:
        reference_value = mean_value  # Use the mean as the reference value

    rmse = np.sqrt(np.mean((values - reference_value) ** 2))
    bias = np.mean(values - reference_value)
    sampling_error = np.std(values) / np.sqrt(len(values))
    mae = np.mean(np.abs(values - reference_value))
    mse = np.mean((values - reference_value) ** 2)
    
    return {
        'mean': mean_value,
        'variance': variance,
        'std_dev': std_dev,
        'rmse': rmse,
        'bias': bias,
        'sampling_error': sampling_error,
        'mae': mae,
        'mse': mse
    }

# Function to plot GPS data and include UTM graph along with error analysis
def plot_gps_data_with_analysis(data, plot_type='Stationary'):
    # Convert data to numpy arrays for easier manipulation
    timestamps, latitudes, longitudes, utm_easting, utm_northing, altitudes = zip(*data)
    timestamps = np.array(timestamps)
    latitudes = np.array(latitudes)
    longitudes = np.array(longitudes)
    utm_easting = np.array(utm_easting)
    utm_northing = np.array(utm_northing)
    altitudes = np.array(altitudes)

    # Compute and print error statistics for UTM Easting and UTM Northing
    easting_stats = compute_statistics(utm_easting)
    northing_stats = compute_statistics(utm_northing)
    
    print("\n### Error Statistics (UTM Easting) ###")
    for key, value in easting_stats.items():
        print(f"{key.capitalize()}: {value}")

    print("\n### Error Statistics (UTM Northing) ###")
    for key, value in northing_stats.items():
        print(f"{key.capitalize()}: {value}")

    # Create subplots for Latitude, Longitude, and Altitude vs Time
    fig, ax = plt.subplots(3, 1, figsize=(12, 8))

    # Plot Latitude vs Time
    ax[0].plot(timestamps, latitudes, color='blue', label='Latitude')
    ax[0].set_title(f'{plot_type.capitalize()} Data: Latitude vs Time')
    ax[0].set_xlabel('Time (s)')
    ax[0].set_ylabel('Latitude')
    ax[0].grid(True)

    # Plot Longitude vs Time
    ax[1].plot(timestamps, longitudes, color='green', label='Longitude')
    ax[1].set_title(f'{plot_type.capitalize()} Data: Longitude vs Time')
    ax[1].set_xlabel('Time (s)')
    ax[1].set_ylabel('Longitude')
    ax[1].grid(True)

    # Plot Altitude vs Time
    ax[2].plot(timestamps, altitudes, color='red', label='Altitude')
    ax[2].set_title(f'{plot_type.capitalize()} Data: Altitude vs Time')
    ax[2].set_xlabel('Time (s)')
    ax[2].set_ylabel('Altitude (m)')
    ax[2].grid(True)

    # Adjust layout for better visualization
    plt.tight_layout()
    plt.show()

    # Plotting UTM Easting vs UTM Northing (2D plot of the position)
    plt.figure(figsize=(10, 6))
    plt.scatter(utm_easting, utm_northing, c='red', marker='o', label='UTM Coordinates')
    plt.title(f'{plot_type.capitalize()} UTM Data: Easting vs Northing')
    plt.xlabel('UTM Easting (m)')
    plt.ylabel('UTM Northing (m)')
    plt.grid()
    plt.legend()
    plt.show()

if __name__ == "__main__":
    # Set the correct path for the database file
    db_path = "/home/adithya/lab1_ws/src/bag files/stationary_data/stationary_data_0.db3"  # Update with your path
    if not os.path.exists(db_path):
        print(f"Database file not found at {db_path}")
    else:
        gps_data = read_gps_data(db_path)
        if gps_data:
            print(f"Retrieved {len(gps_data)} GPS messages.")
            plot_gps_data_with_analysis(gps_data, plot_type='Stationary')
        else:
            print("No GPS data found.")
