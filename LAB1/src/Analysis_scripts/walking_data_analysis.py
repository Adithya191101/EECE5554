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

# Function to plot UTM data for a single dataset along with error analysis
def plot_single_gps_data_with_analysis(data, file_label="Walking Data"):
    # Convert data to numpy arrays for easier manipulation
    timestamps, _, _, utm_easting, utm_northing, _ = zip(*data)

    utm_easting = np.array(utm_easting)
    utm_northing = np.array(utm_northing)

    # Compute error statistics for UTM Easting and UTM Northing
    easting_stats = compute_statistics(utm_easting)
    northing_stats = compute_statistics(utm_northing)
    
    print(f"\n### Error Statistics ({file_label} UTM Easting) ###")
    for key, value in easting_stats.items():
        print(f"{key.capitalize()}: {value}")

    print(f"\n### Error Statistics ({file_label} UTM Northing) ###")
    for key, value in northing_stats.items():
        print(f"{key.capitalize()}: {value}")

    # Plotting UTM Easting vs UTM Northing (2D plot of the position)
    plt.figure(figsize=(10, 6))
    plt.scatter(utm_easting, utm_northing, c='red', marker='o', label=f'{file_label} UTM Coordinates')
    plt.title(f'{file_label.capitalize()} UTM Data: Easting vs Northing')
    plt.xlabel('UTM Easting (m)')
    plt.ylabel('UTM Northing (m)')
    plt.grid()
    plt.legend()
    plt.show()

if __name__ == "__main__":
    # Define paths for the two database files
    db_path1 = "/home/adithya/lab1_ws/src/bag files/walking_data/walking_data_0.db3"  # Update with your actual path
    db_path2 = "/home/adithya/lab1_ws/src/bag files/walking1_data/walking1_data_0.db3"  # Update with your actual path

    if not os.path.exists(db_path1):
        print(f"Database file not found at {db_path1}")
    elif not os.path.exists(db_path2):
        print(f"Database file not found at {db_path2}")
    else:
        # Read and plot data for the first file
        gps_data1 = read_gps_data(db_path1)
        if gps_data1:
            print(f"Retrieved {len(gps_data1)} GPS messages from file 1.")
            plot_single_gps_data_with_analysis(gps_data1, file_label="Walking Data dynamic environment")
        else:
            print("No GPS data found in file 1.")

        # Read and plot data for the second file
        gps_data2 = read_gps_data(db_path2)
        if gps_data2:
            print(f"Retrieved {len(gps_data2)} GPS messages from file 2.")
            plot_single_gps_data_with_analysis(gps_data2, file_label="Walking Data in straaight line")
        else:
            print("No GPS data found in file 2.")
