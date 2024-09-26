# Lab 1: GPS Data Analysis

This project involves reading, analyzing, and visualizing GPS data captured from a GPGGA puck and stored in SQLite database files (`.db3`). The data is processed to compute statistical metrics such as Root Mean Square Error (RMSE), variance, mean, bias, and others. The project uses ROS2 to handle GPS message deserialization and Python libraries such as `matplotlib` for plotting.

## How to Run

step1: clone the repo

```jsx
git clone git@gitlab.com:rajendran.ad/EECE5554.git
cd lab1_ws
```

step 2: `colcon build` the project

step 3 : to launch the driver:

```jsx
ros2 launch gps_driver gps_launch.py 

```

step 2 : Ensure that you have installed the required Python dependencies:

```jsx
pip install matplotlib numpy
```

for running the GPS analysis script: for stationary data

```jsx
python3 src/Analysis_scripts/stationary_data_analysis.py
```

for walking data:

```jsx
python3 src/Analysis_scripts/walking_data_analysis.py
```

inside the data folder **stationary_data** contains the stationary bag file and **walking1_data** contains the walking  bag data when walking in-straight line and the **walking_data** contains the walking bag data in an dynamic environment.

also the .**csv** and .**txt** files are provided in data folder.

**Report.pdf** contains the analysis of the data project.