# Lab 3: IMU Noise Characterization with Allan Variance

## Note on Magnetometer Data Collection

Initially, the magnetometer readings were collected in Gauss. Upon further analysis, it was identified that the readings should be converted to Tesla for proper analysis and standardization.

The data submitted for the Allan Variance analysis was converted from Gauss to Tesla post-collection. I have since updated the IMU driver to ensure that all future magnetometer readings are collected directly in Tesla, in compliance with standard unit conventions.

Apologies for any confusion caused by this initial oversight. The data provided in this report accurately reflects the required conversions for the analysis.