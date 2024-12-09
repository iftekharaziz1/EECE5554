import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

# Load data
imu = pd.read_csv('/home/iftekhar/Downloads/RSN/Lab5/DATA_driving/imu.csv')
gps = pd.read_csv('/home/iftekhar/Downloads/RSN/Lab5/DATA_driving/gps_receiver.csv')

# Extract and preprocess data
imu['time'] = imu['Time']
gps['time'] = gps['Time']

# Convert quaternion to yaw (heading)
def quaternion_to_yaw(row):
    x, y, z, w = row['imu.orientation.x'], row['imu.orientation.y'], row['imu.orientation.z'], row['imu.orientation.w']
    r = R.from_quat([x, y, z, w])
    return r.as_euler('zyx', degrees=False)[0]

# Apply the function row-wise
imu['yaw'] = imu.apply(quaternion_to_yaw, axis=1)

# Integrate accelerations to compute forward velocity
imu['accel_x'] = imu['imu.linear_acceleration.x']  # Acceleration in vehicle frame
imu['yaw'] = imu['yaw'].fillna(method='ffill')  # Fill missing yaw values if needed
imu['dt'] = imu['time'].diff().fillna(0)

imu['velocity_x'] = np.cumsum(imu['accel_x'] * imu['dt'])  # Integrate acceleration

# Compute easting and northing velocities (v_e, v_n)
imu['v_e'] = imu['velocity_x'] * np.cos(imu['yaw'])
imu['v_n'] = imu['velocity_x'] * np.sin(imu['yaw'])

# Integrate velocities to compute positions (x_e, x_n)
imu['x_e'] = np.cumsum(imu['v_e'] * imu['dt'])
imu['x_n'] = np.cumsum(imu['v_n'] * imu['dt'])

# Align GPS data to start at the same origin
gps['utm_easting'] -= gps['utm_easting'].iloc[0]
gps['utm_northing'] -= gps['utm_northing'].iloc[0]
imu['x_e'] -= imu['x_e'].iloc[0]
imu['x_n'] -= imu['x_n'].iloc[0]

# Scale IMU trajectory to align with GPS
scale_factor = gps['utm_easting'].iloc[-1] / imu['x_e'].iloc[-1]
imu['x_e'] *= scale_factor
imu['x_n'] *= scale_factor

# Plotting
plt.figure(figsize=(12, 8))

# Subplot 1: GPS Trajectory
plt.subplot(2, 1, 1)
plt.plot(gps['utm_easting'], gps['utm_northing'], label='GPS Trajectory', color='blue')
plt.title('GPS Trajectory')
plt.xlabel('Easting (m)')
plt.ylabel('Northing (m)')
plt.grid()
plt.legend()

# Subplot 2: IMU Trajectory
plt.subplot(2, 1, 2)
plt.plot(imu['x_e'], imu['x_n'], label='IMU Trajectory', color='orange')
plt.title('IMU Trajectory (Scaled)')
plt.xlabel('Easting (m)')
plt.ylabel('Northing (m)')
plt.grid()
plt.legend()

plt.tight_layout()
plt.show()
