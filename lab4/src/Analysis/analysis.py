import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import integrate

data = pd.read_csv('/home/iftekhar/catkin_ws/src/lab4/Data/circle2_3.csv')

# Extract relevant data
time = data['Time']
mag_x = data['mag_field.magnetic_field.x']
mag_y = data['mag_field.magnetic_field.y']
mag_z = data['mag_field.magnetic_field.z']
acc_x = data['imu.linear_acceleration.x']
acc_y = data['imu.linear_acceleration.y']
acc_z = data['imu.linear_acceleration.z']
gyro_x = data['imu.angular_velocity.x']
gyro_y = data['imu.angular_velocity.y']
gyro_z = data['imu.angular_velocity.z']
orientation_x= data['imu.orientation.x']
orientation_y= data['imu.orientation.y']
orientation_z= data['imu.orientation.z']

dt = np.mean(np.diff(time))

# -----------------------------
# Figure 1: Magnetometer Calibration
# -----------------------------

# Plot before calibration
plt.figure(figsize=(12, 6))
plt.subplot(1, 2, 1)
plt.scatter(mag_x, mag_y, s=1)
plt.title('Before Calibration')
plt.xlabel('MagX')
plt.ylabel('MagY')
plt.axis('equal')

mag_x_offset = (mag_x.max() + mag_x.min()) / 2
mag_y_offset = (mag_y.max() + mag_y.min()) / 2

mag_x_scale = (mag_x.max() - mag_x.min()) / 2
mag_y_scale = (mag_y.max() - mag_y.min()) / 2

mag_x_calibrated = (mag_x - mag_x_offset) / mag_x_scale
mag_y_calibrated = (mag_y - mag_y_offset) / mag_y_scale

# Plot after calibration
plt.subplot(1, 2, 2)
plt.scatter(mag_x_calibrated, mag_y_calibrated, s=1, color='red')
plt.title('After Calibration')
plt.xlabel('MagX Calibrated')
plt.ylabel('MagY Calibrated')
plt.axis('equal')

plt.tight_layout()
plt.show()

# -----------------------------
# Figures 2-4: Gyroscope Data Analysis
# -----------------------------

# Figure 2: Rotational Rate (Gyroscope Data)
plt.figure(figsize=(10, 8))

plt.subplot(3, 1, 1)
plt.plot(time, gyro_x)
plt.title('Gyroscope X-axis Rotation Rate')
plt.xlabel('Time [s]')
plt.ylabel('Angular Velocity [deg/s]')

plt.subplot(3, 1, 2)
plt.plot(time, gyro_y)
plt.title('Gyroscope Y-axis Rotation Rate')
plt.xlabel('Time [s]')
plt.ylabel('Angular Velocity [deg/s]')

plt.subplot(3, 1, 3)
plt.plot(time, gyro_z)
plt.title('Gyroscope Z-axis Rotation Rate')
plt.xlabel('Time [s]')
plt.ylabel('Angular Velocity [deg/s]')

plt.tight_layout()
plt.show()

# Figure 3: Total Rotation (Integrated Gyroscope Data)
orientation_x = integrate.cumtrapz(gyro_x, time, initial=0)
orientation_y = integrate.cumtrapz(gyro_y, time, initial=0)
orientation_z = integrate.cumtrapz(gyro_z, time, initial=0)

plt.figure(figsize=(10, 8))

plt.subplot(3, 1, 1)
plt.plot(time, orientation_x)
plt.title('Integrated Gyroscope X-axis Orientation')
plt.xlabel('Time [s]')
plt.ylabel('Angle [deg]')

plt.subplot(3, 1, 2)
plt.plot(time, orientation_y)
plt.title('Integrated Gyroscope Y-axis Orientation')
plt.xlabel('Time [s]')
plt.ylabel('Angle [deg]')

plt.subplot(3, 1, 3)
plt.plot(time, orientation_z)
plt.title('Integrated Gyroscope Z-axis Orientation')
plt.xlabel('Time [s]')
plt.ylabel('Angle [deg]')

plt.tight_layout()
plt.show()

# Figure 4: Magnetometer X, Y over Time
plt.figure(figsize=(10, 8))

plt.subplot(2, 1, 1)
plt.plot(time, mag_x_calibrated)
plt.title('Calibrated Magnetometer X over Time')
plt.xlabel('Time [s]')
plt.ylabel('MagX Calibrated')

plt.subplot(2, 1, 2)
plt.plot(time, mag_y_calibrated)
plt.title('Calibrated Magnetometer Y over Time')
plt.xlabel('Time [s]')
plt.ylabel('MagY Calibrated')

plt.tight_layout()
plt.show()

# -----------------------------
# Figures 5-7: Accelerometer Data Analysis
# -----------------------------

# Figure 5: Acceleration Data
plt.figure(figsize=(10, 8))

plt.subplot(3, 1, 1)
plt.plot(time, acc_x)
plt.title('Accelerometer X-axis Acceleration')
plt.xlabel('Time [s]')
plt.ylabel('Acceleration [m/s²]')

plt.subplot(3, 1, 2)
plt.plot(time, acc_y)
plt.title('Accelerometer Y-axis Acceleration')
plt.xlabel('Time [s]')
plt.ylabel('Acceleration [m/s²]')

plt.subplot(3, 1, 3)
plt.plot(time, acc_z)
plt.title('Accelerometer Z-axis Acceleration')
plt.xlabel('Time [s]')
plt.ylabel('Acceleration [m/s²]')

plt.tight_layout()
plt.show()

# Removing gravity from Z-axis acceleration (assuming stationary at start)
acc_z_corrected = acc_z - np.mean(acc_z[:100])  

# Figure 6: Velocity (Integrated Acceleration)
velocity_x = integrate.cumtrapz(acc_x, time, initial=0)
velocity_y = integrate.cumtrapz(acc_y, time, initial=0)
velocity_z = integrate.cumtrapz(acc_z_corrected, time, initial=0)

plt.figure(figsize=(10, 8))

plt.subplot(3, 1, 1)
plt.plot(time, velocity_x)
plt.title('Velocity X-axis')
plt.xlabel('Time [s]')
plt.ylabel('Velocity [m/s]')

plt.subplot(3, 1, 2)
plt.plot(time, velocity_y)
plt.title('Velocity Y-axis')
plt.xlabel('Time [s]')
plt.ylabel('Velocity [m/s]')

plt.subplot(3, 1, 3)
plt.plot(time, velocity_z)
plt.title('Velocity Z-axis')
plt.xlabel('Time [s]')
plt.ylabel('Velocity [m/s]')

plt.tight_layout()
plt.show()

# Figure 7: Displacement (Integrated Velocity)
displacement_x = integrate.cumtrapz(velocity_x, time, initial=0)
displacement_y = integrate.cumtrapz(velocity_y, time, initial=0)
displacement_z = integrate.cumtrapz(velocity_z, time, initial=0)

plt.figure(figsize=(10, 8))

plt.subplot(3, 1, 1)
plt.plot(time, displacement_x)
plt.title('Displacement X-axis')
plt.xlabel('Time [s]')
plt.ylabel('Displacement [m]')

plt.subplot(3, 1, 2)
plt.plot(time, displacement_y)
plt.title('Displacement Y-axis')
plt.xlabel('Time [s]')
plt.ylabel('Displacement [m]')

plt.subplot(3, 1, 3)
plt.plot(time, displacement_z)
plt.title('Displacement Z-axis')
plt.xlabel('Time [s]')
plt.ylabel('Displacement [m]')

plt.tight_layout()
plt.show()
