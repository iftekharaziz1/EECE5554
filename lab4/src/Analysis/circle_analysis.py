import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import cumtrapz

# Load the circle dataset
data = pd.read_csv('/home/iftekhar/EECE5554/lab4/src/Data/circle2_5.csv')

# Extract time and sensor data
time = data['Time']
acc_x = data['imu.linear_acceleration.x']
acc_y = data['imu.linear_acceleration.y']
acc_z = data['imu.linear_acceleration.z']
gyro_x = data['imu.angular_velocity.x']
gyro_y = data['imu.angular_velocity.y']
gyro_z = data['imu.angular_velocity.z']
mag_x = data['mag_field.magnetic_field.x']
mag_y = data['mag_field.magnetic_field.y']
mag_z = data['mag_field.magnetic_field.z']

# -----------------------------
# Figure 1: Magnetometer N vs. E Components Before and After Calibration
# -----------------------------

# Plot before calibration
plt.figure(figsize=(12, 6))
plt.subplot(1, 2, 1)
plt.scatter(mag_x, mag_y, s=1, color='blue')
plt.title('Figure 1a: Magnetometer N vs E (Before Calibration)')
plt.xlabel('Magnetic Field X (North, µT)')
plt.ylabel('Magnetic Field Y (East, µT)')
plt.axis('equal')

# Magnetometer calibration (simple offset and scaling)
offset_x = mag_x.mean()
offset_y = mag_y.mean()
offset_z = mag_z.mean()
scale_x = (mag_x.max() - mag_x.min()) / 2
scale_y = (mag_y.max() - mag_y.min()) / 2
scale_z = (mag_z.max() - mag_z.min()) / 2

# Apply calibration
mag_x_cal = (mag_x - offset_x) / scale_x
mag_y_cal = (mag_y - offset_y) / scale_y
mag_z_cal = (mag_z - offset_z) / scale_z

# Plot after calibration
plt.subplot(1, 2, 2)
plt.scatter(mag_x_cal, mag_y_cal, s=1, color='red')
plt.title('Figure 1b: Magnetometer N vs E (After Calibration)')
plt.xlabel('Magnetic Field X (North, µT)')
plt.ylabel('Magnetic Field Y (East, µT)')
plt.axis('equal')

plt.tight_layout()
plt.show()

# -----------------------------
# Figure 2: Gyroscope Rotational Rate vs. Time
# -----------------------------

# Plot rotational rates
plt.figure(figsize=(10, 8))

plt.subplot(3, 1, 1)
plt.plot(time, gyro_x, label='Gyro X')
plt.title('Figure 2: Gyroscope Rotational Rate - X Axis')
plt.ylabel('Angular Velocity (rad/s)')

plt.subplot(3, 1, 2)
plt.plot(time, gyro_y, label='Gyro Y', color='orange')
plt.title('Gyroscope Rotational Rate - Y Axis')
plt.ylabel('Angular Velocity (rad/s)')

plt.subplot(3, 1, 3)
plt.plot(time, gyro_z, label='Gyro Z', color='green')
plt.title('Gyroscope Rotational Rate - Z Axis')
plt.xlabel('Time (samples)')
plt.ylabel('Angular Velocity (rad/s)')

plt.tight_layout()
plt.show()

# -----------------------------
# Figure 3: Total Rotation (Integrated Gyroscope Data) vs. Time
# -----------------------------

# Integrate gyroscope data to get total rotation
angle_x = cumtrapz(gyro_x, time, initial=0)
angle_y = cumtrapz(gyro_y, time, initial=0)
angle_z = cumtrapz(gyro_z, time, initial=0)

# Plot total rotation
plt.figure(figsize=(10, 8))

plt.subplot(3, 1, 1)
plt.plot(time, angle_x, label='Angle X')
plt.title('Figure 3: Total Rotation - X Axis')
plt.ylabel('Angle (rad)')

plt.subplot(3, 1, 2)
plt.plot(time, angle_y, label='Angle Y', color='orange')
plt.title('Total Rotation - Y Axis')
plt.ylabel('Angle (rad)')

plt.subplot(3, 1, 3)
plt.plot(time, angle_z, label='Angle Z', color='green')
plt.title('Total Rotation - Z Axis')
plt.xlabel('Time (samples)')
plt.ylabel('Angle (rad)')

plt.tight_layout()
plt.show()

# -----------------------------
# Figure 4: Magnetometer X and Y Components vs. Time
# -----------------------------

# Plot magnetometer components
plt.figure(figsize=(10, 6))

plt.subplot(2, 1, 1)
plt.plot(time, mag_x)
plt.title('Figure 4a: Magnetometer X Component vs. Time')
plt.ylabel('Magnetic Field X (µT)')

plt.subplot(2, 1, 2)
plt.plot(time, mag_y, color='orange')
plt.title('Figure 4b: Magnetometer Y Component vs. Time')
plt.xlabel('Time (samples)')
plt.ylabel('Magnetic Field Y (µT)')

plt.tight_layout()
plt.show()

# -----------------------------
# Figure 5: Acceleration vs. Time
# -----------------------------

# Plot acceleration
plt.figure(figsize=(10, 8))

plt.subplot(3, 1, 1)
plt.plot(time, acc_x)
plt.title('Figure 5: Acceleration - X Axis')
plt.ylabel('Acceleration (m/s²)')

plt.subplot(3, 1, 2)
plt.plot(time, acc_y, color='orange')
plt.title('Acceleration - Y Axis')
plt.ylabel('Acceleration (m/s²)')

plt.subplot(3, 1, 3)
plt.plot(time, acc_z, color='green')
plt.title('Acceleration - Z Axis')
plt.xlabel('Time (samples)')
plt.ylabel('Acceleration (m/s²)')

plt.tight_layout()
plt.show()

# -----------------------------
# Figure 6: Velocity vs. Time
# -----------------------------

# Integrate acceleration to get velocity
vel_x = cumtrapz(acc_x, time, initial=0)
vel_y = cumtrapz(acc_y, time, initial=0)
vel_z = cumtrapz(acc_z, time, initial=0)

# Plot velocity
plt.figure(figsize=(10, 8))

plt.subplot(3, 1, 1)
plt.plot(time, vel_x)
plt.title('Figure 6: Velocity - X Axis')
plt.ylabel('Velocity (m/s)')

plt.subplot(3, 1, 2)
plt.plot(time, vel_y, color='orange')
plt.title('Velocity - Y Axis')
plt.ylabel('Velocity (m/s)')

plt.subplot(3, 1, 3)
plt.plot(time, vel_z, color='green')
plt.title('Velocity - Z Axis')
plt.xlabel('Time (samples)')
plt.ylabel('Velocity (m/s)')

plt.tight_layout()
plt.show()

# -----------------------------
# Figure 7: Displacement vs. Time
# -----------------------------

# Integrate velocity to get displacement
disp_x = cumtrapz(vel_x, time, initial=0)
disp_y = cumtrapz(vel_y, time, initial=0)
disp_z = cumtrapz(vel_z, time, initial=0)

# Plot displacement
plt.figure(figsize=(10, 8))

plt.subplot(3, 1, 1)
plt.plot(time, disp_x)
plt.title('Figure 7: Displacement - X Axis')
plt.ylabel('Displacement (m)')

plt.subplot(3, 1, 2)
plt.plot(time, disp_y, color='orange')
plt.title('Displacement - Y Axis')
plt.ylabel('Displacement (m)')

plt.subplot(3, 1, 3)
plt.plot(time, disp_z, color='green')
plt.title('Displacement - Z Axis')
plt.xlabel('Time (samples)')
plt.ylabel('Displacement (m)')

plt.tight_layout()
plt.show()
