import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, sosfilt

# -------------------------------------------------------------
# Helper Functions
# -------------------------------------------------------------

def butter_filter(raw_data, cutoff_freq, sampl_freq, filt_type, filt_order):
    """
    Creates and applies a Butterworth filter (low/high pass) using second-order sections.
    """
    nyq_freq = sampl_freq / 2.0
    sos = butter(N=filt_order, Wn=cutoff_freq / nyq_freq, btype=filt_type, analog=False, output='sos')
    filtered_data = sosfilt(sos, raw_data)
    return sos, filtered_data

def hard_soft_iron_correction(mx, my, mz, center, transformation):
    """
    Apply hard-iron and soft-iron correction.
    """
    m = np.vstack((mx, my, mz))
    m_corrected = transformation @ (m.T - center).T
    return m_corrected[0, :], m_corrected[1, :], m_corrected[2, :]

def compute_yaw_from_mag(mx, my):
    # Yaw from magnetometer: yaw = arctan2(my, mx)
    return np.arctan2(my, mx)

def integrate_gyro_yaw(gyro_z, dt):
    # Integrate the yaw rate from gyro over time
    return np.cumsum(gyro_z * dt)

def compute_forward_velocity_from_acc(acc, dt):
    # Integrate linear acceleration to get velocity
    return np.cumsum(acc * dt)

def latlon_to_xy(lat, lon, lat0, lon0):
    # Simple flat-earth approximation (for demonstration)
    R = 6371000.0  # Earth radius in meters
    lat_rad = np.radians(lat)
    lon_rad = np.radians(lon)
    lat0_rad = np.radians(lat0)
    lon0_rad = np.radians(lon0)
    x = (lon_rad - lon0_rad) * R * np.cos(lat0_rad)
    y = (lat_rad - lat0_rad) * R
    return x, y

# -------------------------------------------------------------
# Load Data
# -------------------------------------------------------------
gps_data = pd.read_csv('/home/iftekhar/Downloads/RSN/Lab5/circle_calibration/gps_receiver.csv')
imu_data = pd.read_csv('/home/iftekhar/Downloads/RSN/Lab5/circle_calibration/imu.csv')

# gps_receiver.csv columns: 
# Time, latitude, longitude, altitude, utm_easting, utm_northing, letter, hdop, gpgga_read
time_gps = gps_data['Time'].values
lat = gps_data['latitude'].values
lon = gps_data['longitude'].values
# If needed, you can use utm_easting/utm_northing directly for trajectory
utm_easting = gps_data['utm_easting'].values
utm_northing = gps_data['utm_northing'].values

# imu.csv columns:
# Time, imu.header.seq, imu.header.stamp.secs, imu.header.stamp.nsecs, imu.header.frame_id,
# imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w, imu.orientation_covariance,
# imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z, imu.angular_velocity_covariance,
# imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z, imu.linear_acceleration_covariance,
# mag_field.header.seq, mag_field.header.stamp.secs, mag_field.header.stamp.nsecs, mag_field.header.frame_id,
# mag_field.magnetic_field.x, mag_field.magnetic_field.y, mag_field.magnetic_field.z, mag_field.magnetic_field_covariance,
# VNYMR

time_imu = imu_data['Time'].values
gyro_z = imu_data['imu.angular_velocity.z'].values
acc_x = imu_data['imu.linear_acceleration.x'].values
mag_x = imu_data['mag_field.magnetic_field.x'].values
mag_y = imu_data['mag_field.magnetic_field.y'].values
mag_z = imu_data['mag_field.magnetic_field.z'].values

dt_imu = np.mean(np.diff(time_imu))
fs_imu = 1.0/dt_imu

# -------------------------------------------------------------
# Magnetometer Calibration
# -------------------------------------------------------------
# Dummy calibration parameters - replace with your actual calibration results.
center = np.array([np.mean(mag_x), np.mean(mag_y), np.mean(mag_z)])
transformation = np.eye(3)  # If you have soft-iron correction matrix, put it here

mx_corrected, my_corrected, mz_corrected = hard_soft_iron_correction(mag_x, mag_y, mag_z, center, transformation)

# Yaw from magnetometer before and after calibration
yaw_mag_raw = compute_yaw_from_mag(mag_x, mag_y)
yaw_mag_corrected = compute_yaw_from_mag(mx_corrected, my_corrected)

# Gyro yaw (integrated)
yaw_gyro = integrate_gyro_yaw(gyro_z, dt_imu)

# -------------------------------------------------------------
# Complementary Filtering
# -------------------------------------------------------------
low_cutoff = 0.1  # Hz for low-pass on mag yaw
high_cutoff = 0.1 # Hz for high-pass on gyro yaw

_, yaw_mag_lp = butter_filter(yaw_mag_corrected, cutoff_freq=low_cutoff, sampl_freq=fs_imu, 
                              filt_type='lowpass', filt_order=2)
_, yaw_gyro_hp = butter_filter(yaw_gyro, cutoff_freq=high_cutoff, sampl_freq=fs_imu, 
                               filt_type='highpass', filt_order=2)

alpha = 0.98
yaw_complementary = alpha * yaw_gyro_hp + (1 - alpha) * yaw_mag_lp
imu_heading = yaw_complementary

# -------------------------------------------------------------
# Forward Velocity from Accelerometer
# -------------------------------------------------------------
vel_imu_raw = compute_forward_velocity_from_acc(acc_x, dt_imu)
acc_bias = np.mean(acc_x[0:100]) # estimate bias from first 100 samples (adjust as needed)
acc_x_adjusted = acc_x - acc_bias
vel_imu_adjusted = compute_forward_velocity_from_acc(acc_x_adjusted, dt_imu)

# GPS velocity is not directly given in the columns you provided.
# If needed, compute GPS velocity from consecutive positions and time
# For demonstration, we can approximate using consecutive differences:
# Convert lat/lon to local XY:
lat0, lon0 = lat[0], lon[0]
gps_x, gps_y = latlon_to_xy(lat, lon, lat0, lon0)

gps_dt = np.diff(time_gps)
gps_vel = np.zeros_like(gps_x)
gps_vel[1:] = np.sqrt(np.diff(gps_x)**2 + np.diff(gps_y)**2) / gps_dt

# -------------------------------------------------------------
# IMU Trajectory
# -------------------------------------------------------------
imu_x = np.cumsum(vel_imu_adjusted * np.cos(imu_heading) * dt_imu)
imu_y = np.cumsum(vel_imu_adjusted * np.sin(imu_heading) * dt_imu)

# -------------------------------------------------------------
# Plotting Results
# -------------------------------------------------------------
# -----------------------------
#  Figure 0: Magnetometer (Before and After Calibration)
# -----------------------------

# Plot before calibration
plt.figure(figsize=(12, 6))
plt.subplot(1, 2, 1)
plt.scatter(mag_x, mag_y, s=1, color='blue')
plt.title('Figure 1a: Magnetometer (Before Calibration)')
plt.xlabel('Magnetic Field X')
plt.ylabel('Magnetic Field Y')
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
plt.title('Figure 1b: Magnetometer (After Calibration)')
plt.xlabel('Magnetic Field X ')
plt.ylabel('Magnetic Field Y ')
plt.axis('equal')

plt.tight_layout()
plt.show()


# 1. Magnetometer yaw estimation before and after calibration
plt.figure()
plt.title('Magnetometer Yaw Before and After Calibration')
plt.plot(time_imu, yaw_mag_raw, label='Raw Mag Yaw')
plt.plot(time_imu, yaw_mag_corrected, label='Corrected Mag Yaw')
plt.xlabel('Time [s]')
plt.ylabel('Yaw [rad]')
plt.legend()
plt.show()

# 2. Gyro yaw estimation vs time
plt.figure()
plt.title('Gyro Yaw Estimation')
plt.plot(time_imu, yaw_gyro, label='Gyro Integrated Yaw')
plt.xlabel('Time [s]')
plt.ylabel('Yaw [rad]')
plt.legend()
plt.show()

# 3. Complementary filter subplots
plt.figure()
plt.subplot(4,1,1)
plt.title('Mag Yaw (Low-Pass Filtered)')
plt.plot(time_imu, yaw_mag_lp)
plt.ylabel('Yaw [rad]')

plt.subplot(4,1,2)
plt.title('Gyro Yaw (High-Pass Filtered)')
plt.plot(time_imu, yaw_gyro_hp)
plt.ylabel('Yaw [rad]')

plt.subplot(4,1,3)
plt.title('Complementary Filter Output')
plt.plot(time_imu, yaw_complementary)
plt.ylabel('Yaw [rad]')

plt.subplot(4,1,4)
plt.title('IMU Heading Estimate')
plt.plot(time_imu, imu_heading)
plt.xlabel('Time [s]')
plt.ylabel('Yaw [rad]')
plt.tight_layout()
plt.show()

# 4. Forward velocity from accelerometer before and after adjustments
plt.figure()
plt.title('Forward Velocity from Accelerometer')
plt.plot(time_imu, vel_imu_raw, label='Raw Velocity')
plt.plot(time_imu, vel_imu_adjusted, label='Adjusted Velocity')
plt.xlabel('Time [s]')
plt.ylabel('Velocity [m/s]')
plt.legend()
plt.grid(True)
plt.show()

# 5. Forward velocity from GPS (computed from position differences)
plt.figure()
plt.title('Forward Velocity from GPS')
plt.plot(time_gps, np.hstack([0, gps_vel[1:]]), label='GPS Velocity') # add zero or handle indexing carefully
plt.xlabel('Time [s]')
plt.ylabel('Velocity [m/s]')
plt.legend()
plt.grid(True)
plt.show()

#6 is wrong code. Ignore it. Check plot7-GPS IMU.py for correct code.
# 6. Estimated trajectory from GPS and IMU. 
plt.figure()
plt.subplot(2,1,1)
plt.title('GPS Estimated Trajectory')
plt.plot(gps_x, gps_y, label='GPS Trajectory')
plt.axis('equal')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.legend()

plt.subplot(2,1,2)
plt.title('IMU Estimated Trajectory')
plt.plot(imu_x, imu_y, label='IMU Trajectory', color='r')
plt.axis('equal')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.legend()
plt.tight_layout()
plt.show()
