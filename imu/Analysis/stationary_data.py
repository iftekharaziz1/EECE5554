import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

# Load the data
data = pd.read_csv('/home/iftekhar/catkin_ws/src/imu/src/Analysis/stationary_data.csv')

# Extract Time and sensor data
time = data.iloc[:, 0].values - data.iloc[0, 0]  # Start time at 0

qw = data.iloc[:, 9].values
qx = data.iloc[:, 10].values
qy = data.iloc[:, 11].values
qz = data.iloc[:, 12].values

gyro_x = data.iloc[:, 14].values
gyro_y = data.iloc[:, 15].values
gyro_z = data.iloc[:, 16].values

 
acc_x = data.iloc[:, 18].values
acc_y = data.iloc[:, 19].values
acc_z = data.iloc[:, 20].values

#Convert quaternions to Euler angles in deg
quaternion = np.column_stack((qw, qx, qy, qz))
r = R.from_quat(quaternion)
euler_angles = r.as_euler('xyz', degrees=True)  
eul_x, eul_y, eul_z = euler_angles[:, 0], euler_angles[:, 1], euler_angles[:, 2]

# Fig 1: Plot gyroscope Rotational rate in deg/s
plt.figure(figsize=(10, 6))
plt.subplot(3, 1, 1)
plt.plot(time, np.rad2deg(gyro_x), 'r', label='Gyro X')
plt.ylabel('Rotational rate (deg/s)')
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time, np.rad2deg(gyro_y), 'g', label='Gyro Y')
plt.ylabel('Rotational rate (deg/s)')
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time, np.rad2deg(gyro_z), 'b', label='Gyro Z')
plt.xlabel('Time (s)')
plt.ylabel('Rotational rate (deg/s)')
plt.legend()
plt.suptitle('Gyroscope Rotational Rate (Degrees/s)')
plt.tight_layout()
plt.show()

    # Fig 2: Plot Acceleration in m/s²
plt.figure(figsize=(10, 6))
plt.subplot(3, 1, 1)
plt.plot(time, acc_x, 'r', label='Acc X')
plt.ylabel('Acceleration m/s²')
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time, acc_y, 'g', label='Acc Y')
plt.ylabel('Acceleration m/s²')
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time, acc_z, 'b', label='Acc Z')
plt.xlabel('Time (s)')
plt.ylabel('m/s²')
plt.legend()
plt.suptitle('Acceleration (m/s²)')
plt.tight_layout()
plt.show()

    # Fig 3: Plot Rotation from Euler Angles in degrees
plt.figure(figsize=(10, 6))
plt.subplot(3, 1, 1)
plt.plot(time, eul_x, 'r', label='Roll (X)')
plt.ylabel('Degrees')
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time, eul_y, 'g', label='Pitch (Y)')
plt.ylabel('Degrees')
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time, eul_z, 'b', label='Yaw (Z)')
plt.xlabel('Time (s)')
plt.ylabel('Degrees')
plt.legend()
plt.suptitle('Rotation in Euler Angles (Degrees)')
plt.tight_layout()
plt.show()

    # Fig 4: Histograms of Rotation Angles
plt.figure(figsize=(15, 5))

plt.subplot(1, 3, 1)
plt.hist(eul_x, bins=30, color='red', alpha=0.7)
plt.xlabel('Roll (Degrees)')
plt.ylabel('Frequency')
plt.title('Histogram of Roll (X)')

plt.subplot(1, 3, 2)
plt.hist(eul_y, bins=30, color='green', alpha=0.7)
plt.xlabel('Pitch (Degrees)')
plt.title('Histogram of Pitch (Y)')

plt.subplot(1, 3, 3)
plt.hist(eul_z, bins=30, color='blue', alpha=0.7)
plt.xlabel('Yaw (Degrees)')
plt.title('Histogram of Yaw (Z)')

plt.suptitle('Histograms of Rotation Angles')
plt.tight_layout()
plt.show()

    

