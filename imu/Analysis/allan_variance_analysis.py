import pandas as pd
import numpy as np
import allantools as allan
import matplotlib.pyplot as plt

def check_VNYMR_in_string(s):
    s = s.strip()
    return s.startswith('$VNYMR') or s == '$VNYMR'

# Load data
data_frame = pd.read_csv("path_to_your_csv_file.csv")
data_list = data_frame["data"].tolist()
time_list = data_frame["Time"].tolist()

time_array = np.array(time_list)
delta_t = np.mean(np.diff(time_array))  # Mean time difference between samples
sampling_rate = 1.0 / delta_t           # Sampling frequency

gyroscope_x, gyroscope_y, gyroscope_z = [], [], []
accelerometer_x, accelerometer_y, accelerometer_z = [], [], []

for i in range(len(data_list)):
    line = data_list[i].split(',')

    if not check_VNYMR_in_string(str(line[0])):
        continue

    current_time = time_array[i]

    angular_rate_x = float(line[10])
    angular_rate_y = float(line[11])
    gyro_z_val = str(line[12].split('*')[0])

    try:
        angular_rate_z = float(gyro_z_val)
    except ValueError:
        angular_rate_z = 0

    gyroscope_x.append(angular_rate_x)
    gyroscope_y.append(angular_rate_y)
    gyroscope_z.append(angular_rate_z)

    accelerometer_x.append(float(line[7]))
    accelerometer_y.append(float(line[8]))
    accelerometer_z.append(float(line[9]))

gyroscope_x_array = np.array(gyroscope_x)
gyroscope_y_array = np.array(gyroscope_y)
gyroscope_z_array = np.array(gyroscope_z)
accelerometer_x_array = np.array(accelerometer_x)
accelerometer_y_array = np.array(accelerometer_y)
accelerometer_z_array = np.array(accelerometer_z)

angles_x = np.cumsum(gyroscope_x_array) * delta_t  # [rad]
angles_y = np.cumsum(gyroscope_y_array) * delta_t  # [rad]
angles_z = np.cumsum(gyroscope_z_array) * delta_t  # [rad]

taus_x, ad_x, _, _ = allan.oadev(angles_x, rate=sampling_rate, data_type="phase", taus="all")
taus_y, ad_y, _, _ = allan.oadev(angles_y, rate=sampling_rate, data_type="phase", taus="all")
taus_z, ad_z, _, _ = allan.oadev(angles_z, rate=sampling_rate, data_type="phase", taus="all")

taus_ax, ad_ax, _, _ = allan.oadev(accelerometer_x_array, rate=sampling_rate, data_type="freq", taus="all")
taus_ay, ad_ay, _, _ = allan.oadev(accelerometer_y_array, rate=sampling_rate, data_type="freq", taus="all")
taus_az, ad_az, _, _ = allan.oadev(accelerometer_z_array, rate=sampling_rate, data_type="freq", taus="all")

def extract_parameters(taus, allan_dev):
    bias_instability_idx = np.argmin(allan_dev)
    bias_instability = allan_dev[bias_instability_idx]
    bias_instability_tau = taus[bias_instability_idx]

    idx_1_sec = np.argmin(np.abs(taus - 1))
    N = allan_dev[idx_1_sec]

    log_taus = np.log10(taus)
    log_allan_dev = np.log10(allan_dev)
    slope = np.diff(log_allan_dev) / np.diff(log_taus)
    K_indices = np.where((slope >= 0.4) & (slope <= 0.6))[0]
    if K_indices.size > 0:
        K_idx = K_indices[0]
        K = allan_dev[K_idx] * np.sqrt(taus[K_idx])
    else:
        K = None

    return N, bias_instability, K, bias_instability_tau

gyroscope_params = [
    extract_parameters(taus_x, ad_x),
    extract_parameters(taus_y, ad_y),
    extract_parameters(taus_z, ad_z)
]
accelerometer_params = [
    extract_parameters(taus_ax, ad_ax),
    extract_parameters(taus_ay, ad_ay),
    extract_parameters(taus_az, ad_az)
]

fig, axes = plt.subplots(3, 1, figsize=(10, 12))
for i, (taus_values, allan_dev_values, parameters, color, axis_label) in enumerate(zip(
    [taus_x, taus_y, taus_z],
    [ad_x, ad_y, ad_z],
    gyroscope_params,
    ["red", "green", "blue"],
    ["X", "Y", "Z"]
)):
    N, B, K, BI_tau = parameters
    axes[i].loglog(taus_values, allan_dev_values, color=color,
                   label=f'Gyro Angle Allan deviation {axis_label}\nN={N:.2e}, B={B:.2e}, K={K}')
    axes[i].plot(BI_tau, B, 'o', label="Bias Instability, B")
    axes[i].legend()
    axes[i].grid(True)
    axes[i].set_xlabel("t (s)")
    axes[i].set_ylabel("Allan Deviation")
fig.suptitle("Allan Deviation Plot- Gyroscope Angles")
plt.show()

fig, axes = plt.subplots(3, 1, figsize=(10, 12))
for i, (taus_values, allan_dev_values, parameters, color, axis_label) in enumerate(zip(
    [taus_ax, taus_ay, taus_az],
    [ad_ax, ad_ay, ad_az],
    accelerometer_params,
    ["purple", "orange", "brown"],
    ["X", "Y", "Z"]
)):
    N, B, K, BI_tau = parameters
    axes[i].loglog(taus_values, allan_dev_values, color=color,
                   label=f'Acc. Allan deviation {axis_label}\nN={N:.2e}, B={B:.2e}, K={K}')
    axes[i].plot(BI_tau, B, 'o', label="Bias Instability, B")
    axes[i].legend()
    axes[i].grid(True)
    axes[i].set_xlabel("t (s)")
    axes[i].set_ylabel("Allan Deviation")
fig.suptitle("Allan Deviation Plot- Accelerometer Data")
plt.show()
