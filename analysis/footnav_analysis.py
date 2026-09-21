import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import plotly.graph_objects as go
from pathlib import Path



SESSION_NAME = "Session_2026-09-21_RealTest"

ACC_THRESHOLD = 0.3          # m/s^2
GYRO_THRESHOLD = 0.1        # rad/s
GNSS_ACCURACY_THRESHOLD = 5.0  # m
GNSS_TIME_TOLERANCE = 0.03  # s

GRAVITY = 9.81
EARTH_RADIUS = 6_371_000.0
COMPLEMENTARY_ALPHA = 0.98



# Paths / data loading


base_dir = Path(__file__).resolve().parent.parent
session_dir = base_dir / SESSION_NAME

accelerometer_data = pd.read_csv(session_dir / "accelerometer.csv")
gyroscope_data = pd.read_csv(session_dir / "gyroscope.csv")
magnetometer_data = pd.read_csv(session_dir / "magnetometer.csv")
barometer_data = pd.read_csv(session_dir / "barometer.csv")
gnss_data = pd.read_csv(session_dir / "gnss.csv")

accelerometer_data["time_s"] = accelerometer_data["time_ms"] / 1000.0
gyroscope_data["time_s"] = gyroscope_data["time_ms"] / 1000.0
magnetometer_data["time_s"] = magnetometer_data["time_ms"] / 1000.0
barometer_data["time_s"] = barometer_data["time_ms"] / 1000.0
gnss_data["time_s"] = gnss_data["time_ms"] / 1000.0


# Stationary detection / ZUPT mask


accelerometer_data["magnitude"] = np.sqrt(
    accelerometer_data["x"] ** 2
    + accelerometer_data["y"] ** 2
    + accelerometer_data["z"] ** 2
)

gyroscope_data["magnitude"] = np.sqrt(
    gyroscope_data["x"] ** 2
    + gyroscope_data["y"] ** 2
    + gyroscope_data["z"] ** 2
)

accelerometer_data["gravity_error"] = np.abs(
    accelerometer_data["magnitude"] - GRAVITY
)

acc_stationary = accelerometer_data["gravity_error"] < ACC_THRESHOLD

gyro_magnitude_for_acc = np.interp(
    accelerometer_data["time_s"],
    gyroscope_data["time_s"],
    gyroscope_data["magnitude"],
)

gyro_stationary = gyro_magnitude_for_acc < GYRO_THRESHOLD
stationary = acc_stationary.to_numpy() & gyro_stationary



#  Roll / pitch / yaw estimation


roll_acc = np.arctan2(
    accelerometer_data["y"],
    accelerometer_data["z"],
)

pitch_acc = np.arctan2(
    -accelerometer_data["x"],
    np.sqrt(
        accelerometer_data["y"] ** 2
        + accelerometer_data["z"] ** 2
    ),
)


roll_acc_for_gyro = np.interp(
    gyroscope_data["time_s"],
    accelerometer_data["time_s"],
    roll_acc,
)

pitch_acc_for_gyro = np.interp(
    gyroscope_data["time_s"],
    accelerometer_data["time_s"],
    pitch_acc,
)

gyro_dt = gyroscope_data["time_s"].diff().fillna(0.0)

filtered_roll = roll_acc_for_gyro[0]
filtered_pitch = pitch_acc_for_gyro[0]

filtered_roll_values = []
filtered_pitch_values = []

for i in range(len(gyroscope_data)):
    delta_t = gyro_dt.iloc[i]

    filtered_roll = (
        COMPLEMENTARY_ALPHA
        * (filtered_roll + gyroscope_data["x"].iloc[i] * delta_t)
        + (1.0 - COMPLEMENTARY_ALPHA) * roll_acc_for_gyro[i]
    )

    filtered_pitch = (
        COMPLEMENTARY_ALPHA
        * (filtered_pitch + gyroscope_data["y"].iloc[i] * delta_t)
        + (1.0 - COMPLEMENTARY_ALPHA) * pitch_acc_for_gyro[i]
    )

    filtered_roll_values.append(filtered_roll)
    filtered_pitch_values.append(filtered_pitch)

gyroscope_data["filtered_roll"] = filtered_roll_values
gyroscope_data["filtered_pitch"] = filtered_pitch_values


mag_x = magnetometer_data["x"]
mag_y = magnetometer_data["y"]
mag_z = magnetometer_data["z"]

roll_for_mag = np.interp(
    magnetometer_data["time_s"],
    gyroscope_data["time_s"],
    gyroscope_data["filtered_roll"],
)

pitch_for_mag = np.interp(
    magnetometer_data["time_s"],
    gyroscope_data["time_s"],
    gyroscope_data["filtered_pitch"],
)

mag_x_comp = (
    mag_x * np.cos(pitch_for_mag)
    + mag_z * np.sin(pitch_for_mag)
)

mag_y_comp = (
    mag_x * np.sin(roll_for_mag) * np.sin(pitch_for_mag)
    + mag_y * np.cos(roll_for_mag)
    - mag_z * np.sin(roll_for_mag) * np.cos(pitch_for_mag)
)

tilt_compensated_yaw = np.arctan2(
    mag_y_comp,
    mag_x_comp,
)


roll_for_acc = np.interp(
    accelerometer_data["time_s"],
    gyroscope_data["time_s"],
    gyroscope_data["filtered_roll"],
)

pitch_for_acc = np.interp(
    accelerometer_data["time_s"],
    gyroscope_data["time_s"],
    gyroscope_data["filtered_pitch"],
)

yaw_for_acc = np.interp(
    accelerometer_data["time_s"],
    magnetometer_data["time_s"],
    tilt_compensated_yaw,
)



# Device -> world acceleration + gravity compensation


acc_world_values = []

for i in range(len(accelerometer_data)):
    acc_device = np.array([
        accelerometer_data["x"].iloc[i],
        accelerometer_data["y"].iloc[i],
        accelerometer_data["z"].iloc[i],
    ])

    roll_i = roll_for_acc[i]
    pitch_i = pitch_for_acc[i]
    yaw_i = yaw_for_acc[i]

    cos_roll = np.cos(roll_i)
    sin_roll = np.sin(roll_i)
    cos_pitch = np.cos(pitch_i)
    sin_pitch = np.sin(pitch_i)
    cos_yaw = np.cos(yaw_i)
    sin_yaw = np.sin(yaw_i)

    rotation_matrix = np.array([
        [
            cos_yaw * cos_pitch,
            cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll,
            cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll,
        ],
        [
            sin_yaw * cos_pitch,
            sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll,
            sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll,
        ],
        [
            -sin_pitch,
            cos_pitch * sin_roll,
            cos_pitch * cos_roll,
        ],
    ])

    acc_world_values.append(rotation_matrix @ acc_device)

acc_world_values = np.asarray(acc_world_values)

accelerometer_data["world_x"] = acc_world_values[:, 0]
accelerometer_data["world_y"] = acc_world_values[:, 1]
accelerometer_data["world_z"] = acc_world_values[:, 2]

accelerometer_data["linear_x"] = accelerometer_data["world_x"]
accelerometer_data["linear_y"] = accelerometer_data["world_y"]
accelerometer_data["linear_z"] = accelerometer_data["world_z"] - GRAVITY



# Bias correction


if stationary.any():
    bias_x = accelerometer_data.loc[stationary, "linear_x"].mean()
    bias_y = accelerometer_data.loc[stationary, "linear_y"].mean()
    bias_z = accelerometer_data.loc[stationary, "linear_z"].mean()
else:
    bias_x = bias_y = bias_z = 0.0

accelerometer_data["linear_x_corrected"] = (
    accelerometer_data["linear_x"] - bias_x
)
accelerometer_data["linear_y_corrected"] = (
    accelerometer_data["linear_y"] - bias_y
)
accelerometer_data["linear_z_corrected"] = (
    accelerometer_data["linear_z"] - bias_z
)



#  INS velocity + position with ZUPT


dt = accelerometer_data["time_s"].diff().fillna(0.0)

velocity = np.zeros(3)
position = np.zeros(3)

velocity_values = []
position_values = []

for i in range(len(accelerometer_data)):
    delta_t = dt.iloc[i]

    acc_corrected = np.array([
        accelerometer_data["linear_x_corrected"].iloc[i],
        accelerometer_data["linear_y_corrected"].iloc[i],
        accelerometer_data["linear_z_corrected"].iloc[i],
    ])

    velocity += acc_corrected * delta_t

    if stationary[i]:
        velocity[:] = 0.0

    position += velocity * delta_t

    velocity_values.append(velocity.copy())
    position_values.append(position.copy())

velocity_values = np.asarray(velocity_values)
position_values = np.asarray(position_values)

accelerometer_data["velocity_x_corrected"] = velocity_values[:, 0]
accelerometer_data["velocity_y_corrected"] = velocity_values[:, 1]
accelerometer_data["velocity_z_corrected"] = velocity_values[:, 2]

accelerometer_data["position_x"] = position_values[:, 0]
accelerometer_data["position_y"] = position_values[:, 1]
accelerometer_data["position_z"] = position_values[:, 2]



#  Barometric relative altitude


p0 = barometer_data["pressure"].iloc[0]

barometer_data["relative_altitude"] = 44330.0 * (
    1.0 - (barometer_data["pressure"] / p0) ** (1.0 / 5.255)
)

baro_z_for_acc = np.interp(
    accelerometer_data["time_s"],
    barometer_data["time_s"],
    barometer_data["relative_altitude"],
)



# GNSS latitude/longitude -> local X/Y meters

lat0 = gnss_data["latitude"].iloc[0]
lon0 = gnss_data["longitude"].iloc[0]

delta_lat = np.radians(gnss_data["latitude"] - lat0)
delta_lon = np.radians(gnss_data["longitude"] - lon0)

gnss_data["gnss_y"] = delta_lat * EARTH_RADIUS
gnss_data["gnss_x"] = (
    delta_lon
    * EARTH_RADIUS
    * np.cos(np.radians(lat0))
)



#  EKF: INS prediction + ZUPT + GNSS corrections


# state = [x, y, vx, vy]
state = np.zeros(4)

P = np.eye(4)
Q = np.eye(4) * 0.01
R_zupt = np.eye(2) * 0.01

H_gnss = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
])

H_zupt = np.array([
    [0, 0, 1, 0],
    [0, 0, 0, 1],
])

gnss_index = 0
last_gnss_available = False

ekf_x_values = []
ekf_y_values = []
gnss_available_values = []

gnss_update_times = []
zupt_update_count = 0

for i in range(len(accelerometer_data)):
    delta_t = dt.iloc[i]
    current_time = accelerometer_data["time_s"].iloc[i]

    F = np.array([
        [1, 0, delta_t, 0],
        [0, 1, 0, delta_t],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])

    state = F @ state

    acc_x = accelerometer_data["linear_x_corrected"].iloc[i]
    acc_y = accelerometer_data["linear_y_corrected"].iloc[i]

    state[0] += 0.5 * acc_x * delta_t**2
    state[1] += 0.5 * acc_y * delta_t**2
    state[2] += acc_x * delta_t
    state[3] += acc_y * delta_t

    P = F @ P @ F.T + Q

    # ZUPT measurement update.
    if stationary[i]:
        z_zupt = np.array([0.0, 0.0])
        innovation_zupt = z_zupt - H_zupt @ state
        S_zupt = H_zupt @ P @ H_zupt.T + R_zupt
        K_zupt = P @ H_zupt.T @ np.linalg.inv(S_zupt)

        state = state + K_zupt @ innovation_zupt
        P = (np.eye(4) - K_zupt @ H_zupt) @ P
        zupt_update_count += 1

    
    while (
        gnss_index < len(gnss_data)
        and gnss_data["time_s"].iloc[gnss_index]
        < current_time - GNSS_TIME_TOLERANCE
    ):
        gnss_index += 1

    # GNSS measurement update.
    if gnss_index < len(gnss_data):
        gnss_time = gnss_data["time_s"].iloc[gnss_index]
        gnss_accuracy = gnss_data["accuracy"].iloc[gnss_index]

        last_gnss_available = (
            gnss_accuracy <= GNSS_ACCURACY_THRESHOLD
        )

        if (
            last_gnss_available
            and abs(current_time - gnss_time) < GNSS_TIME_TOLERANCE
        ):
            z_gnss = np.array([
                gnss_data["gnss_x"].iloc[gnss_index],
                gnss_data["gnss_y"].iloc[gnss_index],
            ])

            innovation = z_gnss - H_gnss @ state

            
            R_gnss = np.eye(2) * (gnss_accuracy**2)

            S = H_gnss @ P @ H_gnss.T + R_gnss
            K = P @ H_gnss.T @ np.linalg.inv(S)

            state = state + K @ innovation
            P = (np.eye(4) - K @ H_gnss) @ P

            gnss_update_times.append(current_time)
            gnss_index += 1

    ekf_x_values.append(state[0])
    ekf_y_values.append(state[1])
    gnss_available_values.append(last_gnss_available)

accelerometer_data["ekf_x"] = ekf_x_values
accelerometer_data["ekf_y"] = ekf_y_values
accelerometer_data["ekf_z"] = baro_z_for_acc
accelerometer_data["gnss_available"] = gnss_available_values




fused_trajectory = accelerometer_data[
    ["time_s", "ekf_x", "ekf_y", "ekf_z", "gnss_available"]
].copy()

csv_output = base_dir / "fused_trajectory.csv"
html_output = base_dir / "footnav_3d.html"

fused_trajectory.to_csv(csv_output, index=False)



plt.figure(figsize=(10, 4))
plt.plot(
    accelerometer_data["time_s"],
    accelerometer_data["gnss_available"].astype(int),
)
plt.xlabel("Time (s)")
plt.ylabel("GNSS State")
plt.yticks([0, 1], ["Unavailable", "Available"])
plt.title("GNSS Availability")
plt.grid()
plt.tight_layout()
plt.show()

# INS vs GNSS vs EKF
plt.figure(figsize=(8, 6))
plt.plot(
    accelerometer_data["position_x"],
    accelerometer_data["position_y"],
    label="INS",
)
plt.plot(
    gnss_data["gnss_x"],
    gnss_data["gnss_y"],
    marker="o",
    markersize=3,
    label="GNSS",
)
plt.plot(
    accelerometer_data["ekf_x"],
    accelerometer_data["ekf_y"],
    label="EKF",
)
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.title("INS vs GNSS vs EKF Trajectory")
plt.legend()
plt.axis("equal")
plt.grid()
plt.tight_layout()
plt.show()

# Interactive 3D fused trajectory
fig = go.Figure()

fig.add_trace(
    go.Scatter3d(
        x=fused_trajectory["ekf_x"],
        y=fused_trajectory["ekf_y"],
        z=fused_trajectory["ekf_z"],
        mode="lines",
        name="Fused Trajectory",
    )
)

fig.update_layout(
    title="FootNav 3D Fused Trajectory",
    scene=dict(
        xaxis_title="X Position (m)",
        yaxis_title="Y Position (m)",
        zaxis_title="Altitude (m)",
    ),
)

fig.write_html(html_output)

print(f"Total GNSS samples: {len(gnss_data)}")
print(f"Actual GNSS updates: {len(gnss_update_times)}")
print(f"ZUPT updates: {zupt_update_count}")
print(f"Saved: {csv_output}")
print(f"Saved: {html_output}")
