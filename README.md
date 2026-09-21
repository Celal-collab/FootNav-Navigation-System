# FootNav Navigation System

FootNav is a smartphone-based prototype navigation system that combines GNSS and inertial sensors to estimate a user's 3D trajectory, including periods where GNSS quality becomes unreliable.

The project uses accelerometer, gyroscope, magnetometer, barometer and GNSS data collected from an Android device.

## Features

- Android sensor data collection
- Accelerometer and gyroscope based motion processing
- Roll and pitch estimation with a complementary filter
- Tilt-compensated magnetometer yaw estimation
- Device-to-world coordinate transformation
- Gravity compensation
- Sensor bias correction
- Zero Velocity Update (ZUPT)
- Inertial Navigation System (INS)
- GNSS coordinate conversion to local X/Y coordinates
- Extended Kalman Filter (EKF) based GNSS + INS fusion
- Barometric relative altitude estimation
- GNSS quality based availability detection
- CSV trajectory export
- Interactive 3D trajectory visualization with Plotly

## System Pipeline

  
Android Phone
     │
     ├── Accelerometer
     ├── Gyroscope
     ├── Magnetometer
     ├── Barometer
     └── GNSS
          │
          ▼
Sensor Recording
          │
          ▼
Orientation Estimation
          │
          ▼
Device → World Transformation
          │
          ▼
Gravity & Bias Compensation
          │
          ▼
ZUPT + INS
          │
          ▼
GNSS Quality Check
          │
          ▼
Extended Kalman Filter
          │
          ▼
Barometric Altitude
          │
          ▼
3D Fused Trajectory

-Project Structure-

FootNav-Navigation-System/
│
├── analysis/
│   └── footnav_analysis.py
│
├── README.md
├── requirements.txt
└── .gitignore

-Input Data-

The analysis expects a session directory containing:

Session_YYYY-MM-DD_Name/
├── accelerometer.csv
├── gyroscope.csv
├── magnetometer.csv
├── barometer.csv
└── gnss.csv

The session directory is selected in:
SESSION_NAME = "Session_2026-09-21_RealTest"

-Outputs-

The analysis generates:

fused_trajectory.csv
footnav_3d.html

It also displays:

GNSS availability
INS vs GNSS vs EKF trajectory comparison

The generated HTML file contains an interactive 3D fused trajectory.

Navigation Logic

The system first estimates device orientation using accelerometer, gyroscope and magnetometer measurements.

Acceleration is transformed from the device coordinate system into the world coordinate system. Gravity and stationary bias are then removed.

The corrected acceleration is integrated to estimate velocity and position.

When the system detects that the device is stationary, ZUPT is used to reduce accumulated velocity drift.

GNSS measurements are converted into a local metric coordinate system. When GNSS accuracy is considered reliable, GNSS measurements are used to correct the inertial estimate through an Extended Kalman Filter.

Barometric pressure measurements are used to estimate relative altitude.

Current Prototype Results

The prototype has successfully processed real smartphone recordings and produced:

GNSS position measurements
INS trajectory estimates
EKF corrected trajectories
ZUPT updates
Relative barometric altitude
Interactive 3D fused trajectories
Limitations

This project is currently a prototype.

Some known limitations include:

IMU integration drift
Magnetometer interference, especially indoors
Barometric altitude drift
Simplified EKF process noise model
GNSS accuracy is used as an approximate measurement uncertainty
Device axis orientation requires physical calibration
EKF state currently does not estimate accelerometer bias or yaw error

Future versions could improve the system using quaternion-based orientation estimation, bias-aware EKF states and more advanced pedestrian dead reckoning algorithms.

Technologies
Kotlin / Android
Python
NumPy
Pandas
Matplotlib
Plotly
Extended Kalman Filter
ZUPT
INS
GNSS
Status

Functional prototype completed.

The current system supports the complete pipeline from smartphone sensor recording to GNSS/INS fusion and interactive 3D trajectory visualization.

