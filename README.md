FootNav

FootNav is a prototype navigation system that fuses smartphone IMU, magnetometer,
barometer and GNSS data to estimate a 3D trajectory.

Pipeline

Read synchronized sensor CSV files.

Detect stationary periods for ZUPT.

Estimate roll/pitch with a complementary filter.

Estimate yaw from a tilt-compensated magnetometer.

Rotate acceleration from device coordinates to world coordinates.

Remove gravity and estimate stationary bias.

Integrate acceleration for INS velocity/position.

Apply ZUPT velocity corrections.

Convert GNSS latitude/longitude to local meter coordinates.

Fuse INS and GNSS with a basic EKF.

Use barometric relative altitude as the Z component.

Export the fused trajectory to CSV and interactive Plotly HTML.

Expected session files

Create a session directory beside the analysis folder:

Session_YYYY-MM-DD_Name/
├── accelerometer.csv
├── gyroscope.csv
├── magnetometer.csv
├── barometer.csv
└── gnss.csv

Then update SESSION_NAME near the top of analysis/footnav_analysis.py.

Install

pip install -r requirements.txt

Run

python analysis/footnav_analysis.py

Outputs

fused_trajectory.csv

footnav_3d.html

GNSS availability plot

INS vs GNSS vs EKF trajectory plot

Notes

This is a prototype rather than a production-grade inertial navigation system.
IMU drift, magnetometer disturbances, barometric drift and sensor-axis calibration
can affect trajectory accuracy.