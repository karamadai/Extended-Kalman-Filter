<h2>Radar and Lidar Sensor Fusion with Extended Kalman Filter</h2>

The goal of this project is to combine the Radar and Lidar sensor data using an extended kalman filter developed in C++

The code read the input from the text file. The input file provides the Radar and the Lidar data in the following format:

L(for laser) meas_px meas_py timestamp gt_px gt_py gt_vx gt_vy
R(for radar) meas_rho meas_phi meas_rho_dot timestamp gt_px gt_py gt_vx gt_vy

The code outputs to the console the accuracy of the predictions as the root mean squared error (RMSE)  px, py, vx and vy . The predictions (estimations) corresponding to the sensor inputs are saved to an output file.

<h3>Assumptions<h3>

The measurement covariance R for laser is a 2x2 matrix  [0.0225 0,  0 0.0225]
The measurement covariance R for radar is a 3x3 matrix [0.09 0 0, 0 0.009 0, 0 0 0.09]

<h3> Input data validation for zero position data in Radar and Lidar sensor output</h3>

Presence of zero valued position data in the radar output will cause a divide by zero error when computing the Jacobian at that point.  To avoid this, the code will add a small position data to the rho value of the radar output if the value is zero( line 26, kalman_filter.cpp) .
 
 <h3> Filter Performance </h3>
The performance of the filter is measured by the root mean squared error of the estimated state against the ground truth. The filter gives the following RMSE against the data file 1 and 2 .
**Sample-Laser-Radar-Measurement-Data-2.Txt**
RMSE
0.186472
0.19027
0.477449
0.807359

**Sample-Laser-Radar-Measurement-Data-1.Txt**
RMSE
0.0651648
0.0605379
0.533212
0.544193






> Written with [StackEdit](https://stackedit.io/).
