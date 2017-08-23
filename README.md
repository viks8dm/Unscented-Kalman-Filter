# Unscented Kalman Filter

In this project-2 of term-2 of self driving car nanodegree program by Udacity Unscented Kalman Filter is utilized to estimate the state of a moving object of interest with noisy lidar and radar measurements.

For satisfactory project completion, the requirement is that the RMSE values obtained should be lower than the target-RMSE values outlined in project ruberic, details of which are available on [the project resources page](https://review.udacity.com/#!/projects/284/view)

## Contents of this repository

The project has been created using Udacity's [starter Code](https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project)

To successfully run this code, one needs to download the [Eigen-library](https://d17h27t6h515a5.cloudfront.net/topher/2017/March/58b7604e_eigen/eigen.zip), unzip it and save the `Eigen` folder in the `src` directory (`NOTE`: The `src` folder on my github page already has this in place, as I have forked the github page from `Udacity`)


* `src/main.cpp` - communicates with the [simulator](https://github.com/udacity/self-driving-car-sim/releases/); receiving data measurements, calls a function to run the Unscented Kalman filter script and calls a function to calculate RMSE
* `src/ukf.cpp` - initializes the filter, defines the prediction function, the update function for lidar, and the update function for radar
* `src/tools.cpp` - function to calculate RMSE
* `results` - directory that contains any results


## Results & Discussion

[image1]: ./results/dataset1.jpeg "simulation result for dataset-1 with RSME values"
[image2]: ./results/dataset2.jpeg "simulation result for dataset-2 with RSME values"

The RMSE-target criteria for dataset-1 and dataset-2 were given in the project ruberic as listed in the table below:


| RMSE parameter  | dataset-1: target | dataset-2: target  |
| ------------- |:-------------:| -----:|
| PX      | 0.09 | 0.20 |
| PY      | 0.10 | 0.20 |
| VX 		| 0.40 | 0.55 |
| VY      | 0.30 | 0.55 |

As shown in the screenshots below. Both for dataset-1 and dataset-2, the final RMSE values satisfy this criteria, and corresponsing values are:

| RMSE parameter  | dataset-1: output | dataset-2: output  |
| ------------- |:-------------:| -----:|
| PX      | 0.0687 | 0.0664 |
| PY      | 0.0835 | 0.0651 |
| VX 		| 0.3615 | 0.4977 |
| VY      | 0.1951 | 0.2023 |


--

![alt text][image1]
--

![alt text][image2]
--

### Parameter Tuning

Achieving the RMSE target involved playing around with the process noise parameters `std_a_` and `std_yawdd_`. The simulation was started with default values of 30 for each and after multiple tuning runs, the final values used are:

* `std_a_` = 1.25
*  `std_yawdd_` = 0.75

It is possible that slightly higher value combination of these parameters will also achieve the desired RMSE target (for a given initial value set), however it was not attempted to optimize these values, but just get a combination for with the RMSE criteria for both data-sets are satisfied.

### Setting Initial value

The simulation was started with default values for both the state vector `x_` and state covariance matrix `P_`, such that;

* `x_` = `[0, 0, 0, 0, 0]` ; a vector of zeros for CTRV Model with `x_`  = `[px, py, vel, yaw-ang, yaw-ang_rate]`
* `P_` = 5x5 identity matrix

However, with further tuning of the process noise parameters it was evident that by just tuning these noise parameters, achieving the target RMSE values is not an easy task, hence, initial values were updated after looking at some sample lidar and radar data. The final values used were:

* `x_` = `[0.01, 0.01, 0, 0.001, 0]`
* `P_` = 5x5 matrix with non-zero diagonal entries. The diagonal entries were updated to:
	* `P_(0,0) = 1`
	* `P_(1,1) = 1`
	* `P_(2,2) = 40`
	* `P_(3,3) = 1`
	* `P_(4,4) = 0.1`

	
It was observed that covariance matrix entry corresponding to velocity-magnitude `P_(2,2) = 40` has significant impact on RMSE for `VX` and this was most difficult to tune, especially in the sense of stricking a balance between the path traces in dataset-1 and dataset-2. If other parameters are kept constant, values greater than 50 seem to work great with dataset-2, however, they lead to deterioration in performance for dataset-1. This might be because the final result is dependent on combination of parameters for both velocity-magnitude and yaw-rate.

## To run the code

Clone this repository and enter following commands in a terminal

`mkdir build && cd build`

`cmake .. && make`

`./UnscentedKF`

After execution of `./UnscentedKF`, simulator should be opened and it should be started with dataset of interest selected, as shown in the screenshots above. 




# Unscented-Kalman-Filter
