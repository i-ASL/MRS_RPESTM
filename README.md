# multiple_turtlebots_sim
Relative pose estimation between 2 robots in gazebo simulation.
There are four scenarios
1) Network + odometry + range-only sensors
2) Network + odometry + bearing-only sensors
3) Network + odometry + range and bearing sensors
4) Odometry + range and bearing sensors

Compared the results of each scenarios using Kalman Filter and Nonlinear optimization.
Ceres Sovler is used for Non-linear optimization.
And I did M-estimator experiments when there are outliers in sensor data.

`roslaunch multiple_turtlebots_sim simulation_kalman.launch `
