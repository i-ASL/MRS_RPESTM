# Multi-Robot Relative Pose Estimation in SE(2) with Observability Analysis: A Comparison of Extended Kalman Filtering and Robust Pose Graph Optimization (T-IV)

In this study, we address challenges in multi-robot localization, with a particular focus on cooperative localization and the observability analysis of relative pose estimation. Cooperative localization enhances each robot’s information accuracy through communication networks and message passing. When odometry data from a target robot can be transmitted to an ego robot, the observability of their relative pose estimation can be achieved using either range-only or bearing-only measurements, provided that both robots have non-zero linear velocities. However, if the target robot’s odometry data is not directly transmitted and must instead be estimated by the ego robot, both range and bearing measurements are necessary to ensure observability. This research validates the feasibility of relative pose estimation in ground-based multi-robot systems by exploring different sensing and communication structures. 

In ROS/Gazebo simulations, we compare the estimation accuracy of extended Kalman filter (EKF) and pose graph optimization (PGO) methods, incorporating different robust loss functions through filtering and smoothing techniques, with varying sliding window batch sizes. For hardware experiments, two TurtleBot3 robots equipped with UWB modules were used to estimate inter-robot relative poses in real-world scenarios, employing both EKF and PGO-based methods. The real-world experiments demonstrate the practical applicability of the proposed decentralized relative pose estimation methods, which rely solely on onboard sensing without the need for inter-robot communication. 

# Package Summary
- Requirements

# Try it out
`roslaunch multiple_turtlebots_sim simulation_kalman.launch`

<!-- # Updates
- updates
- cite as:

# Documentation

# Publications -->


<!-- # multiple_turtlebots_sim
Relative pose estimation between 2 robots in gazebo simulation.
There are four scenarios
1) Network + odometry + range-only sensors
2) Network + odometry + bearing-only sensors
3) Network + odometry + range and bearing sensors
4) Odometry + range and bearing sensors

Compared the results of each scenarios using Kalman Filter and Nonlinear optimization.
Ceres Sovler is used for Non-linear optimization.
And I did M-estimator experiments when there are outliers in sensor data.

`roslaunch multiple_turtlebots_sim simulation_kalman.launch ` -->
