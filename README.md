

# Localize

Adaptive Monte Carlo Localization integrated with ROS and the [MuSHR car](https://mushr.io).

_Note: Initially there were thoughts of implementing other algorithms in this repository, but that is not likely to happen given priorities on other projects._

</br>

# Adaptive Monte Carlo Localization | AMCL

## Overview
Adaptive Monte Carlo Localization (AMCL) is a **particle filter** based technique for localizing a mobile robot using motion control inputs, measurements from a range sensor, and a static map.

<details><summary><b>Algorithm Details</b></summary>

The core algorithm is comprised of the following steps:

1. **Initialization** - A distribution of particles is generated uniformly and randomly in the map's free space, or in proximity to an initial pose estimate.
2. **Motion update** - Particles are propagated forward in time using control inputs and a model of the robot's motion.
3. **Sensor update** - Particle likelihoods are updated using range measurements and a model of the range sensor's behavior.
4. **Resampling** - Particles are selected with replacement from the current distribution with probability proportional to their (normalized) likelihood determined during the sensor update.

_Adaptive_ MCL improves upon this algorithm by decreasing the number of particles as they converge to a small region of the state space. The approach utilizes the Kullback-Leibler divergence measure to determine the number of samples required to guarantee the error between the estimated distribution and the true distribution is within some specified bound.

A key assumption in the adaptive KLD-based approach is that the estimated distribution does not completely diverge from the true distribution. Given enough time though, a particle filter will generate an estimate that is arbitrarily incorrect (this occurs more often in symmetric spaces and large open areas). To recover from such scenarios - or any induced failure - random poses are added to the distribution as the average likelihood drops and falls below a threshold.
</details>

</br>

## Features

* **Path Tracking** - Estimates are provided at a rate of 50 Hz.
* **Global Localization** - Localization within the map with no prior estimate.
* **Failure Recovery** - Automatic recovery in the event of global localization failure.

</br>

## Demonstration

The video below shows AMCL running on the robot.

The performance of the laser limits the quality of the estimate. In particular, when the car rotates, the laser often warps what should be a straight line into an arc. This results in inaccuracies in the heading estimate as AMCL compares the measurements with the map. The backlash in the robot's steering mechanism (the car is using a hobby RC car servo which has low precision) also contributes to this chattering as it requires a noisier motion model to represent correctly - and in turn, more particles.

Note that RViz and the video itself are at 30fps.

https://user-images.githubusercontent.com/45929033/120374473-6ea09000-c2ce-11eb-8144-2e7277aafe50.mp4

</br>

## ROS Launch Files

Launch File | Description
------------|------------
`amcl.launch` | Launches AMCL on its own. Note that AMCL requires data from other nodes during initialization (e.g., the map), so you must start these nodes separately when using this.
`amcl_teleop.launch` | Launches AMCL and all required nodes for running with teleop control.

</br>

## ROS Launch File Parameters

Launch File Parameter | Type | Default | Description
----------------------|------|---------|------------
`car_name` | `string` | `car` | The name of the car, functioning as the namespace and tf_prefix.
`mode_real` | `bool` | `false` | Set to load actual hardware nodes instead of simulated versions (sensor, drive, etc.).
`use_modified_map` | `bool` | `false` | Set to load a modified map for AMCL to localize within. See the **Map Configuration** section for more information.

</br>

## ROS Parameters

ROS Parameter | Type | Default | Description
--------------|------|---------|------------
`~node_names/amcl` | `string` | `localizer` | The name of the localizer node.
`~node_names/drive` | `string` | `vesc` | The name of the drive node which provides velocity and steering angle data.
`~node_names/sensor` | `string` | `laser` | The name of the sensor node which provides range measurement data.
`~frame_ids/map` | `string` | `map` | Map frame ID for publishing transforms.
`~frame_ids/car_base` | `string` | `car_name/base_link` | Car origin frame ID for the pose estimate.
`~frame_ids/car_wheel_back_left` | `string` | `car_name/back_left/wheel_link` | Car back left wheel frame ID used in the motion model.
`~amcl/use_modified_map` | `bool` | `false` | Set to load a modified map for AMCL to localize within. See the **Map Configuration** section for more information.

</br>

## AMCL Parameters

AMCL Parameter | Type | Default (Sim) | Default (Real) | Description
---------------|------|---------------|----------------|------------
`~amcl/update_rate` | `double` | `50.0` | `50.0` | How often to publish the estimate (hz).
`~amcl/num_particles_min` | `int` | `1000` | `1000` | Minimum number of particles to use.
`~amcl/num_particles_max_local` | `int` | `3500` | `3500` | Maximum number of particles to use during local tracking.
`~amcl/num_particles_max_global` | `int` | `20000` | `20000` | Maximum number of particles to use during global localization / relocalization.
`~amcl/weight_avg_random_sample` | `double` | `1.0e-8` | `1.0e-8` | Particle distribution weight average below which random sampling is enabled.
`~amcl/weight_rel_dev_resample` | `double` | `0.50` | `0.50` | Relative standard deviation in particle distribution weights above which resampling is performed.

</br>

## Motion Model Parameters

Motion Model Parameter | Type | Default (Sim) | Default (Real) | Description
-----------------------|------|---------------|----------------|------------
`~motion/vel_lin_n1` | `double` | `0.005` | `0.10` | Increases translational noise as a function of the robot's linear velocity.
`~motion/vel_lin_n2` | `double` | `0.005` | `0.10` | Increases translational noise as a function of the robot's angular velocity.
`~motion/vel_ang_n1` | `double` | `0.01` | `0.25` | Increases angular noise (creating a wider 'arc' of x/y locations) as a function of the robot's linear velocity.
`~motion/vel_ang_n2` | `double` | `0.01` | `0.35` | Increases angular noise (creating a wider 'arc' of x/y locations) as a function of the robot's angular velocity.
`~motion/th_n1` | `double` | `0.01` | `0.25` | Increases rotational noise as a function of the robot's linear velocity.
`~motion/th_n2` | `double` | `0.01` | `0.50` | Increases rotational noise as a function of the robot's angular velocity.

</br>

## Sensor Model Parameters

Sensor Model Parameter | Type | Default (Sim) | Default (Real) | Description
-----------------------|------|---------------|----------------|------------
`~sensor/range_std_dev` | `float` | `0.20` | `0.20` | Range measurement standard deviation. Note that this should be significantly larger than the actual standard deviation of the sensor due to the sensitivity of the model to small changes in the pose (as well as imprecision in the map). Too small of a value will lead to many reasonably good estimates getting a very low weight, and this can lead to instability in the localization estimate.
`~sensor/decay_rate_new_obj` | `float` | `0.30` | `0.30` | Exponential decay rate for the new / unexpected (i.e., unmapped) object probability calculation. Typically expressed as a percentage with a value between 0 and 100.0. Higher values mean that only unexpected detections very close to the robot get a higher weight. Lower values give weight to unexpected detections both near and far away from the robot.
`~sensor/weight_no_obj` | `double` | `2.00` | `2.00` | Proportion (0 to 100) of the particle's final weight that is due to the sensor reporting nothing was detected (i.e., a 'max range measurement'). This value should be very low because AMCL rejects max range measurements unless a wide arc of measurements report a miss.
`~sensor/weight_new_obj` | `double` | `10.00` | `10.00` | Proportion (0 to 100) of the particle's final weight that is due to the sensor reporting a new / unexpected (i.e., unmapped) object.
`~sensor/weight_map_obj` | `double` | `87.00` | `87.00` | Proportion (0 to 100) of the particle's final weight that is due to the sensor reporting an expected (i.e., mapped) object.
`~sensor/weight_rand_effect` | `double` | `1.00` | `1.00` | Proportion (0 to 100) of the particle's final weight that is due to the sensor reporting a random measurement.
`~sensor/weight_uncertainty_factor` | `double` | `1.10` | `1.10` | Uncertainty factor used to reduce the weight of particle due to the approximate nature of the model. This value must be greater than 1.0.
`~sensor/prob_new_obj_reject` | `double` | `0.50` | `0.50` | Probability above which a ray is rejected for likely representing a new / unexpected (i.e., unmapped) object.

</br>

## Map Configuration

Map Config File | Description
----------------|------------
`map_actual.yaml` | Points to the map that represents the real environment. This should be the map that you plan to use during real navigation.
`map_modified.yaml` | (Optional) Points to a modified map for AMCL to localize within, if enabled by setting the parameter `amcl/use_modified_map` to `true`. The sensor simulation will generate range measurements based off of the _real_ map (loaded from `map_actual.yaml`) while AMCL will evaluate those range measurements against the _modified_ map. This allows for you to intentionally 'corrupt' the real map for the purposes of simulating real-world dynamic environments, such as furniture being moved, people walking around, etc.

</br>

## Publishers

Topic | Type | Description
------|------|------------
`<car name>/<amcl node name>/pose` | `geometry_msgs::PoseStamped` | Estimated pose of the car base frame in the map frame.
`<car name>/<amcl node name>/pose_array` | `geometry_msgs::PoseArray` | Top 5 estimated poses, in descending order of likelihood.
`/tf` | `geometry_msgs::TransformStamped` | AMCL publishes the `map` to `odom` coordinate frame transform based on the estimated pose of the car base frame in the map frame.

</br>

## Subscribers

Topic | Type | Description
------|------|------------
`<car name>/<drive node name>/sensors/core` | `vesc_msgs::VescStateStamped` | Motor electrical RPM - used to calculate linear velocity.
`<car name>/<drive node name>/sensors/servo_position_command` | `std_msgs::Float64` | Steering servo position command - used to calculate steering angle.
`<car name>/<sensor node name>/scan` | `sensor_msgs::LaserScan` | Sensor range measurements.
`/map_actual` | `nav_msgs::OccupancyGrid` | Temporary subscription to retrieve info for the map that represents the real environment.
`/map_modified` | `nav_msgs::OccupancyGrid` | Temporary subscription to retrieve info for a test map that simulates an altered environment.
`/tf` | `geometry_msgs::TransformStamped` | Temporary subscription to lookup fixed transforms from the car base frame to the sensor frame and from the car back left wheel frame to the sensor frame

</br>

## References

1. [MuSHR (site)](https://mushr.io) | [MuSHR (github)](https://github.com/prl-mushr) for building the car and getting an introduction to ROS.
2. S. Thrun, W. Burgard, and D. Fox. _Probabilistic Robotics_, The MIT Press, 2006.
3. D. Knuth. _The Art of Computer Programming, Volume 2: Seminumerical Algorithms_, 3rd edition, Addison-Wesley, 1998.
