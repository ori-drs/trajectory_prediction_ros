trajectory_prediction_ros
===================================================

This package implements the human trajectory prediction method as described in the paper:<br>
"Motion Planning in Dynamic Environments Using 
Context-Aware Human Trajectory Prediction", <em>Mark Nicholas Finean, Luka Petrović, 
Wolfgang Merkt, Ivan Marković, and Ioannis Havoutis</em>, submitted to The International Journal
of Robotics Research. <br>

Link to: [Preprint](http://arxiv.org/abs/2201.05058)<br>
Link to: [Video](https://www.youtube.com/watch?v=gdC3mpZNjG4&t=5s)<br>
Link to: [Dataset](https://ori-drs.github.io/oxford-indoor-human-motion-dataset/)<br>

### Prerequisites & installation

If both GTSAM and GPMP2 are installed correctly, then this package can be built within a catkin workspace using:
```
catkin build
```

### To run
Place the dataset bags in the `bags` folder. Place the corresponding `map.yaml` in the `scripts` folder.
Edit the launch file to point to the correct bag.
Run roscore and rviz.
Launch the roslaunch file, which runs the map server, static_tf_publisher, trajectory_prediction node and plays the bag you selected in the launch file.

### Inputs and outputs
The `trajectory_prediction` node subscribes to the following topics:
```
/vicon/person_1/person_1
```

It publishes the following:
```
/desired_goal
```
which is a `PointStamped` that publishes the current desired goal obtained by our intention recognition method,
```
/predicted_trajectory
```
which is a `PoseArray` that publishes the currently predicted trajectory obtained by running gpmp2 starting from the current position with the desired goal.

### Launch commands

roslaunch trajectory_prediction trajectory_prediction.launch global_frame:=odom use_empty_distance_field:=true
roslaunch trajectory_prediction hsr_sim.launch global_frame:=odom use_empty_distance_field:=true

## Citing
-----

If you use this work, please cite following publications:

```
@INPROCEEDINGS{Finean2022,
  author={Finean, Mark Nicholas and Petrović, Luka and Merkt, Wolfgang and Marković, Ivan and Havoutis, Ioannis},
  title={Motion Planning in Dynamic Environments Using Human Trajectory Prediction}, 
  year={2022},
  }
```

## License
-----
This repository is licensed under the BSD-3-Clause license.