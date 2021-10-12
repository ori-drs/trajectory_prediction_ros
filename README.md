# trajectory-prediction-ros
-----

A ROS node for trajectory prediction on the ORI dataset.

### Prerequisites & installation

If both gtsam and gpmp2 are installed correctly, then just use
```
catkin_make
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





