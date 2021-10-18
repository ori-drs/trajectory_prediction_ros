#include <trajectory_prediction/trajectory_prediction.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_prediction_node");
  ros::NodeHandle nh;

  TrajectoryPrediction trajPred(nh);

  ros::Rate loop_rate(10);  // 10 Hz

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
