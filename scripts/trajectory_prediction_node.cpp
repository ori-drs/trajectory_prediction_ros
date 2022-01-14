#include <trajectory_prediction/trajectory_prediction.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_prediction_node");
  ros::NodeHandle nh;
  
  int prediction_mode;

  nh.param<int>("prediction_mode", prediction_mode, 1);

  TrajectoryPrediction trajPred(nh, prediction_mode);

  ros::Rate loop_rate(10);  // 10 Hz

  while (ros::ok()) {
    ros::spinOnce();
    trajPred.plan();
    loop_rate.sleep();
  }

  return 0;
}
