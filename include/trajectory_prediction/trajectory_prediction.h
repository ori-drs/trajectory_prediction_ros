// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the trajectory_prediction_ros package.
// © Copyright 2022, Mark Finean and Luka Petrovic 
// 
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the followingdisclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation 
// and/or other materials provided with the distribution.
// 
// 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software 
// without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS 
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE 
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Mark Finean and Luka Petrovic
 * \date    01-12-2022 
 */
//----------------------------------------------------------------------

#ifndef TRAJECTORY_PREDICTION_H
#define TRAJECTORY_PREDICTION_H

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <array>
#include <cmath>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "tf/transform_listener.h"
#include <control_msgs/JointTrajectoryControllerState.h>

#include <gpmp2/kinematics/PointRobot.h>
#include <gpmp2/kinematics/PointRobotModel.h>

#include <gpmp2/obstacle/AgentAvoidanceFactorPointRobot.h>
#include <gpmp2/obstacle/AgentAvoidanceFactorGPPointRobot.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGPPointRobot.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPointRobot.h>
#include <gpmp2/obstacle/PlanarSDF.h>
#include <gpmp2/planner/BatchTrajOptimizer.h>
#include <gpmp2/planner/TrajOptimizerSetting.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
//#include <gtsam/nonlinear/NonlinearOptimizer.h>

#include <gpmp2/gp/GaussianProcessPriorLinear.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include <mutex>

enum PredictionMode {CVM_MODE = 0, MP_MODE = 1};

// Constrain angles between -pi and pi
float constrainAngle(float x);

class TrajectoryPrediction {
 public:
  // TrajectoryPrediction() {}
  TrajectoryPrediction(ros::NodeHandle node, int prediction_mode = MP_MODE);

  ~TrajectoryPrediction() {}

  void distanceFieldCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
  void viconPersonPoseCallback(
      const geometry_msgs::TransformStamped::ConstPtr &msg);
  void cameraPersonPoseCallback(
      const geometry_msgs::PointStamped::ConstPtr &msg);
  void robotPositionCallback(
      const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);

  void pruneDetectionHistory();
  bool isLatestDetectionRecent();

  std::array<double, 2> getIntendedGoal(float &dist_to_goal);
  float getObstacleSpeed();

  void reestimateTrajectoryParams(const float obstacle_speed,
                                  const std::array<double, 2> obstacle_goal,
                                  const float dist_to_goal);
  void createSettings(float total_time = 10, int total_time_step = 10);
  void constructGraph(const gtsam::Vector &start_conf,
                      const gtsam::Vector &start_vel,
                      const gtsam::Vector &end_conf,
                      const gtsam::Vector &end_vel,
                      const gtsam::Point3 obstacle_point,
                      const bool add_goal_factors);
  void plan();
  gtsam::Values getCVMTraj(const gtsam::Vector &start_conf, const float speed);

  gtsam::Values getInitTraj(const gtsam::Vector &start_conf,
                            const gtsam::Vector &end_conf);
  gtsam::Values optimize(const gtsam::Values &init_values);
  gtsam::Values optimize(std::shared_ptr<gtsam::NonlinearOptimizer> opt,
                         const gtsam::NonlinearOptimizerParams &params,
                         bool iter_no_increase);

  void visualisePrediction(const gtsam::Values &plan,
                           const size_t num_keys) const;
  void visualiseStationaryPath() const;
  void publishDesiredGoal(const std::array<double, 2> obstacle_goal) const;

  void publishEmptyPrediction() const;
  void publishPredictedTrajectory(const gtsam::Values &result) const;
  void publishStationaryTrajectory() const;

 private:
  // ROS
  ros::NodeHandle node_;
  std::string goal_topic_, predicted_traj_topic_, global_frame_,
      path_vis_topic_, distance_field_topic_, person_position_topic_, robot_pos_topic_;
  ros::Subscriber sub_, distance_field_sub_, robot_pos_sub_;
  ros::Publisher desired_goal_pub_, predicted_traj_pub_, path_vis_pub_;
  std_msgs::Float32MultiArray::ConstPtr latest_msg_;
  ros::Time latest_person_msg_time_;
  bool person_detected_ = false;

  // std::array<std::array<double, 2>, 4> possible_goals_{
  //                                                     {{{0, 4}},
  //                                                     {{1.6, -3}},
  //                                                     {{-0.73, -2.4}},
  //                                                     {{-1.82, 0.62}}}};

  int prediction_mode_;

  // Intention
  // std::array<std::array<double, 2>, 1> possible_goals_{{{{3, -2}}}};
  std::array<double, 2> latest_person_pose_;
  float thresh_observed_t_secs_ = 2.0;   // seconds
  float prune_t_secs = 1.0;   // seconds
  float min_required_history_t_ = 0.3;   // seconds
  float default_human_speed_ = 4.0;      // m/s
  float stationary_thresh_speed_ = 0.1;  // m/s
  float stationary_thresh_dist_ = 0.5;   // m/s

  std::vector<ros::Time> detection_times_;
  std::vector<std::array<double, 2>> detection_positions_;

    // Most of the experiments
  std::array<std::array<double, 2>, 1> possible_goals_{{{{-1, 0}}}};
  
    //   For multi-goal experiment
//   std::array<std::array<double, 2>, 2> possible_goals_{{
//                                                         {{0, 0}},
//                                                         {{1.85, -1}}
//                                                       }};

  std::array<double, 2> pose_past_ = {0, 0};

  std::vector<double> distance_field_2d_;
  bool use_empty_distance_field_, use_rgbd_, df_initialised_;
  int num_rows_;

  // Motion planner and SDF
  gpmp2::PointRobotModel robot_;
  gpmp2::PlanarSDF sdf_;
  gtsam::Matrix data_;
  gtsam::NonlinearFactorGraph graph_;
  gpmp2::TrajOptimizerSetting setting_;
 
  gtsam::Vector3 robot_odom_state_;


  // Graph params
  double resolution_;
  double delta_t_, inter_dt_;
  double min_distance_ = 0.8;

  std::mutex data_mutex_;
};

#endif