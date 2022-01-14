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

#include <trajectory_prediction/trajectory_prediction.h>

// Constrain angles between -pi and pi
float constrainAngle(float x) {
  x = fmod(x + M_PI, 2 * M_PI);
  if (x < 0) x += 2 * M_PI;
  return x - M_PI;
}



TrajectoryPrediction::TrajectoryPrediction(ros::NodeHandle node, int prediction_mode) {
  node_ = node;
  prediction_mode_ = prediction_mode;

  ROS_INFO("Initialising trajectory prediction node.");

  node_.param<std::string>("goal_topic", goal_topic_, "desired_goal");
  node_.param<std::string>("predicted_traj_topic", predicted_traj_topic_,
                           "predicted_path");
  node_.param<std::string>("path_vis_topic", path_vis_topic_,
                           "predicted_rviz_path");
  node_.param<std::string>("global_frame", global_frame_, "/vicon/world");
  node_.param<std::string>("distance_field_topic", distance_field_topic_,
                           "/distance_field_2d");
  node_.param<std::string>("person_position_topic", person_position_topic_,
                           "/vicon/person_1/person_1");

  node_.param<std::string>("robot_odom_topic", robot_pos_topic_,
                           "/hsrb/omni_base_controller/state");
                           
  node_.param<bool>("use_rgbd", use_rgbd_, true);

  node_.param<double>("resolution", resolution_, 0.05);
  node_.param<int>("num_rows", num_rows_, 256);
  node_.param<bool>("use_empty_distance_field", use_empty_distance_field_,
                    true);
  node_.param<double>("obstacle_delta_t", delta_t_, 0.5);

  df_initialised_ = false;
  
  gpmp2::BodySphereVector body_spheres;
  body_spheres.push_back(gpmp2::BodySphere(0, 0.6, Point3(0, 0, 0)));
  robot_ = gpmp2::RobotModel<gpmp2::PointRobot>(gpmp2::PointRobot(2, 1),
                                                body_spheres);

  if (use_rgbd_) {
    sub_ =
        node_.subscribe(person_position_topic_, 1,
                        &TrajectoryPrediction::cameraPersonPoseCallback, this);
  } else {
    sub_ =
        node_.subscribe(person_position_topic_, 100,
                        &TrajectoryPrediction::viconPersonPoseCallback, this);
  }

  distance_field_sub_ =
      node_.subscribe(distance_field_topic_, 2,
                      &TrajectoryPrediction::distanceFieldCallback, this);

  robot_pos_sub_ =
      node_.subscribe(robot_pos_topic_, 1,
                      &TrajectoryPrediction::robotPositionCallback, this);

  desired_goal_pub_ =
      node_.advertise<geometry_msgs::PointStamped>(goal_topic_, 100);
  predicted_traj_pub_ =
      node_.advertise<geometry_msgs::PoseArray>(predicted_traj_topic_, 100);
  path_vis_pub_ = node_.advertise<nav_msgs::Path>(path_vis_topic_, 1000);

  robot_odom_state_ = gtsam::Vector3(0.0, 0.0, 0.0);


  createSettings();
  ROS_INFO("Trajectory prediction node initialised.");
}

void TrajectoryPrediction::distanceFieldCallback(
    const std_msgs::Float32MultiArray::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_msg_ = msg;
  df_initialised_ = true;
}

void TrajectoryPrediction::robotPositionCallback(
    const control_msgs::JointTrajectoryControllerState::ConstPtr& msg) {
    robot_odom_state_[0] = msg->actual.positions[0];
    robot_odom_state_[1] = msg->actual.positions[1];
    robot_odom_state_[2] = msg->actual.positions[2];
}

void TrajectoryPrediction::viconPersonPoseCallback(
    const geometry_msgs::TransformStamped::ConstPtr &msg) {
  std::array<double, 2> pose_current = {msg->transform.translation.x,
                                        msg->transform.translation.y};
  person_detected_ = true;

  latest_person_pose_ = {msg->transform.translation.x,
                         msg->transform.translation.y};
  latest_person_msg_time_ = msg->header.stamp;

  detection_positions_.push_back(
      {msg->transform.translation.x, msg->transform.translation.y});
  detection_times_.push_back(msg->header.stamp);
  // this->plan(pose_current);
}

void TrajectoryPrediction::cameraPersonPoseCallback(
    const geometry_msgs::PointStamped::ConstPtr &msg) {
  person_detected_ = true;

  latest_person_pose_ = {msg->point.x, msg->point.y};
  latest_person_msg_time_ = msg->header.stamp;

  detection_positions_.push_back({msg->point.x, msg->point.y});
  detection_times_.push_back(msg->header.stamp);
  // this->plan();
}

void TrajectoryPrediction::createSettings(float total_time,
                                          int total_time_step) {
  int total_step = 20;

  gtsam::Matrix2 Qc = 0.1 * gtsam::Matrix::Identity(2, 2);
  setting_ = gpmp2::TrajOptimizerSetting(2);
  setting_.setLM();
  setting_.set_total_step(total_step);
  setting_.set_total_time(total_step * delta_t_);
  // setting_.set_epsilon(0.1); // day 1
  setting_.set_epsilon(0.3);
  setting_.set_cost_sigma(0.4);
  setting_.set_obs_check_inter(3);
  setting_.set_conf_prior_model(0.001);
  setting_.set_vel_prior_model(0.001);
  setting_.set_Qc_model(Qc);
  // setting_.setVerbosityError();
  setting_.setVerbosityNone();
  setting_.set_rel_thresh(1e-2);

  inter_dt_ = delta_t_ / static_cast<double>(setting_.obs_check_inter + 1);
}

void TrajectoryPrediction::reestimateTrajectoryParams(
    const float obstacle_speed, const std::array<double, 2> obstacle_goal,
    const float dist_to_goal) {
  // float dist = std::sqrt(std::pow(obstacle_goal[0] - latest_person_pose_[0],
  // 2) +
  //                          std::pow(obstacle_goal[1] -
  //                          latest_person_pose_[1], 2));

  float est_time = dist_to_goal / obstacle_speed;
  float num_steps = ceil(est_time / delta_t_);

  setting_.set_total_step(num_steps);
  setting_.set_total_time(num_steps * delta_t_);
}

void TrajectoryPrediction::constructGraph(const gtsam::Vector &start_conf,
                                          const gtsam::Vector &start_vel,
                                          const gtsam::Vector &end_conf,
                                          const gtsam::Vector &end_vel,
                                          const gtsam::Point3 obstacle_point,
                                          const bool add_goal_factors) {
  
  graph_ = gtsam::NonlinearFactorGraph();

  if (!use_empty_distance_field_ && df_initialised_) {

    std::vector<double> vec_data(latest_msg_->data.begin(),
                                 latest_msg_->data.end());
    gtsam::Point2 origin(-(num_rows_ / 2.0) * resolution_,
                         -(num_rows_ / 2.0) * resolution_);

    std::lock_guard<std::mutex> lock(data_mutex_);

    data_ = Eigen::Map<gtsam::Matrix>(
        &vec_data[0], num_rows_,
        num_rows_);  // Assume num_rows and num_cols are equal
    data_.transposeInPlace();


    sdf_ = gpmp2::PlanarSDF(origin, resolution_, data_);
  } else {
    data_ = Eigen::MatrixXd::Zero(num_rows_, num_rows_);
    gtsam::Point2 origin(0.0, 0.0);
    sdf_ = gpmp2::PlanarSDF(origin, resolution_, data_);
  }

  for (size_t i = 0; i < setting_.total_step; i++) {
    gtsam::Key pose_key = gtsam::Symbol('x', i);
    gtsam::Key vel_key = gtsam::Symbol('v', i);

    // start and end
    if (i == 0) {
      graph_.add(gtsam::PriorFactor<gtsam::Vector>(pose_key, start_conf,
                                                   setting_.conf_prior_model));
      graph_.add(gtsam::PriorFactor<gtsam::Vector>(vel_key, start_vel,
                                                   setting_.vel_prior_model));

    } else if ((i == setting_.total_step - 1) && add_goal_factors) {
      graph_.add(gtsam::PriorFactor<gtsam::Vector>(pose_key, end_conf,
                                                   setting_.conf_prior_model));
      graph_.add(gtsam::PriorFactor<gtsam::Vector>(vel_key, end_vel,
                                                   setting_.vel_prior_model));
    }

    if (i > 0) {
      gtsam::Key last_pose_key = gtsam::Symbol('x', i - 1);
      gtsam::Key last_vel_key = gtsam::Symbol('v', i - 1);

      // GP factor
      graph_.add(gpmp2::GaussianProcessPriorLinear(last_pose_key, last_vel_key,
                                                   pose_key, vel_key, delta_t_,
                                                   setting_.Qc_model));
      // non-interpolated cost factor
      graph_.add(gpmp2::ObstaclePlanarSDFFactorPointRobot(
          pose_key, robot_, sdf_, setting_.cost_sigma, setting_.epsilon));

      // human-robot factor (we assume that a human is affected by robot's
      // movement and will try to avoid collision)
      // graph_.add(gpmp2::AgentAvoidanceFactorPointRobot(pose_key, robot_,
      //                                            0.1,
      //                                            setting_.epsilon + 0.3, obstacle_point));

      // interpolated cost factor
      if (setting_.obs_check_inter > 0) {
        for (size_t j = 1; j <= setting_.obs_check_inter; j++) {
          const double tau = inter_dt_ * static_cast<double>(j);
          graph_.add(gpmp2::ObstaclePlanarSDFFactorGPPointRobot(
              last_pose_key, last_vel_key, pose_key, vel_key, robot_, sdf_,
              setting_.cost_sigma, setting_.epsilon, setting_.Qc_model,
              delta_t_, tau));
          // graph_.add(gpmp2::AgentAvoidanceFactorGPPointRobot(
          //     last_pose_key, last_vel_key, pose_key, vel_key, robot_,
          //     0.1, setting_.epsilon + 0.3, setting_.Qc_model,
          //     delta_t_, tau, obstacle_point));
        }
      }
    }
  }
}

void TrajectoryPrediction::pruneDetectionHistory() {
  // ros::Time t_now = ros::Time::now();
  ros::Time latest_t = detection_times_[detection_times_.size() - 1];

  for (size_t i = detection_times_.size() - 1; i > 0; i--) {
    float diff = (latest_t - detection_times_[i]).toSec();

    if (diff > prune_t_secs) {
      detection_times_.erase(detection_times_.begin(),
                             detection_times_.begin() + i);
      detection_positions_.erase(detection_positions_.begin(),
                                 detection_positions_.begin() + i);
      break;
    }
  }
}

gtsam::Values TrajectoryPrediction::getCVMTraj(const gtsam::Vector &start_conf, const float speed) {

  pose_past_ = detection_positions_.front();

  double human_ori = std::atan2(latest_person_pose_[1] - pose_past_[1], 
                                latest_person_pose_[0] - pose_past_[0]);

  int num_steps = 30;

  gtsam::Values init_values;
  
  for (size_t i = 0; i < num_steps; i++) {
    gtsam::Vector pose = start_conf + 
                        (i * delta_t_ * speed) * gtsam::Vector2(cos(human_ori), sin(human_ori));
    init_values.insert(gtsam::Symbol('x', i), pose);
  }

  return init_values;
}

gtsam::Values TrajectoryPrediction::getInitTraj(const gtsam::Vector &start_conf,
                                                const gtsam::Vector &end_conf) {
  gtsam::Values init_values;
  
  gtsam::Vector avg_vel =
      ((end_conf - start_conf) / setting_.total_step) / delta_t_;

  for (size_t i = 0; i < setting_.total_step; i++) {
    gtsam::Vector pose =
        start_conf + (end_conf - start_conf) * i / (setting_.total_step - 1);

    init_values.insert(gtsam::Symbol('x', i), pose);
    init_values.insert(gtsam::Symbol('v', i), avg_vel);
  }

  

  return init_values;
}

std::array<double, 2> TrajectoryPrediction::getIntendedGoal(
    float &dist_to_goal) {
  std::array<double, 2> goal;

  // std::cout << "------------------------------" << std::endl;
  // std::cout << "-------Current history--------"<< std::endl;
  // for (size_t i = 0; i < detection_times_.size() ;  i++)
  // {
  //     std::cout << "(" << detection_positions_[i][0] << ", " <<
  //     detection_positions_[i][1] << ")" << std::endl;
  // }
  // std::cout << "------------------------------" << std::endl;
  // std::cout << std::endl;
  // std::cout << std::endl;

  // Get the current orientation
  pose_past_ = detection_positions_.front();

  double human_ori =
      constrainAngle(std::atan2(latest_person_pose_[1] - pose_past_[1],
                                latest_person_pose_[0] - pose_past_[0]));

  // std::cout << "Past position: (" << pose_past_[0] << ", " << pose_past_[1]
  // << ")" << std::endl; std::cout << "Current position: (" <<
  // latest_person_pose_[0] << ", " << latest_person_pose_[1] << ")" <<
  // std::endl;

  // pose_past_ = latest_person_pose_;

  // Identify the likely intended goal
  double min_theta = 3.15;
  double goal_dist, goal_theta;

  for (int i = 0; i < possible_goals_.size(); ++i) {
    goal_dist =
        std::sqrt(std::pow(possible_goals_[i][0] - latest_person_pose_[0], 2) +
                  std::pow(possible_goals_[i][1] - latest_person_pose_[1], 2));

    goal_theta = constrainAngle(
        std::atan2(possible_goals_[i][1] - latest_person_pose_[1],
                   possible_goals_[i][0] - latest_person_pose_[0]));

    // ROS_INFO("goal_dist: [%f]", goal_dist);
    if (goal_dist < min_distance_) {
      goal = possible_goals_[i];
      dist_to_goal = goal_dist;
      break;
    } else if (std::abs(constrainAngle(goal_theta - human_ori)) < min_theta) {
      goal = possible_goals_[i];
      min_theta = std::abs(constrainAngle(goal_theta - human_ori));
      dist_to_goal = goal_dist;
    }
    // std::cout << "Indx: " << i
    //           << "\t goal_theta: " << goal_theta
    //           << "\t human_ori: " << human_ori
    //           << "\t d_theta: " << std::abs(constrainAngle(goal_theta -
    //           human_ori)) << "\t Goal: (" << possible_goals_[i][0] << ", " <<
    //           possible_goals_[i][1] << ")" << std::endl;
  }
  // std::cout << "Actual goal chosen Goal: (" << goal[0] << ", " << goal[1] <<
  // ")" << std::endl;

  return goal;
}

float TrajectoryPrediction::getObstacleSpeed() {
  float obstacle_speed;

  // Determine the obstacle/human speed using either recent history or a default
  // value
  float history_time_diff =
      (detection_times_.back() - detection_times_.front()).toSec();
  // ROS_INFO("History length (s): %0.2f", history_time_diff);

  bool isHistoryReliable = history_time_diff > min_required_history_t_;
  if (isHistoryReliable) {
    std::array<double, 2> pos_1 = detection_positions_.front();
    std::array<double, 2> pos_2 = detection_positions_.back();

    float dist = std::sqrt(std::pow(pos_1[0] - pos_2[0], 2) +
                           std::pow(pos_1[1] - pos_2[1], 2));

    obstacle_speed = dist / history_time_diff;
    // ROS_INFO("History reliable. Speed is %0.2f", obstacle_speed);
  } else {
    // obstacle_speed = default_human_speed_;
    obstacle_speed = 0.0;
    // ROS_INFO("History not reliable. Speed is %0.2f", obstacle_speed);
  }

  return obstacle_speed;
}

bool TrajectoryPrediction::isLatestDetectionRecent() {
  // Get current time and compare with the last time a person was observed
  ros::Duration diff = ros::Time::now() - latest_person_msg_time_;

  // If the person wasn't recently observed, publish a message with no predicted
  // poses
  if (diff.toSec() > thresh_observed_t_secs_) {
    return false;
  } else {
    return true;
  }
}

void TrajectoryPrediction::plan() {
  // If people have not been detected or recently observed
  if (!person_detected_) {
    publishEmptyPrediction();
    ROS_INFO("No person has been detected yet.");
    return;
  } else if (!isLatestDetectionRecent()) {
    // All history is too old so clear it
    ROS_INFO("History is too old");
    detection_times_.clear();
    detection_positions_.clear();
    // ROS_INFO("No recent observation of a person.");
    publishEmptyPrediction();
    return;
  }

  // Since we have a recent observation, ensure we only have 'recent'
  // observations of the person that we can use
  pruneDetectionHistory();

  // With a pruned history, get the human's speed. If history
  // is long enough, this will be an avg, otherwise a default value
  float obstacle_speed = getObstacleSpeed();

  switch(prediction_mode_){

    case MP_MODE:{
      ROS_INFO("Using MP_MODE");

      // Now we perform intention recognition and determine the goal
      float dist_to_goal = 0;
      std::array<double, 2> obstacle_goal = getIntendedGoal(dist_to_goal);
      bool add_goal_factors =
          true;  // TODO - determine this based on how certain we are about the goal

      ROS_INFO("Obstacle speed: %0.2f", obstacle_speed);
      // ROS_INFO("Obstacle speed: %0.2f \t Goal: (%0.2f,%0.2f)", obstacle_speed,
      // obstacle_goal[0], obstacle_goal[1]); ROS_INFO("Goal: (%0.2f,%0.2f)",
      // obstacle_speed, obstacle_goal[0], obstacle_goal[1]);

      // If the speed is very low, assume the person is stationary
      // If person is very close to a goal, assume stationary
      bool speed_stationary_bool = (obstacle_speed < stationary_thresh_speed_);
      bool close_to_goal = (dist_to_goal < stationary_thresh_dist_);

      if (speed_stationary_bool || close_to_goal) {
        ROS_INFO("Stationary person detected. Stationary speed: %d \t Close to goal %d", (int) speed_stationary_bool, (int) close_to_goal);
        publishStationaryTrajectory();
        publishDesiredGoal(latest_person_pose_);
        visualiseStationaryPath();
        return;
      }

      // With the speed of the obstacle/human and the intended goal
      // we can now estimate the factor graph params
      reestimateTrajectoryParams(obstacle_speed, obstacle_goal, dist_to_goal);

      gtsam::Vector2 start_conf(latest_person_pose_[0], latest_person_pose_[1]);
      gtsam::Vector2 end_conf(obstacle_goal[0], obstacle_goal[1]);
      gtsam::Vector2 start_vel(0.0, 0.0);
      gtsam::Vector2 end_vel(0.0, 0.0);
      gtsam::Point3 obstacle_position(robot_odom_state_[0], robot_odom_state_[1], 0); // get this from vicon reading of HSR's position

      constructGraph(start_conf, start_vel, end_conf, end_vel, obstacle_position, add_goal_factors);

      // Init values
      gtsam::Values init_values = getInitTraj(start_conf, end_conf);
      gtsam::Values result = optimize(init_values);

      // Publish a msg of the predicted human trajectory
      publishPredictedTrajectory(result);
      publishDesiredGoal(obstacle_goal);
      visualisePrediction(result, setting_.total_step);
      break;
    }
    case CVM_MODE:{
      ROS_INFO("Using CVM_MODE");
      gtsam::Vector2 start_conf(latest_person_pose_[0], latest_person_pose_[1]);
      gtsam::Values result = getCVMTraj(start_conf, obstacle_speed);
      publishPredictedTrajectory(result);
      visualisePrediction(result, 30);
      break;
    }


  }

}

gtsam::Values TrajectoryPrediction::optimize(
    std::shared_ptr<gtsam::NonlinearOptimizer> opt,
    const gtsam::NonlinearOptimizerParams &params, bool iter_no_increase) {
  double currentError = opt->error();

  // check if we're already close enough
  if (currentError <= params.errorTol) {
    if (params.verbosity >= NonlinearOptimizerParams::ERROR)
      cout << "Exiting, as error = " << currentError << " < " << params.errorTol
           << endl;
    return opt->values();
  }

  // Return if we already have too many iterations
  if (opt->iterations() >= params.maxIterations) {
    if (params.verbosity >= NonlinearOptimizerParams::TERMINATION)
      cout << "iterations: " << opt->iterations() << " > "
           << params.maxIterations << endl;
    return opt->values();
  }

  Values last_values;

  // Iterative loop
  do {
    // iteration
    currentError = opt->error();
    // copy last values in case error increase
    if (iter_no_increase) last_values = opt->values();
    opt->iterate();
    // Maybe show output
    if (params.verbosity >= NonlinearOptimizerParams::ERROR)
      cout << "newError: " << opt->error() << endl;

  } while (opt->iterations() < params.maxIterations &&
           !checkConvergence(params.relativeErrorTol, params.absoluteErrorTol,
                             params.errorTol, currentError, opt->error(),
                             params.verbosity));

  // Printing if verbose
  if (params.verbosity >= NonlinearOptimizerParams::TERMINATION) {
    cout << "iterations: " << opt->iterations() << " > " << params.maxIterations
         << endl;
    if (opt->iterations() >= params.maxIterations)
      cout << "Terminating because reached maximum iterations" << endl;
  }

  // check whether values increase
  // if increase use last copied values
  if (opt->error() > currentError) {
    if (iter_no_increase) {
      if (params.verbosity >= NonlinearOptimizerParams::ERROR)
        cout << "Error increase, use last copied values" << endl;
      return last_values;
    } else {
      return opt->values();
    }
  } else {
    return opt->values();
  }
}

gtsam::Values TrajectoryPrediction::optimize(const gtsam::Values &init_values) {
  return gpmp2::optimize(graph_, init_values, setting_);
};

void TrajectoryPrediction::visualisePrediction(const gtsam::Values &plan,
                                               const size_t num_keys) const {
  nav_msgs::Path path;
  path.header.frame_id = global_frame_;

  for (size_t i = 0; i < num_keys; i++) {
    gtsam::Vector pose = plan.at<gtsam::Vector>(gtsam::Symbol('x', i));

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = global_frame_;
    pose_msg.pose.position.x = pose[0];
    pose_msg.pose.position.y = pose[1];
    pose_msg.pose.position.z = 0;

    path.poses.push_back(pose_msg);
  }

  path_vis_pub_.publish(path);
};

void TrajectoryPrediction::visualiseStationaryPath() const {
  nav_msgs::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = latest_person_msg_time_;

  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.frame_id = global_frame_;
  pose_msg.pose.position.x = latest_person_pose_[0];
  pose_msg.pose.position.y = latest_person_pose_[1];
  pose_msg.pose.position.z = 0;
  path.poses.push_back(pose_msg);

  path_vis_pub_.publish(path);
};

void TrajectoryPrediction::publishDesiredGoal(
    const std::array<double, 2> obstacle_goal) const {
  // Publish a msg indicating estimated goal of the person
  geometry_msgs::PointStamped desired_goal_msg;
  desired_goal_msg.header.frame_id = global_frame_;
  desired_goal_msg.point.x = obstacle_goal[0];
  desired_goal_msg.point.y = obstacle_goal[1];
  desired_goal_pub_.publish(desired_goal_msg);
}

void TrajectoryPrediction::publishEmptyPrediction() const {
  geometry_msgs::PoseArray predicted_trajectory;
  predicted_trajectory.header.frame_id = global_frame_;
  predicted_trajectory.header.stamp = ros::Time::now();
  predicted_traj_pub_.publish(predicted_trajectory);
}

void TrajectoryPrediction::publishStationaryTrajectory() const {
  geometry_msgs::PoseArray predicted_trajectory;
  predicted_trajectory.header.frame_id = global_frame_;
  predicted_trajectory.header.stamp = latest_person_msg_time_;

  geometry_msgs::Pose p;
  p.position.x = latest_person_pose_[0];
  p.position.y = latest_person_pose_[1];
  predicted_trajectory.poses.push_back(p);

  predicted_traj_pub_.publish(predicted_trajectory);
}

void TrajectoryPrediction::publishPredictedTrajectory(
    const gtsam::Values &result) const {
  geometry_msgs::PoseArray predicted_trajectory;
  predicted_trajectory.header.frame_id = global_frame_;
  predicted_trajectory.header.stamp = latest_person_msg_time_;

  gtsam::Vector predicted_pose;
  geometry_msgs::Pose p;
  for (int i = 0; i < setting_.total_step; i++) {
    predicted_pose = result.at<gtsam::Vector>(gtsam::Symbol('x', i));
    p.position.x = predicted_pose[0];
    p.position.y = predicted_pose[1];
    predicted_trajectory.poses.push_back(p);
  }

  predicted_traj_pub_.publish(predicted_trajectory);
}
