#include <trajectory_prediction/trajectory_prediction.h>

TrajectoryPrediction::TrajectoryPrediction(ros::NodeHandle node){

  node_ = node;

  ROS_INFO("Initialising trajectory prediction node.");


  node_.param<std::string>("goal_topic", goal_topic_, "desired_goal");
  node_.param<std::string>("predicted_traj_topic", predicted_traj_topic_, "predicted_path");
  node_.param<std::string>("path_vis_topic", path_vis_topic_, "predicted_rviz_path");
  node_.param<std::string>("global_frame", global_frame_, "/vicon/world");
  node_.param<std::string>("distance_field_topic", distance_field_topic_, "/distance_field_2d");
  node_.param<std::string>("person_position_topic", person_position_topic_, "/vicon/person_1/person_1");
  node_.param<bool>("use_rgbd", use_rgbd_, true);

  node_.param<double>("resolution", resolution_, 0.05);
  node_.param<int>("num_rows", num_rows_, 256);
  node_.param<bool>("use_empty_distance_field", use_empty_distance_field_, true);

  df_initialised_ = false;

  gpmp2::BodySphereVector body_spheres;
  body_spheres.push_back(gpmp2::BodySphere(0, 0.6, Point3(0, 0, 0)));
  robot_ = gpmp2::RobotModel<gpmp2::PointRobot>(gpmp2::PointRobot(2,1), body_spheres);

 if (use_rgbd_)
 {
  sub_ = node_.subscribe(person_position_topic_, 
                                        100,
                                        &TrajectoryPrediction::cameraPersonPoseCallback, 
                                        this);
 }
 else{
  sub_ = node_.subscribe(person_position_topic_, 
                                    100,
                                    &TrajectoryPrediction::viconPersonPoseCallback, 
                                    this);
 }
 


  distance_field_sub_ = node_.subscribe(distance_field_topic_, 
                                        100,
                                        &TrajectoryPrediction::distanceFieldCallback, 
                                        this);

  desired_goal_pub_ = node_.advertise<geometry_msgs::PointStamped>(goal_topic_, 100);
  predicted_traj_pub = node_.advertise<geometry_msgs::PoseArray>(predicted_traj_topic_, 100);
  path_vis_pub_ = node_.advertise<nav_msgs::Path>(path_vis_topic_, 1000);

  createSettings();
  ROS_INFO("Trajectory prediction node initialised.");

}

void TrajectoryPrediction::createSettings(float total_time, int total_time_step){
  gtsam::Matrix2 Qc = 0.1 * gtsam::Matrix::Identity(2, 2);
  setting_ = gpmp2::TrajOptimizerSetting(2);
  setting_.setLM();
  setting_.set_total_step(20);
  setting_.set_total_time(10);
  setting_.set_epsilon(0.3);
  setting_.set_cost_sigma(0.2);
  setting_.set_obs_check_inter(3);
  setting_.set_conf_prior_model(0.001);
  setting_.set_vel_prior_model(0.001);
  setting_.set_Qc_model(Qc);
  // setting_.setVerbosityError();
  setting_.setVerbosityNone();
  setting_.set_rel_thresh(1e-2);

  delta_t_ = setting_.total_time / static_cast<double>(setting_.total_step);
  inter_dt_ = delta_t_ / static_cast<double>(setting_.obs_check_inter + 1);
}

void TrajectoryPrediction::constructGraph(const gtsam::Vector &start_conf, const gtsam::Vector &start_vel, const gtsam::Vector &end_conf, const gtsam::Vector &end_vel) {
  
  std::lock_guard<std::mutex> lock(data_mutex_);
  graph_ = gtsam::NonlinearFactorGraph();

  std::cout << "epsilon: " << setting_.epsilon << std::endl;

  if (!use_empty_distance_field_ && df_initialised_){
    std::vector<double> vec_data(latest_msg_->data.begin(), latest_msg_->data.end());
    gtsam::Point2 origin(-(num_rows_/2.0)*resolution_, -(num_rows_/2.0) * resolution_);
    
    
    
    std::cout << "--------------------------------------------" << std::endl;
    std::cout << "Original" << std::endl;
    data_ = Eigen::Map<gtsam::Matrix>(&vec_data[0], num_rows_, num_rows_) ; // Assume num_rows and num_cols are equal
    data_.transposeInPlace();

    sdf_ = gpmp2::PlanarSDF(origin, resolution_, data_);
  }
  else{
    data_ = Eigen::MatrixXd::Zero(num_rows_, num_rows_);
    gtsam::Point2 origin(0.0, 0.0);
    sdf_ = gpmp2::PlanarSDF(origin, resolution_, data_);
  }
  
  for (size_t i = 0; i < setting_.total_step; i++) {
    gtsam::Key pose_key = gtsam::Symbol('x', i);
    gtsam::Key vel_key = gtsam::Symbol('v', i);

    // start and end
    if (i == 0) {
      graph_.add(gtsam::PriorFactor<gtsam::Vector>(
          pose_key, start_conf, setting_.conf_prior_model));
      graph_.add(gtsam::PriorFactor<gtsam::Vector>(vel_key, start_vel, setting_.vel_prior_model));

    } else if (i == setting_.total_step - 1) {
      graph_.add(gtsam::PriorFactor<gtsam::Vector>(
          pose_key, end_conf, setting_.conf_prior_model));
      graph_.add(gtsam::PriorFactor<gtsam::Vector>(vel_key, end_vel, setting_.vel_prior_model));
    }


    if (i > 0) {
      gtsam::Key last_pose_key = gtsam::Symbol('x', i - 1);
      gtsam::Key last_vel_key = gtsam::Symbol('v', i - 1);
      
      // GP factor
      graph_.add(gpmp2::GaussianProcessPriorLinear(
          last_pose_key, last_vel_key, pose_key, vel_key, delta_t_,
          setting_.Qc_model));

      // non-interpolated cost factor
      graph_.add(gpmp2::ObstaclePlanarSDFFactorPointRobot(
        pose_key, robot_, sdf_, setting_.cost_sigma, setting_.epsilon));

      // interpolated cost factor
      if (setting_.obs_check_inter > 0) {
        for (size_t j = 1; j <= setting_.obs_check_inter; j++) {
          const double tau = inter_dt_ * static_cast<double>(j);
          graph_.add(gpmp2::ObstaclePlanarSDFFactorGPPointRobot(last_pose_key, last_vel_key, pose_key, vel_key, robot_, sdf_,
              setting_.cost_sigma, setting_.epsilon, setting_.Qc_model, delta_t_, tau));
        }
      }


    }
  }
}

void TrajectoryPrediction::distanceFieldCallback( const std_msgs::Float32MultiArray::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_msg_ = msg;
  df_initialised_ = true;
}

void TrajectoryPrediction::viconPersonPoseCallback(const geometry_msgs::TransformStamped::ConstPtr &msg) {
  std::array<double, 2> pose_current = {msg->transform.translation.x, msg->transform.translation.y};
                                        
  this->plan(pose_current);
}

void TrajectoryPrediction::cameraPersonPoseCallback(const geometry_msgs::PointStamped::ConstPtr &msg) {
  std::array<double, 2> pose_current = {msg->point.x, msg->point.y};

  this->plan(pose_current);
}

gtsam::Values TrajectoryPrediction::getInitTraj(const gtsam::Vector &start_conf, const gtsam::Vector &end_conf) {

  gtsam::Values init_values;
  
  gtsam::Vector avg_vel = (end_conf - start_conf) / setting_.total_step / delta_t_;

  for (size_t i = 0; i < setting_.total_step; i++) {
    gtsam::Vector pose = start_conf + (end_conf - start_conf) * i / (setting_.total_step - 1);

    init_values.insert(gtsam::Symbol('x', i), pose);
    init_values.insert(gtsam::Symbol('v', i), avg_vel);
  }

  return init_values;
}

void TrajectoryPrediction::plan(std::array<double, 2> pose_current) {
                                        
  double goal_x, goal_y;

  double human_ori = std::atan2(pose_current[1] - pose_past_[1],
                                pose_current[0] - pose_past_[0]);
  pose_past_ = pose_current;

  double min_theta = 3.15;
  for (int i = 0; i < possible_goals_.size(); ++i) {
    
    double goal_dist = std::sqrt(std::pow(possible_goals_[i][0] - pose_current[0], 2) +
                                 std::pow(possible_goals_[i][1] - pose_current[1], 2));

    double goal_theta = std::atan2(possible_goals_[i][1] - pose_current[1],
                                    possible_goals_[i][0] - pose_current[0]);

    // ROS_INFO("goal_dist: [%f]", goal_dist);
    if (goal_dist < min_distance_) {
      goal_x = possible_goals_[i][0];
      goal_y = possible_goals_[i][1];
      break;
    } else if (std::abs(goal_theta - human_ori) < min_theta) {
      goal_x = possible_goals_[i][0];
      goal_y = possible_goals_[i][1];
      min_theta = std::abs(goal_theta - human_ori);
    }
  }

  gtsam::Vector2 start_conf(pose_current[0], pose_current[1]);
  gtsam::Vector2 end_conf(goal_x, goal_y);
  gtsam::Vector2 start_vel(0.0, 0.0);
  gtsam::Vector2 end_vel(0.0, 0.0);

  constructGraph(start_conf, start_vel, end_conf, end_vel);
  
  // Init values
  gtsam::Values init_values = getInitTraj(start_conf, end_conf);
  gtsam::Values result = optimize(init_values);

  // Publish a msg of the predicted human trajectory

  geometry_msgs::PoseArray predicted_trajectory;
  predicted_trajectory.header.frame_id = global_frame_;
  
  gtsam::Vector predicted_pose;
  geometry_msgs::Pose p;
  for (int i = 0; i < setting_.total_step; i++) {
    predicted_pose = result.at<gtsam::Vector>(gtsam::Symbol('x', i));
    p.position.x = predicted_pose[0];
    p.position.y = predicted_pose[1];
    predicted_trajectory.poses.push_back(p);
  }

  predicted_traj_pub.publish(predicted_trajectory);

  // Publish a msg indicating estimated goal of the person
  geometry_msgs::PointStamped desired_goal_msg;
  desired_goal_msg.header.frame_id = global_frame_;
  desired_goal_msg.point.x = goal_x;
  desired_goal_msg.point.y = goal_y;
  desired_goal_pub_.publish(desired_goal_msg);

  visualisePrediction(result, setting_.total_step);
}

gtsam::Values TrajectoryPrediction::optimize(std::shared_ptr<gtsam::NonlinearOptimizer> opt, const gtsam::NonlinearOptimizerParams &params, bool iter_no_increase) {
    double currentError = opt->error();

    // check if we're already close enough
    if (currentError <= params.errorTol) {
      if (params.verbosity >= NonlinearOptimizerParams::ERROR)
        cout << "Exiting, as error = " << currentError << " < "
             << params.errorTol << endl;
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
      cout << "iterations: " << opt->iterations() << " > "
           << params.maxIterations << endl;
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

gtsam::Values TrajectoryPrediction::optimize(const gtsam::Values& init_values){

  return gpmp2::optimize(graph_, init_values, setting_);
};

void TrajectoryPrediction::visualisePrediction(const gtsam::Values& plan, const size_t num_keys) const{
    nav_msgs::Path path;
    path.header.frame_id = global_frame_;

    for (size_t i = 0; i < num_keys; i++)
    {
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
