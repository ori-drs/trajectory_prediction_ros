#include <trajectory_prediction/trajectory_prediction.h>

TrajectoryPrediction::TrajectoryPrediction(ros::NodeHandle node){
  node_ = node;
  sub_ = node_.subscribe("/vicon/person_1/person_1", 
                                        100,
                                        &TrajectoryPrediction::personPoseCallback, 
                                        this);

  node_.param<std::string>("goal_topic", goal_topic_, "desired_goal");
  node_.param<std::string>("predicted_traj_topic", predicted_traj_topic_, "predicted_path");
  node_.param<std::string>("global_frame", global_frame_, "/vicon/world");

  desired_goal_pub_ = node_.advertise<geometry_msgs::PointStamped>(goal_topic_, 100);
  predicted_traj_pub = node_.advertise<geometry_msgs::PoseArray>(predicted_traj_topic_, 100);

}

gtsam::Values TrajectoryPrediction::constructGraph(const gpmp2::PointRobotModel &arm, const gtsam::Vector &start_conf, const gtsam::Vector &start_vel,
                              const gtsam::Vector &end_conf, const gtsam::Vector &end_vel) {
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Point2 origin(0.0, 0.0);
  double cell_size = 0.1;
  gtsam::Matrix data = Eigen::MatrixXd::Zero(300, 300);
  gpmp2::PlanarSDF sdf = gpmp2::PlanarSDF(origin, cell_size, data);

  gtsam::Matrix2 Qc = 0.1 * gtsam::Matrix::Identity(2, 2);
  gpmp2::TrajOptimizerSetting setting_ = gpmp2::TrajOptimizerSetting(2);
  setting_.setLM();
  setting_.set_total_step(10);
  setting_.set_total_time(10);
  setting_.set_epsilon(0.2);
  setting_.set_cost_sigma(0.1);
  setting_.set_obs_check_inter(1);
  setting_.set_conf_prior_model(0.1);
  setting_.set_vel_prior_model(0.001);
  setting_.set_Qc_model(Qc);
  // setting_.setVerbosityError();
  setting_.setVerbosityNone();
  gtsam::Values init_values;
  // GP interpolation setting
  const double delta_t =
      setting_.total_time / static_cast<double>(setting_.total_step);
  const double inter_dt =
      delta_t / static_cast<double>(setting_.obs_check_inter + 1);
  gtsam::Vector avg_vel =
      (end_conf - start_conf) / setting_.total_step / delta_t;

  for (size_t i = 0; i < setting_.total_step; i++) {
    gtsam::Key pose_key = gtsam::Symbol('x', i);
    gtsam::Key vel_key = gtsam::Symbol('v', i);

    gtsam::Vector pose =
        start_conf + (end_conf - start_conf) * i / (setting_.total_step - 1);

    init_values.insert(gtsam::Symbol('x', i), pose);
    init_values.insert(gtsam::Symbol('v', i), avg_vel);

    // start and end
    if (i == 0) {
      graph_.add(gtsam::PriorFactor<gtsam::Vector>(
          pose_key, start_conf, setting_.conf_prior_model));
      graph_.add(gtsam::PriorFactor<gtsam::Vector>(vel_key, start_vel,
                                                    setting_.vel_prior_model));

    } else if (i == setting_.total_step - 1) {
      graph_.add(gtsam::PriorFactor<gtsam::Vector>(
          pose_key, end_conf, setting_.conf_prior_model));
      graph_.add(gtsam::PriorFactor<gtsam::Vector>(vel_key, end_vel,
                                                    setting_.vel_prior_model));
    }

    // non-interpolated cost factor
    graph_.add(gpmp2::ObstaclePlanarSDFFactorPointRobot(
        pose_key, arm, sdf, setting_.cost_sigma, setting_.epsilon));

    if (i > 0) {
      gtsam::Key last_pose_key = gtsam::Symbol('x', i - 1);
      gtsam::Key last_vel_key = gtsam::Symbol('v', i - 1);

      // GP factor
      graph_.add(gpmp2::GaussianProcessPriorLinear(
          last_pose_key, last_vel_key, pose_key, vel_key, delta_t,
          setting_.Qc_model));
    }
  }

  // optimize!
  std::shared_ptr<gtsam::NonlinearOptimizer> opt_ptr;
  std::shared_ptr<gtsam::NonlinearOptimizerParams> opt_params_ptr;

  opt_params_ptr = std::shared_ptr<gtsam::NonlinearOptimizerParams>(
      new GaussNewtonParams());

  // common settings
  opt_params_ptr->setMaxIterations(setting_.max_iter);
  opt_params_ptr->setRelativeErrorTol(setting_.rel_thresh);

  // optimizer
  opt_ptr =
      std::shared_ptr<gtsam::NonlinearOptimizer>(new GaussNewtonOptimizer(
          graph_, init_values,
          *(dynamic_cast<GaussNewtonParams *>(opt_params_ptr.get()))));
  return optimize(opt_ptr, *opt_params_ptr, 0);
}

void TrajectoryPrediction::personPoseCallback( const geometry_msgs::TransformStamped::ConstPtr &msg) {

  std::array<double, 2> pose_current = {msg->transform.translation.x,
                                        msg->transform.translation.y};
  double goal_x, goal_y;

  double human_ori = std::atan2(pose_current[1] - pose_past_[1],
                                pose_current[0] - pose_past_[0]);
  pose_past_ = pose_current;

  double min_theta = 3.15;
  for (int i = 0; i < possible_goals_.size(); ++i) {
    double goal_dist =
        std::sqrt(std::pow(possible_goals_[i][0] - pose_current[0], 2) +
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

  gpmp2::PointRobotModel robot = gpmp2::RobotModel<gpmp2::PointRobot>();

  gtsam::Vector2 start_conf(pose_current[0], pose_current[1]);
  gtsam::Vector2 end_conf(goal_x, goal_y);
  gtsam::Vector2 start_vel(0.0, 0.0);
  gtsam::Vector2 end_vel(0.0, 0.0);

  gtsam::Values result;
  result = constructGraph(robot, start_conf, start_vel, end_conf, end_vel);
  result.print("Final Result:\n");

  gpmp2::TrajOptimizerSetting setting_ = gpmp2::TrajOptimizerSetting(2);
  setting_.set_total_step(10);
  
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
