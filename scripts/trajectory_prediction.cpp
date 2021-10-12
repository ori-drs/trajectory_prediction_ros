#include <finean_msgs/PersonPosition.h>
#include <finean_msgs/PersonPositions.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <array>
#include <cmath>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_listener.h"

#include <gpmp2/kinematics/PointRobot.h>
#include <gpmp2/kinematics/PointRobotModel.h>
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

class TrajectoryPrediction {
 public:
  ros::NodeHandle n;
  ros::Publisher desired_goal_pub_ =
      n.advertise<geometry_msgs::PointStamped>("/desired_goal", 100);
  ros::Publisher predicted_traj_pub =
      n.advertise<geometry_msgs::PoseArray>("/predicted_path", 100);
  std::array<std::array<double, 2>, 4> possible_goals_{
      {{{0, 4}}, {{1.6, -3}}, {{-0.73, -2.4}}, {{-1.82, 0.62}}}};
  std::array<double, 2> pose_past_ = {0, 0};

  //   int history_length = 2;
  //   std::vector<double> pose_history(history_length*2, 0.0);
  //    geometry_msgs::PoseArray  possible_goals;

  TrajectoryPrediction() {}

  gtsam::Values constructGraph(const gpmp2::PointRobotModel &arm,
                               const gtsam::Vector &start_conf,
                               const gtsam::Vector &start_vel,
                               const gtsam::Vector &end_conf,
                               const gtsam::Vector &end_vel) {
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

  void personPoseCallback(
      const geometry_msgs::TransformStamped::ConstPtr &msg) {
    std::array<double, 2> pose_current = {msg->transform.translation.x,
                                          msg->transform.translation.y};
    double goal_x, goal_y;
    geometry_msgs::PointStamped desired_goal_msg;
    desired_goal_msg.header.frame_id = "/vicon/world";
    double human_ori = std::atan2(pose_current[1] - pose_past_[1],
                                  pose_current[0] - pose_past_[0]);
    pose_past_ = pose_current;

    double min_distance = 0.8;
    double min_theta = 3.15;
    for (int i = 0; i < possible_goals_.size(); ++i) {
      double goal_dist =
          std::sqrt(std::pow(possible_goals_[i][0] - pose_current[0], 2) +
                    std::pow(possible_goals_[i][1] - pose_current[1], 2));
      double goal_theta = std::atan2(possible_goals_[i][1] - pose_current[1],
                                     possible_goals_[i][0] - pose_current[0]);
      // ROS_INFO("goal_dist: [%f]", goal_dist);
      if (goal_dist < min_distance) {
        goal_x = possible_goals_[i][0];
        goal_y = possible_goals_[i][1];
        break;
      } else if (std::abs(goal_theta - human_ori) < min_theta) {
        goal_x = possible_goals_[i][0];
        goal_y = possible_goals_[i][1];
        min_theta = std::abs(goal_theta - human_ori);
      }
    }
    desired_goal_msg.point.x = goal_x;
    desired_goal_msg.point.y = goal_y;
    desired_goal_pub_.publish(desired_goal_msg);
    gpmp2::PointRobot pR = gpmp2::PointRobot(2, 1);

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
    predicted_trajectory.header.frame_id = "/vicon/world";
    gtsam::Vector predicted_pose;
    geometry_msgs::Pose p;
     for (int i = 0; i < setting_.total_step; i++) {
      predicted_pose = result.at<gtsam::Vector>(gtsam::Symbol('x', i));
      p.position.x = predicted_pose[0];
      p.position.y = predicted_pose[1];
      predicted_trajectory.poses.push_back(p);
    }

    predicted_traj_pub.publish(predicted_trajectory);
  }

  gtsam::Values optimize(std::shared_ptr<gtsam::NonlinearOptimizer> opt,
                         const gtsam::NonlinearOptimizerParams &params,
                         bool iter_no_increase) {
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
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_prediction");
  TrajectoryPrediction trajPred;

  ros::Subscriber sub = trajPred.n.subscribe(
      "/vicon/person_1/person_1", 100,
      &TrajectoryPrediction::personPoseCallback, &trajPred);

  ros::Rate loop_rate(10);  // 10 Hz

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

/*
        dtheta = []
        distance = []
        for i in range(len(self.possible_goals_list)):
            distance.append(goal_dist)
            dtheta.append(abs(goal_theta - human_ori))
        min_goal_dist, idx = min((val, idx) for (idx, val) in
   enumerate(distance)) if min_goal_dist > 0.8: min_theta, idx = min((val, idx)
   for (idx, val) in enumerate(dtheta)) self.desired_goal.point.x =
   self.possible_goals_list[idx][0] self.desired_goal.point.y =
   self.possible_goals_list[idx][1] if not(self.last_idx == idx):
            self.gpmp2Prediction()
        self.last_idx = idx
*/

// gtsam::Values result = gtsam::GaussNewtonOptimizer(graph_,
// init_values).optimize();
