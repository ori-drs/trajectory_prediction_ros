#ifndef TRAJECTORY_PREDICTION_H
#define TRAJECTORY_PREDICTION_H

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
#include "std_msgs/Float32MultiArray.h"
#include "tf/transform_listener.h"
#include <nav_msgs/Path.h>

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

#include <mutex>

// Constrain angles between -pi and pi
float constrainAngle(float x);

class TrajectoryPrediction {
    public:

        // TrajectoryPrediction() {}
        TrajectoryPrediction(ros::NodeHandle node);

        ~TrajectoryPrediction() {}
        
        void distanceFieldCallback( const std_msgs::Float32MultiArray::ConstPtr &msg);
        void viconPersonPoseCallback(const geometry_msgs::TransformStamped::ConstPtr &msg);
        void cameraPersonPoseCallback(const geometry_msgs::PointStamped::ConstPtr &msg);
        void pruneDetectionHistory();
        bool isLatestDetectionRecent();

        std::array<double, 2> getIntendedGoal(float &dist_to_goal);
        float getObstacleSpeed();

        void reestimateTrajectoryParams(const float obstacle_speed, const std::array<double, 2> obstacle_goal, const float dist_to_goal);
        void createSettings(float total_time = 10, int total_time_step = 10);
        void constructGraph(const gtsam::Vector &start_conf, const gtsam::Vector &start_vel, const gtsam::Vector &end_conf, const gtsam::Vector &end_vel, const bool add_goal_factors);
        void plan();

        gtsam::Values getInitTraj(const gtsam::Vector &start_conf, const gtsam::Vector &end_conf);
        gtsam::Values optimize(const gtsam::Values& init_values);
        gtsam::Values optimize(std::shared_ptr<gtsam::NonlinearOptimizer> opt, 
                            const gtsam::NonlinearOptimizerParams &params,
                                bool iter_no_increase);


        void visualisePrediction(const gtsam::Values& plan, const size_t num_keys) const;
        void visualiseStationaryPath() const;
        void publishDesiredGoal(const std::array<double, 2> obstacle_goal) const;

        void publishEmptyPrediction() const;
        void publishPredictedTrajectory(const gtsam::Values &result) const;
        void publishStationaryTrajectory() const;

    private:
        
        // ROS
        ros::NodeHandle node_;
        std::string goal_topic_, predicted_traj_topic_, global_frame_, path_vis_topic_, distance_field_topic_, person_position_topic_;
        ros::Subscriber sub_, distance_field_sub_;
        ros::Publisher desired_goal_pub_, predicted_traj_pub_, path_vis_pub_;
        std_msgs::Float32MultiArray::ConstPtr latest_msg_;
        ros::Time latest_person_msg_time_;
        bool person_detected_ = false;

        // std::array<std::array<double, 2>, 4> possible_goals_{
        //                                                     {{{0, 4}}, 
        //                                                     {{1.6, -3}}, 
        //                                                     {{-0.73, -2.4}}, 
        //                                                     {{-1.82, 0.62}}}};

        // Intention
        // std::array<std::array<double, 2>, 1> possible_goals_{{{{3, -2}}}};
        std::array<double, 2> latest_person_pose_;
        float thresh_observed_t_secs_ = 5.0; // seconds
        float min_required_history_t_ = 1.0; // seconds
        float default_human_speed_ = 4.0; // m/s
        float stationary_thresh_speed_ = 0.1; // m/s
        float stationary_thresh_dist_ = 0.5; // m/s

        std::vector<ros::Time> detection_times_;
        std::vector<std::array<double, 2>>detection_positions_;

        std::array<std::array<double, 2>, 2> possible_goals_{
                                                                {
                                                                    {{5, -2}},
                                                                    {{1.0, 0}}
                                                                    }
                                                            };

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

        // Graph params
        double resolution_;
        double delta_t_, inter_dt_;
        double min_distance_ = 0.8;
        
        std::mutex data_mutex_;
};

#endif