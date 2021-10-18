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

        // TrajectoryPrediction() {}
        TrajectoryPrediction(ros::NodeHandle node);

        ~TrajectoryPrediction() {}

        void personPoseCallback(const geometry_msgs::TransformStamped::ConstPtr &msg);

        gtsam::Values constructGraph(const gpmp2::PointRobotModel &arm, const gtsam::Vector &start_conf, const gtsam::Vector &start_vel,
                                    const gtsam::Vector &end_conf, const gtsam::Vector &end_vel);

        gtsam::Values optimize(std::shared_ptr<gtsam::NonlinearOptimizer> opt, 
                            const gtsam::NonlinearOptimizerParams &params,
                                bool iter_no_increase);

    private:
        ros::NodeHandle node_;
        std::string goal_topic_, predicted_traj_topic_, global_frame_;
        ros::Subscriber sub_;
        ros::Publisher desired_goal_pub_, predicted_traj_pub;
        std::array<std::array<double, 2>, 4> possible_goals_{
                                                            {{{0, 4}}, 
                                                            {{1.6, -3}}, 
                                                            {{-0.73, -2.4}}, 
                                                            {{-1.82, 0.62}}}};
        std::array<double, 2> pose_past_ = {0, 0};
        double min_distance_ = 0.8;

};


#endif