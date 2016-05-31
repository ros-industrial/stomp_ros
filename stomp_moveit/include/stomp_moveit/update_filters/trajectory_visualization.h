/*
 * trajectory_visualization.h
 *
 *  Created on: Apr 14, 2016
 *      Author: Jorge Nicho
 */

#ifndef INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_UPDATE_FILTERS_TRAJECTORY_VISUALIZATION_H_
#define INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_UPDATE_FILTERS_TRAJECTORY_VISUALIZATION_H_

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <Eigen/Core>
#include <geometry_msgs/Point.h>
#include <stomp_moveit/update_filters/stomp_update_filter.h>
#include <visualization_msgs/Marker.h>

namespace stomp_moveit
{
namespace update_filters
{

class TrajectoryVisualization : public StompUpdateFilter
{
public:
  TrajectoryVisualization();
  virtual ~TrajectoryVisualization();

  virtual bool initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                          const std::string& group_name,const XmlRpc::XmlRpcValue& config) override;

  virtual bool configure(const XmlRpc::XmlRpcValue& config) override;

  virtual bool setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   const stomp_core::StompConfiguration &config,
                   moveit_msgs::MoveItErrorCodes& error_code) override;

  /**
   * @brief visualizes the trajectory.
   *
   * @param start_timestep    start index into the 'parameters' array, usually 0.
   * @param num_timesteps     number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param parameters        The parameters generated in the previous iteration [num_dimensions] x [num_timesteps]
   * @param updates           The updates to be applied to the parameters [num_dimensions] x [num_timesteps]
   * @param filtered          set ot 'true' if the updates were modified.
   * @return false if something failed
   */
  virtual bool filter(std::size_t start_timestep,
                      std::size_t num_timesteps,
                      int iteration_number,
                      const Eigen::MatrixXd& parameters,
                      Eigen::MatrixXd& updates,
                      bool& filtered) override;

  /**
   * @brief Called by the Stomp at the end of the optimization process
   *
   * @param success           Whether the optimization succeeded
   * @param total_iterations  Number of iterations used
   * @param final_cost        The cost value after optimizing.
   */
  virtual void done(bool success,int total_iterations,double final_cost) override;


  virtual std::string getName() const override
  {
    return name_ + "/" + group_name_;
  }


  virtual std::string getGroupName() const override
  {
    return group_name_;
  }


protected:

  // identity
  std::string name_;

  // robot
  std::string group_name_;
  moveit::core::RobotModelConstPtr robot_model_;
  moveit::core::RobotStatePtr state_;

  // ros comm
  ros::NodeHandle nh_;
  ros::Publisher viz_pub_;

  // parameters
  double line_width_;
  std_msgs::ColorRGBA rgb_;
  std_msgs::ColorRGBA error_rgb_;
  bool publish_intermediate_;
  std::string marker_topic_;
  std::string marker_namespace_;

  // tool trajectory
  Eigen::MatrixXd tool_traj_line_;
  visualization_msgs::Marker tool_traj_marker_;


};

} /* namespace filters */
} /* namespace stomp_moveit */

#endif /* INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_UPDATE_FILTERS_TRAJECTORY_VISUALIZATION_H_ */