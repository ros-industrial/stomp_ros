/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Henning Kayser
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Raghavender Sahdev nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Henning Kayser */

// ROS
#include <ros/ros.h>
#include <class_loader/class_loader.hpp>

// MoveIt
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/planning_interface/planning_interface.h>

// STOMP
#include <stomp_moveit/stomp_planner.h>

namespace stomp_moveit
{
class StompSmoothingAdapter : public planning_request_adapter::PlanningRequestAdapter
{
public:
  StompSmoothingAdapter() : planning_request_adapter::PlanningRequestAdapter()
  {
    ros::NodeHandle nh("~");
    initialize(nh);
  }

  // todo[noetic] add override again
  virtual void initialize(const ros::NodeHandle& node_handle)
  {
    ros::NodeHandle nh(node_handle);
    if (!StompPlanner::getConfigData(nh, group_config_))
      ROS_ERROR("Unable to find valid group config for StompSmoothingAdapter");
  }

  virtual std::string getDescription() const override
  {
    return "Stomp Smoothing Adapter";
  }

  bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& ps,
                    const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
                    std::vector<std::size_t>& added_path_index) const override
  {
    // Following call to planner() calls the OMPL planner and stores the trajectory inside the MotionPlanResponse res
    // variable which is then used by STOMP for optimization of the computed trajectory
    if (!planner(ps, req, res))
      return false;

    // STOMP reads the seed trajectory from trajectory constraints so we need to convert the waypoints first
    const size_t seed_waypoint_count = res.trajectory_->getWayPointCount();
    const std::vector<std::string> variable_names =
      res.trajectory_->getFirstWayPoint().getJointModelGroup(req.group_name)->getVariableNames();
    const size_t variable_count = variable_names.size();
    planning_interface::MotionPlanRequest seed_req = req;
    seed_req.trajectory_constraints.constraints.clear();
    seed_req.trajectory_constraints.constraints.resize(seed_waypoint_count);
    for (size_t i = 0; i < seed_waypoint_count; ++i)
    {
      seed_req.trajectory_constraints.constraints[i].joint_constraints.resize(variable_count);
      for (size_t j = 0; j < variable_count; ++j)
      {
        seed_req.trajectory_constraints.constraints[i].joint_constraints[j].joint_name = variable_names[j];
        seed_req.trajectory_constraints.constraints[i].joint_constraints[j].position =
          res.trajectory_->getWayPoint(i).getVariablePosition(variable_names[j]);
      }
    }

    // Get group config
    const auto& group_config_it = group_config_.find(req.group_name);
    if (group_config_it == group_config_.end())
    {
      ROS_ERROR_STREAM("STOMP is not configured for planning group " << req.group_name);
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
      return false;
    }

    // Initialize STOMP Planner
    stomp_moveit::StompPlanner stomp_planner(req.group_name, group_config_it->second, ps->getRobotModel());
    if(!stomp_planner.canServiceRequest(seed_req))
    {
      ROS_ERROR("STOMP planner unable to service request");
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
      return false;
    }

    // Setup Planning Context
    stomp_planner.clear();
    stomp_planner.setPlanningScene(ps);
    stomp_planner.setMotionPlanRequest(seed_req);

    // Solve
    ROS_DEBUG("Smoothing result trajectory with STOMP");
    planning_interface::MotionPlanDetailedResponse stomp_res;
    bool success = stomp_planner.solve(stomp_res);
    if (success)
    {
      // Successful responses always contain one entry for trajectory_ and proccessing_time_
      res.trajectory_ = stomp_res.trajectory_.back();
      res.planning_time_ += stomp_res.processing_time_.back();
    }
    res.error_code_ = stomp_res.error_code_;
    return success;
  }

private:
  std::map<std::string, XmlRpc::XmlRpcValue> group_config_;
};
}  // namespace stomp_moveit

CLASS_LOADER_REGISTER_CLASS(stomp_moveit::StompSmoothingAdapter, planning_request_adapter::PlanningRequestAdapter);
