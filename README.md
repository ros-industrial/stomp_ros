# stomp_ros

[![Build Status: Ubuntu Bionic (Actions)](https://github.com/ros-industrial/stomp_ros/workflows/CI%20-%20Ubuntu%20Bionic/badge.svg?branch=melodic-devel)](https://github.com/ros-industrial/stomp_ros/actions?query=workflow%3A%22CI+-+Ubuntu+Bionic%22)
[![Build Status: Ubuntu Focal (Actions)](https://github.com/ros-industrial/stomp_ros/workflows/CI%20-%20Ubuntu%20Focal/badge.svg?branch=melodic-devel)](https://github.com/ros-industrial/stomp_ros/actions?query=workflow%3A%22CI+-+Ubuntu+Focal%22)

#### Build
- Build the workspace:
  - Cd into the catkin workspace directory and type the following command:
```
catkin build
```

#### Seeding Stomp
The STOMP planner works through optimization: it starts with a given trajectory, called the ***seed***, and iteratively attempts to improve it. This seed is set:
 1. By default, it is set to the joint interpolated path between the start and end joint configurations.
 2. If you wish, you can set your own seed trajectory.

The `StompPlanner` class works off of the `moveit_msgs/MotionPlanRequest` message type which does not provide an interface for seeds. Until that is added, we bastardize the unused `MotionPlanRequest::trajectory_constraints` field to serve this purpose. Use the `StompPlanner::encodeSeedTrajectory(const trajectory_msgs::JointTrajectory& seed)` static function to do this:

```c++

StompPlanner planner = makeStompPlanner(); // However you initialize
planning_interface::MotionPlanRequest request;

// set your nominal goals, start conditions, etc...

trajectory_msgs::JointTrajectory seed_traj; // Look up your seed traj
request.trajectory_constraints = StompPlanner::encodeSeedTrajectory(seed_traj);

// Call the planning service or the planner itself
planner.setMotionPlanRequest(request)

MotionPlanResponse res;
planner.solve(res);
``` 
There is no current way to set this through the MoveGroupInterface class. 
