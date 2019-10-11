
# coding: utf-8

# In[ ]:

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")

rospy.sleep(2)
group.set_planning_time(25)
p = PoseStamped()
rospy.loginfo('p defined as PoseStamped')

p.header.frame_id = robot.get_planning_frame()
p.pose.position.x = 0
p.pose.position.y = -0.6
p.pose.position.z = 0.01
scene.attach_box("base_link", "table_base", p, (1,0.5,0.02))
p.pose.position.x = 0.3
p.pose.position.y = -0.6
p.pose.position.z = 0.25
scene.attach_box("base_link", "left_wall", p, (0.02,0.5,0.5))
p.pose.position.x = -0.3
p.pose.position.y = -0.6
p.pose.position.z = 0.25
scene.attach_box("base_link", "right_wall", p, (0.02,0.5,0.5))
rospy.loginfo('objects attached to base_link')

group_variable_values = group.get_current_joint_values()
# Workaround for setting current state as start state as set_start_state is not working as intended
current_state = robot.get_current_state()
joint_name = current_state.joint_state.name
joint_values = current_state.joint_state.position
joint_command = {name:value for name, value in zip(joint_name, joint_values) if name.startswith('manipulator')}
joint_state = JointState()
joint_state.header = Header()
# joint_state.header.stamp = rospy.Time.now()
joint_state.name = joint_command.keys()
joint_state.position = joint_command.values()
moveit_robot_state = RobotState()
moveit_robot_state.joint_state = joint_state
group.set_start_state(moveit_robot_state)

target_joint_values = target
group.set_joint_value_target(target_joint_values)
rospy.loginfo( 'About to plan')

plan = group.plan()
rospy.loginfo('Planning done, now sleeping for 3 secs')

rospy.sleep(3)
rospy.loginfo('Plan should execute now')
group.execute(plan)
rospy.loginfo('Plan executed')

