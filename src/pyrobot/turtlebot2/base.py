# # Copyright (c) Facebook, Inc. and its affiliates.

# # This source code is licensed under the MIT license found in the
# # LICENSE file in the root directory of this source tree.

# from __future__ import print_function

# import copy
# import os
# from functools import partial
# from os.path import expanduser

# import numpy as np
# import rospy
# import tf
# import tf.transformations
# from ca_msgs.msg import Bumper
# from kobuki_msgs.msg import BumperEvent, CliffEvent, WheelDropEvent
# from nav_msgs.msg import Odometry 

# from pyrobot.core import Base
# from std_msgs.msg import Empty


# class TurtleBot2Base(Base):
#     """
#     This is a common base class for the turtlebot2 base.
#     """

#     def __init__(
#         self,
#         configs,
#         map_img_dir=None,
#         base_controller="ilqr",
#         base_planner=None,
#         base=None,
#     ):
#         """
#         The constructor for Turtlebot2Base class.
#         :param configs: configurations read from config file
#         :param map_img_dir: parent directory of the saved RGB images and depth images
#         :type configs: YACS CfgNode
#         :type map_img_dir: string 
#         """
#         super(TurtleBot2Base, self).__init__(configs=configs)
#         use_base = rospy.get_param("use_base", False) or rospy.get_param("use_sim", False)
#         if not use_base:
#             rospy.logwarn(
#                 "Neither use_base, nor use_sim, is set to True "
#                 "when the Turtlebot2 driver is launched. "
#                 "You may not be able to command the base "
#                 "correctly using PyRobot!"
#             )
#             return
#         if base is None:
#             base = configs.BASE.BASE_TYPE
#         assert base in [
#             "kobuki",
#             "create",
#         ], "BASE should be one of kobuki, create but is {:s}".format(base)
        
#         if map_img_dir is None:
#             map_img_dir = os.path.join(expanduser("~"), ".ros/Imgs")
        
#         self.build_map = rospy.get_param("use_vslam", False)
#         self.base_state = BaseState(base, self.build_map, map_img_dir, configs)

#         #Path planner
#         if base_planner is None:
#             base_planner = configs.BASE.BASE_PLANNER 
#         assert base_planner in [
#             "movebase",
#             "none",
#         ], "BASE. [BASE_PLANNER] should be movebase or none"
#         if base_planner == "movebase":
#             self.planner = MoveBasePlanner(self.configs)
#         elif base_planner == "none":
#             self.planner = None
#         self.base_planner = base_planner

#         #Set up low-level controllers
#         if base_controller is None:
#             base_controller = configs.BASE.BASE_CONTROLLER 
#         assert base_controller in [
#             "proportional",
#             "ilqr",
#             "movebase"
#         ], ("BASE.BASE_CONTROLLER should be one of proportional, ilqr, movebase but is {:s}".format(base_controller))
#         self.base_controller = base_controller
#         if base_controller == "ilqr":
#             self.controller = ILQRControl(self.base_state, self.ctrl_pub, self.configs)
#         elif base_controller == "proportional":
#             self.controller = ProportionalControl(self.base_state, self.ctrl_pub, self.configs)
#         elif base_controller == "movebase":
#             self.controller = MoveBaseControl(self.base_state, self.configs)

#         rospy.on_shutdown(self.clean_shutdown)

#     def clean_shutdown(self):
#         rospy.loginfo("Stop Turtlebot2 Base")
#         if self.base_controller == "movebase":
#             self.controller.cancel_goal()
#         self.stop()

#     def get_state(self, state_type):
#         """
#         Returns the requested base pose in the (x, y, yaw)
#         format as computed either from 
#         Wheel encoder readings or Visual-SLAM

#         :param state_type: Requested state type. Ex: Odom, SLAM, etc
#         :type state_type: string
#         :return: pose of the form [x, y, yaw]
#         :rtype: list
#         """
#         if state_type == "odom":
#             return self.base_state._get_odom_state()
#         elif state_type == "vslam":
#             assert self.build_map, (
#                 "Error: Cannot vslam state without enabling build map feature"
#             )
#             assert self.build_map, "build_map was set to False"
#             return self.base_state.vslam.base_pose

#     def _get_plan(self, xyt_position):
#         """
#         Generates a plan that can take the robot to given goal state.
#         :param xyt_position: The goal state of the form (x, y, t)
#         :type xyt_position: list
#         """
#         plan, states = self.planner.get_plan_absolute(
#             xyt_position[0], xyt_position[1], xyt_position[2]
#         )
#         if not status:
#             raise ValueError("Failed to find a valid plan!")
#         return self.planner.parse_plan(plan)

#     def go_to_relative(
#         self, xyt_position, use_map=False, close_loop=True, smooth=False
#     ):
#         """
#         Moves the robot to the given goal state relative to its initial pose.

#         :param xyt_position: The relative goal state of the form (x, y, t)
#         :param use_map: When set to "True", ensures that controler is using only 
#             free space on the map to mave the robot.
#         :param close_loop: When set to "True", ensures that controler is operating in open loop
#             by taking account of odometry.
#         :param smooth: When set to "True", ensures that the motion leading to the goal
#             is a smooth one.
#         :type xyt_position: list or np.ndarray
#         :type use_map: bool
#         :type close_loop: bool
#         :type smooth: bool
        
#         :return: True if successful; False otherwise (timeout, etc.)
#         :rtype: bool
#         """
#         start_pos = self.base_state.state.state_f.copy()
#         goal_pos = _get_absolute_pose(xyt_position, start_pos.ravel())
#         return self.go_to_absolute(goal_pos, use_map, close_loop, smooth)

#     def go_to_absolute(
#         self, xyt_postion, use_map=False, close_loop=True, smooth=False
#     ):
#         xyt_position = np.asarray(xyt_position)
#         try:
#             if use_map:
#                 if self.base_controller == "ilqr":
#                     goto = partial(self.go_to_relative, close_loop = close_loop, smooth=smooth)
#                     self.planner.move_to_goal(xyt_position, goto)
#                     return
#                 elif self.base_controller == "proportional":
#                     self.planner.move_to_goal(xyt_position, self.controller.goto)
#                     return
#                 elif self.base_controller == "gpmp":
#                     self.controller.go_to_absolute_with_map(
#                         xyt_position, close_loop, smooth, self.planner
#                     )
#                     return

#             self.controller.go_to_absolute(xyt_position, close_loop, smooth)

#         except AssertionError as error:
#             print(error)
#             return False
#         except:
#             print("Unexpected error encountered during position control!")
#             return False
#         return True

#     def track_trajectory(self, states, controls=None, close_loop=True):
#         """
#         State trajectory that the robot should track.
#         :param states: sequence of (x, y, t) states that the robot should track
#         :param controls: optionally specify control sequence as well
#         :param close_loop: whether to close loop on the computed control sequence or not

#         :type states: list
#         :type controls: list
#         :type close_loop: bool

#         :return: True if successful; False otherwise (timeout, etc)
#         :rtype: bool
#         """
#         if len(states) == 0:
#             rospy.loginfo("The given trajectory is empty")
#             return
#         try:
#             if self.base_controller == "ilqr":
#                 self.controller.track_trajectory(states, controls, close_loop)
#             else:
#                 plan_idx = 0
#                 while True:
#                     plan_idx = min(plan_idx, len(states) - 1)
#                     point = states[plan_idx]
#                     self.controller.go_to_absolute(point, close_loop=close_loop)
#                     if plan_idx == len(states) - 1:
#                         break
#                     plan_idx += self.configs.BASE.TRACKED_POINT
#         except AssertionError as error:
#             print(error)
#             return False
#         except:
#             print("Unexpected error encountered during position control!")
#             return False
#         return True
