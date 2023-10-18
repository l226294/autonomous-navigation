#!/usr/bin/env python
#
# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
This module contains a local planner to perform
low-level waypoint following based on PID controllers.
"""

import collections
import math
import threading

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile, DurabilityPolicy

from carla_ad_agent.vehicle_pid_controller import VehiclePIDController
from carla_ad_agent.misc import distance_vehicle

from carla_msgs.msg import CarlaEgoVehicleControl  # pylint: disable=import-error
from nav_msgs.msg import Odometry, Path
from autoware_auto_planning_msgs.msg import Trajectory
from autoware_auto_planning_msgs.msg import TrajectoryPoint
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker


class LocalPlanner(CompatibleNode):
    """
    LocalPlanner implements the basic behavior of following a trajectory of waypoints that is
    generated on-the-fly. The low-level motion of the vehicle is computed by using two PID
    controllers, one is used for the lateral control and the other for the longitudinal
    control (cruise speed).

    When multiple paths are available (intersections) this local planner makes a random choice.
    """

    # minimum distance to target waypoint as a percentage (e.g. within 90% of
    # total distance)
    MIN_DISTANCE_PERCENTAGE = 0.9

    def __init__(self):
        super(LocalPlanner, self).__init__("local_planner")

        role_name = self.get_param("role_name", "ego_vehicle")
        self.control_time_step = self.get_param("control_time_step", 0.05)

        args_lateral_dict = {}
        args_lateral_dict['K_P'] = self.get_param("Kp_lateral", 0.9)
        args_lateral_dict['K_I'] = self.get_param("Ki_lateral", 0.0)
        args_lateral_dict['K_D'] = self.get_param("Kd_lateral", 0.0)

        args_longitudinal_dict = {}
        args_longitudinal_dict['K_P'] = self.get_param("Kp_longitudinal", 0.206)
        args_longitudinal_dict['K_I'] = self.get_param("Ki_longitudinal", 0.0206)
        args_longitudinal_dict['K_D'] = self.get_param("Kd_longitudinal", 0.515)

        self.data_lock = threading.Lock()

        self._current_pose = None
        self._current_speed = None
        self._target_speed = 0.0

        self._waypoints_queue = collections.deque(maxlen=2000)

        # subscribers
        self._odometry_subscriber = self.new_subscription(
            Odometry,
            "/carla/{}/odometry".format(role_name),
            self.odometry_cb,
            qos_profile=10)
        self._path_subscriber = self.new_subscription(
            Trajectory,
            "/planning/scenario_planning/lane_driving/trajectory",
            self.path_cb,
            qos_profile=1)
        # "/planning/scenario_planning/lane_driving/trajectory"
        # publishers
        self._control_cmd_publisher = self.new_publisher(
            CarlaEgoVehicleControl,
            "/carla/{}/vehicle_control_cmd".format(role_name),
            qos_profile=10)

        # initializing controller
        self._vehicle_controller = VehiclePIDController(
            self, args_lateral=args_lateral_dict, args_longitudinal=args_longitudinal_dict)

    def odometry_cb(self, odometry_msg):
        with self.data_lock:
            self._current_pose = odometry_msg.pose.pose
            self._current_speed = math.sqrt(odometry_msg.twist.twist.linear.x ** 2 +
                                            odometry_msg.twist.twist.linear.y ** 2 +
                                            odometry_msg.twist.twist.linear.z ** 2) * 3.6


    def path_cb(self, path_msg):
        with self.data_lock:
            self._waypoints_queue.clear()
            self._waypoints_queue.extend([point for point in path_msg.points])


    def run_step(self):
        """
        Executes one step of local planning which involves running the longitudinal
        and lateral PID controllers to follow the waypoints trajectory.
        """
        if self._current_pose is None:
            return
        with self.data_lock:
            if not self._waypoints_queue:
                self.loginfo("Waiting for a route...")
                self.emergency_stop()
                return

            min_dist = 1000
            dist_list = []
            nearest_index = 0
            for i, route_point in enumerate(self._waypoints_queue):
                dist = distance_vehicle(route_point.pose, self._current_pose.position)
                dist_list.append(dist)
                if dist < min_dist:
                    min_dist = dist
                    nearest_index = i


            self._target_speed = self._waypoints_queue[nearest_index+1].longitudinal_velocity_mps
            print("target speed: ", self._target_speed)
            if self._target_speed < 1 :
                self._target_speed = 1 
            sampling_radius = self._target_speed * 1  # search radius for next waypoints in seconds
            min_distance = max(sampling_radius * self.MIN_DISTANCE_PERCENTAGE, 8)
            for i in range(nearest_index, len(dist_list)):
                if dist_list[i] < min(min_distance, 10):
                    max_index = i

            # move using PID controllers
            control_msg = self._vehicle_controller.run_step(
                self._target_speed * 3.6, self._current_speed, self._current_pose, self._waypoints_queue[min(max_index+1, len(dist_list))].pose)

            self._control_cmd_publisher.publish(control_msg)

    def emergency_stop(self):
        control_msg = CarlaEgoVehicleControl()
        control_msg.steer = 0.0
        control_msg.throttle = 0.0
        control_msg.brake = 1.0
        control_msg.hand_brake = False
        control_msg.manual_gear_shift = False
        self._control_cmd_publisher.publish(control_msg)


def main(args=None):
    """

    main function

    :return:
    """
    roscomp.init("local_planner", args=args)

    local_planner = None
    update_timer = None
    try:
        local_planner = LocalPlanner()
        roscomp.on_shutdown(local_planner.emergency_stop)

        update_timer = local_planner.new_timer(
            local_planner.control_time_step, lambda timer_event=None: local_planner.run_step())

        local_planner.spin()

    except KeyboardInterrupt:
        pass

    finally:
        roscomp.loginfo('Local planner shutting down.')
        roscomp.shutdown()

if __name__ == "__main__":
    main()
