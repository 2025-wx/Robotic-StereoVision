import rclpy
from rclpy.node import Node
import rclpy.action
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.parameter import Parameter
from lebai import LebaiRobot
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from typing import List
from lebai.type import PVAT, RobotState
from robotic_driver.param_utils import get_joint_names
import time

class TrajectoryActionServer(Node):
    def __init__(self):
        super().__init__('trajectory_action_server')
        self.declare_parameter("controller_joint_names",
                               Parameter.Type.STRING_ARRAY)
        self.declare_parameter("robot_ip_address", "")
        self.joints_name_ = get_joint_names(
            self, 'controller_joint_names', "robot_description")
        # self.loop_ = asyncio.new_event_loop()
        if not self.joints_name_:
            self.get_logger().error('controller_joint_names is not assigned!')
            raise ValueError("No 'controller_joint_names' parameter.")
        if not self.has_parameter('robot_ip_address'):
            self.get_logger().info('"robot_ip_address" is not assigned.')
            raise ValueError("No 'robot_ip_address' parameter.")
        self.robot_ip_ = self.get_parameter(
            'robot_ip_address').get_parameter_value().string_value
        self.robot_ = LebaiRobot(self.robot_ip_, False)

        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'robotic_trajectory_controller',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    # def __del__(self):
    #     self.destroy_node()

    def goal_callback(self, goal_request):
        assert isinstance(goal_request, FollowJointTrajectory.Goal)
        self.get_logger().info('Lebai trajectory received goal request. points size: %d' %
                               (len(goal_request.trajectory.points)))
        if not goal_request.trajectory.points:
            self.get_logger().error('Lebai trajectory failed on empty trajectory')
            return GoalResponse.REJECT
        if goal_request.trajectory.joint_names != self.joints_name_:
            self.get_logger().error('Lebai trajectory failed on invalid joints!')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Lebai trajectory executing trajectory..')
        result = FollowJointTrajectory.Result()
        assert isinstance(goal_handle, rclpy.action.server.ServerGoalHandle)
        assert isinstance(goal_handle.request, FollowJointTrajectory.Goal)

        if not self.is_valid_traj(goal_handle.request.trajectory):
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            goal_handle.abort()
            return result

        try:
            self.loop_send_to_robot(goal_handle)

            self.get_logger().info("Wait for trajectory execution finished...")
            while self.robot_.get_robot_mode() in [RobotState.RUNNING, RobotState.PAUSED]:
                if goal_handle.is_cancel_requested:
                    self.robot_.stop_move()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    goal_handle.canceled()
                    self.get_logger().info('Trajectory execution canceled.')
                    return result
                time.sleep(0.1)

            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            goal_handle.succeed()
            self.get_logger().info('Trajectory execution finished.')

        except Exception as e:
            self.get_logger().error(f"Trajectory execution failed: {str(e)}")
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            goal_handle.abort()

        return result


    def cancel_callback(self, goal_handle):
        # assert isinstance(goal_handle, rclpy.action.server.ServerGoalHandle)
        # goal_handle. = True
        self.get_logger().info('Trajectory received cancel request')
        return CancelResponse.ACCEPT

    def generate_data_list(self, traj_msg: JointTrajectory):
        for i in range(len(traj_msg.points)-1):
            t1 = float(traj_msg.points[i+1].time_from_start.sec) + \
                float(traj_msg.points[i+1].time_from_start.nanosec / 1e9)
            t0 = float(traj_msg.points[i].time_from_start.sec) + \
                float(traj_msg.points[i].time_from_start.nanosec / 1e9)
            data = PVAT(t1 - t0, traj_msg.points[i+1].positions,
                        traj_msg.points[i+1].velocities, traj_msg.points[i+1].accelerations)
            yield data

    def send_to_robot(self, traj_msg: JointTrajectory):
        self.get_logger().info('Trajectory send to robot start.')
        data_gen = self.generate_data_list(traj_msg)
        self.robot_.move_pvats(data_gen)
        self.get_logger().info('Trajectory send to robot end.')

    def loop_send_to_robot(self, goal_handle):
        self.get_logger().info(
            'Sending trajectory start.')
        traj_msg = goal_handle.request.trajectory
        for i in range(len(traj_msg.points)-1):
            if goal_handle.is_cancel_requested:
                self.get_logger().info(
                    'Sending trajectory abort.')
                return
            # rclpy.logging.get_logger('send trajectory').info('i: %d' % i)
            t1 = float(traj_msg.points[i+1].time_from_start.sec) + \
                float(traj_msg.points[i+1].time_from_start.nanosec / 1e9)
            t0 = float(traj_msg.points[i].time_from_start.sec) + \
                float(traj_msg.points[i].time_from_start.nanosec / 1e9)
            # self.get_logger().info('{0} {1} {2} {3} {4} {5} {6} {7}'.format(t0, t1, traj_msg.points[i+1].velocities[0], traj_msg.points[i+1].velocities[1], traj_msg.points[i+1].velocities[2], traj_msg.points[i+1].velocities[3], traj_msg.points[i+1].velocities[4], traj_msg.points[i+1].velocities[5]))
            self.robot_.move_pvat(traj_msg.points[i+1].positions,
                                        traj_msg.points[i+1].velocities,
                                        traj_msg.points[i+1].accelerations,
                                        t1-t0
                                        )
        self.get_logger().info(
            'Sending trajectory finished.')

    def is_valid_traj(self, traj):
        for i in range(len(traj.points)):
            point = traj.points[i]
            if not point.positions:
                return False
            # TODO vel limitation
            # point = JointTrajectoryPoint()
            if i > 0 and point.time_from_start.to_sec() < 1e-6:
                return False
            return True
