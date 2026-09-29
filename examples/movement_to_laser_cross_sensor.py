#!/usr/bin/env python3
import rclpy
import uuid
from queue import Queue
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from movement_controller.action import ExecuteTrajectory
from movement_controller.msg import TrajectoryPath

q = Queue()

def goal_response_callback(future):
    try:
        goal_handle = future.result()
        if not goal_handle.accepted:
            print('Goal rejected')
            return

        print('Goal accepted')

        _get_result_future = goal_handle.get_result_async()
        _get_result_future.add_done_callback(get_result_callback)
        # sleep(1)
        # print('Cancelling goal')
        # goal_handle.cancel_goal_async().add_done_callback(lambda future: print('Goal cancellation approved'))
    except Exception as e:
        print(f'Exception in goal response callback: {e}')

def get_result_callback(future):
    try:
        result = future.result().result
        print('Result received')
        if result is None:
            print('Failed to get result from action server')
            return

        if result.success:
            print('Motion executed successfully')
            print(f'Trajectory paths executed: {result.trajectory_paths_completed}')
        else:
            print(f'Motion execution failed, error message: {result.error_message}')
    except Exception as e:
        print(f'Exception in get result callback: {e}')
        rclpy.shutdown()

def feedback_callback(feedback_msg):
    try:
        feedback = feedback_msg.feedback
        print(f'Feedback with status: {feedback.status} for path IDs: {feedback.trajectory_path_ids}')
    except Exception as e:
        print(f'Exception in feedback callback: {e}')

def main():
    rclpy.init()
    controller = Node('sandbox')
    action_client = ActionClient(controller, ExecuteTrajectory, 'movement_controller/execute_trajectory')
    
    if not action_client.wait_for_server(timeout_sec=5.0):
        controller.get_logger().error('ExecuteTrajectory action server not available')
        return

    paths = []

    # Cross laser center point
    target_laser = PoseStamped()
    target_laser.header.frame_id = "base_link"
    target_laser.header.stamp = controller.get_clock().now().to_msg()
    target_laser.pose.position.x = 0.050
    target_laser.pose.position.y = 0.494
    target_laser.pose.position.z = 0.070
    target_laser.pose.orientation.x = -0.001
    target_laser.pose.orientation.y = 0.706
    target_laser.pose.orientation.z = 0.708
    target_laser.pose.orientation.w = -0.009
    path_laser = TrajectoryPath()
    path_laser.cartesian_speed = 0.1
    path_laser.cartesian_acceleration = 2.2
    path_laser.joint_speed = 1.5
    path_laser.joint_acceleration = 1.0
    path_laser.target_pose = target_laser
    path_laser.motion_type = "PTP"
    path_laser.path_id = str(uuid.uuid4())
    path_laser.tool_frame = "dispensing_endtool_tip_uncalibrated"
    path_laser.blend_radius = 0.0
    paths.append(path_laser)

    # Home position
    # target_home = PoseStamped()
    # target_home.header.frame_id = "base_link"
    # target_home.header.stamp = controller.get_clock().now().to_msg()
    # target_home.pose.position.x = 0.70
    # target_home.pose.position.y = -0.1
    # target_home.pose.position.z = 0.4
    # target_home.pose.orientation.x = 0.50368
    # target_home.pose.orientation.y = 0.49173
    # target_home.pose.orientation.z = 0.50988
    # target_home.pose.orientation.w = 0.49449
    # path_home = TrajectoryPath()
    # path_home.cartesian_speed = 0.1
    # path_home.cartesian_acceleration = 2.2
    # path_home.joint_speed = 2.0
    # path_home.joint_acceleration = 1.0
    # path_home.target_pose = target_home
    # path_home.motion_type = "PTP"
    # path_home.path_id = str(uuid.uuid4())
    # path_home.tool_frame = "dispensing_endtool_tip_uncalibrated"
    # path_home.blend_radius = 0.0
    # paths.append(path_home)
    
    controller.get_logger().info(f'Calling action')

    try:
        request = ExecuteTrajectory.Goal()
        request.paths = paths

        future = action_client.send_goal_async(request, feedback_callback=feedback_callback)
        future.add_done_callback(goal_response_callback)
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Keyboard interrupt received. Shutting down.')
    except Exception as e:
        controller.get_logger().error(f'Exception while executing motion: {e}')
    finally:
        action_client.destroy()
        controller.destroy_node()
        try:
            rclpy.shutdown()
        except Exception as e:
            pass

if __name__ == '__main__':
    main()