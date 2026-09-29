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
    
    # Example usage
    poses = []
    for xy in [
        (0.70, -0.1),
        (0.8, -0.1),
        (0.80, -0.2),
        (0.7, -0.2),
        (0.70, -0.1),
    ]:
        target = PoseStamped()
        target.header.frame_id = "base_link"
        target.header.stamp = controller.get_clock().now().to_msg()
        target.pose.position.x = xy[0]
        target.pose.position.y = xy[1]
        target.pose.position.z = 0.4
        target.pose.orientation.x = 0.50368
        target.pose.orientation.y = 0.49173
        target.pose.orientation.z = 0.50988
        target.pose.orientation.w = 0.49449
        poses.append(target)
    
    controller.get_logger().info(f'Calling action')

    try:
        request = ExecuteTrajectory.Goal()
        paths = []
        for target in poses:
            path = TrajectoryPath()
            path.cartesian_speed = 0.1
            path.cartesian_acceleration = 2.2
            path.joint_speed = 0.157
            path.joint_acceleration = 2.2
            path.target_pose = target
            path.motion_type = "PTP"
            path.path_id = str(uuid.uuid4())
            path.tool_frame = "dispensing_endtool_tip_uncalibrated"
            path.blend_radius = 0.0
            paths.append(path)

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