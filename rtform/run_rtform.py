#!/usr/bin/env python3

"""
Cartesian Motion Evaluation Script

This script implements a robust Cartesian motion evaluation for robotic tasks using MoveIt.
It uses Cartesian path planning (GetCartesianPath) to execute trajectories in task space,
ensuring straight-line movements in Cartesian space.

Key Features:
1. Uses GetCartesianPath service for Cartesian path planning
2. Plans paths in task space (straight lines in Cartesian space)
3. Robust demo collection with retry logic
4. Comprehensive error handling and success tracking
5. Progress tracking with tqdm
6. Configurable trajectory waypoints
7. Default task: 'open_box'

Main Differences from eval_pose_control.py:
1. Uses Cartesian path planning instead of kinematic path planning
2. More robust error handling and retry logic
3. Better progress tracking and logging
4. Different coordinate transformation for UR5 workspace
5. Success rate tracking for rollouts
6. Maximum execution steps limit
7. More detailed configuration handling

Usage:
    ros2 run ip eval_cartesian_motion --ros-args --task_name <task_name> --num_demos <num> --num_rollouts <num>

Example:
    ros2 run ip eval_cartesian_motion --ros-args --task_name open_box --num_demos 1 --num_rollouts 1
"""

import sys
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from moveit_msgs.srv import GetMotionPlan
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from rclpy.action import ActionClient


class rtformRobot(Node):
    def __init__(self):
        super().__init__('rollout_pose_node')

        self.declare_parameter('json_path', 'assets/cutting_patterns/custom_cutting_pattern_2.json')
        self.json_path = self.get_parameter('json_path').value

        print(f"json for cutting pattern: {self.json_path}")
        self.actutal_corners = [[0,0], [0,100], [100,100], [100,0]]

        # Load points and corners from JSON file
        with open(self.json_path, 'r') as f:
            data = json.load(f)
            self.dxf_points = data['entities']
            self.dxf_corners = data['corners']
        
        print(f"\nLoaded data from {self.json_path}:")
        print(f"Number of entities: {len(self.dxf_points)}")
        for entity_id, points in self.dxf_points.items():
            print(f"Entity {entity_id}: {len(points)} points")
        print(f"Number of corner points: {len(self.dxf_corners)}")
        
        # start the cartesian path planning client
        # Initialize MoveIt services
        self.plan_though_pose_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        self.get_logger().info("Waiting for MoveIt services...")
        self.plan_though_pose_client.wait_for_service()
        self.cartesian_client.wait_for_service()
        self.execute_client.wait_for_server()
        self.get_logger().info("Services ready.")

        self.run_robot()

    def run_robot(self):
        # ask the user to pick an entity

        entity_id = input("Enter the entity ID: ")
        if entity_id not in self.dxf_points:
            raise ValueError(f"Entity ID '{entity_id}' not found in points data")
        entity_points = self.dxf_points[entity_id]
        print(f"total points before filtering: {len(entity_points)}")
        
        # Filter points that are too close to previous point
        filtered_points = []
        min_distance = 0.01  # 1cm threshold
        
        for i, point in enumerate(entity_points):
            if i == 0:
                filtered_points.append(point)
            else:
                prev_point = filtered_points[-1]
                distance = ((point[0] - prev_point[0])**2 + (point[1] - prev_point[1])**2)**0.5
                if distance >= min_distance:
                    filtered_points.append(point)
        entity_points = filtered_points
        print(f"total points after filtering: {len(entity_points)}")

        request = self.createCartesiaRequest()

        print('creating waypoints')
        for j in range(len(entity_points)):

            pose = self.create_pose_from_point(entity_points[j])
            request.waypoints.append(pose)
            # print(f"added position (x, y, z): {pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}")
        


        print('sending the request to planner')
        plan_future = self.cartesian_client.call_async(request)

        print('spinning until future complete')
        rclpy.spin_until_future_complete(self, plan_future)

        if not plan_future.result():
            self.get_logger().warn("Cartesian planning failed.")
            return
        
        # Check if the future is done after timeout or normal completion
        print('exceuting the plan')
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = plan_future.result().solution

        send_goal_future = self.execute_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, send_goal_future)
        print(f'done exceuting the plan')

    def createCartesiaRequest(self):
        request = GetCartesianPath.Request()
        request.group_name = 'ur_manipulator'
        request.link_name = 'tool0'
        request.max_step = 0.01 # 0.01  # 1cm resolution
        request.jump_threshold = 0.0        
        request.avoid_collisions = True
        request.start_state.is_diff = True
        return request
        
    def create_pose_from_point(self, pointxyz):
        'returns the pose'
        pose = Pose()
        pose.position.x = pointxyz[0]/1000  # convert to meters
        pose.position.y = pointxyz[1]/1000  # convert to meters
        pose.position.z = 0.01 # to bring it within the workspace of UR5
        # print(f"transformed pose: {round(pose.position.x,3)} , {round(pose.position.y,3)} , {round(pose.position.z,3)}")
        
        pose.orientation.x = 1.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0
        return pose

     
    

def main():
    rclpy.init()
    node = rtformRobot()
    node.destroy_node()
    rclpy.shutdown()

def printConfig(config):
    for key, value in config.items():
        print(f"{key} : , {value}")
if __name__ == '__main__':

    main()
