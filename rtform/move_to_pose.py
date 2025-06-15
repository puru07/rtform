import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.action import ExecuteTrajectory

from rclpy.action import ActionClient

class MoveToPoseNode(Node):
    def __init__(self):
        super().__init__('move_to_pose_node')

        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        self.get_logger().info("⏳ Waiting for services...")
        self.plan_client.wait_for_service()
        self.execute_client.wait_for_server()
        self.get_logger().info("✅ Services ready.")

        self.move_to_pose()

    def move_to_pose(self):
        # ✨ Define the goal pose for the end-effector
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "base_link"
        goal_pose.pose.position.x = 0.275 # 0.4
        goal_pose.pose.position.y = 1.462 - 1 # 0.0
        goal_pose.pose.position.z = -0.009 + 0.2 # 0.3

        # Orientation (as quaternion)
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 1.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 0.0
        print(goal_pose.pose.position)

        request = GetMotionPlan.Request()
        request.motion_plan_request.group_name = 'ur_manipulator'
        request.motion_plan_request.allowed_planning_time = 5.0
        request.motion_plan_request.start_state.is_diff = True

        # ✨ Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header = goal_pose.header
        pos_constraint.link_name = "tool0"  # End-effector link
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0

        # Bounding region: a small box around the target point
        region = SolidPrimitive()
        region.type = SolidPrimitive.BOX
        region.dimensions = [0.001, 0.001, 0.001]  # very tight box
        bounding_volume = BoundingVolume()
        bounding_volume.primitives.append(region)
        bounding_volume.primitive_poses.append(goal_pose.pose)
        pos_constraint.constraint_region = bounding_volume
        pos_constraint.weight = 1.0

        # ✨ Orientation constraint
        ori_constraint = OrientationConstraint()
        ori_constraint.header = goal_pose.header
        ori_constraint.link_name = "tool0"
        ori_constraint.orientation = goal_pose.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.01
        ori_constraint.absolute_y_axis_tolerance = 0.01
        ori_constraint.absolute_z_axis_tolerance = 0.01
        ori_constraint.weight = 1.0

        # ✨ Add constraints to request
        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)
        request.motion_plan_request.goal_constraints.append(constraints)

        # 🔄 Call the planner
        future = self.plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if not future.result() or not future.result().motion_plan_response.trajectory.joint_trajectory.points:
            self.get_logger().error("❌ Planning failed or trajectory is empty.")
            return

        self.get_logger().info("✅ Plan received. Executing...")

        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = future.result().motion_plan_response.trajectory

        send_goal_future = self.execute_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        result_future = send_goal_future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info("🏁 Trajectory execution complete!")

def main():
    rclpy.init()
    node = MoveToPoseNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
