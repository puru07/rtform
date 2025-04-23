import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.action import ExecuteTrajectory
from rclpy.action import ActionClient


class MoveToHomeNode(Node):
    def __init__(self):
        super().__init__('move_to_home_node')

        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        self.get_logger().info("⏳ Waiting for MoveIt services...")
        self.plan_client.wait_for_service()
        self.execute_client.wait_for_server()
        self.get_logger().info("✅ MoveIt services are ready.")

        self.move_to_home()

    def move_to_home(self):
        # Define joint goal for UR5e — use correct joint names
        home_joint_positions = {
            "shoulder_pan_joint": 0.0,
            "shoulder_lift_joint": -1.57,
            "elbow_joint": 1.57,
            "wrist_1_joint": 0.0,
            "wrist_2_joint": 1.57,
            "wrist_3_joint": 0.0,
        }

        request = GetMotionPlan.Request()
        request.motion_plan_request.group_name = 'ur_manipulator'
        request.motion_plan_request.num_planning_attempts = 5
        request.motion_plan_request.allowed_planning_time = 5.0
        request.motion_plan_request.start_state.is_diff = True

        # Create goal constraints using the actual joint names
        goal = Constraints()
        for joint_name, position in home_joint_positions.items():
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = position
            joint_constraint.tolerance_above = 0.001
            joint_constraint.tolerance_below = 0.001
            joint_constraint.weight = 1.0
            goal.joint_constraints.append(joint_constraint)

        request.motion_plan_request.goal_constraints.append(goal)

        # Send motion planning request
        future = self.plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if not future.result() or not future.result().motion_plan_response.trajectory.joint_trajectory.points:
            self.get_logger().error("❌ Planning failed or empty trajectory.")
            return

        self.get_logger().info("✅ Planning succeeded. Executing trajectory...")

        # Execute the trajectory
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = future.result().motion_plan_response.trajectory

        send_goal_future = self.execute_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        result_future = send_goal_future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info("🏁 Trajectory execution complete!")

def main():
    rclpy.init()
    node = MoveToHomeNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
