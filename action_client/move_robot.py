from typing import List
import json

import rclpy
import rclpy.callback_groups
import rclpy.executors
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

from custom_interfaces.srv import BimanualJson


class MoveGroupActionClientNode(Node):
    def __init__(self, node_name: str, choose_robot: str) -> None:
        super().__init__(node_name)

        self.action_server = f"/{choose_robot}/move_action"
        self.move_group_name = f"{choose_robot}_arm"
        self.base = "world"
        self.end_effector = "link_ee"

        self.move_group_action_client = ActionClient(
            self, MoveGroup, self.action_server
        )

        self.get_logger().info(f"Waiting for action server {self.action_server}...")
        if not self.move_group_action_client.wait_for_server(timeout_sec=1):
            raise RuntimeError(
                f"Couldn't connect to action server {self.action_server}."
            )
        self.get_logger().info(f"Done.")

    def send_goal_async(self, target: Pose):
        goal = MoveGroup.Goal()
        goal.request.allowed_planning_time = 1.0
        goal.request.goal_constraints.append(
            Constraints(
                position_constraints=[
                    PositionConstraint(
                        header=Header(frame_id=self.base),
                        link_name=self.end_effector,
                        constraint_region=BoundingVolume(
                            primitives=[SolidPrimitive(type=2, dimensions=[0.0001])],
                            primitive_poses=[Pose(position=target.position)],
                        ),
                        weight=1.0,
                    )
                ],
                orientation_constraints=[
                    OrientationConstraint(
                        header=Header(frame_id=self.base),
                        link_name=self.end_effector,
                        orientation=target.orientation,
                        absolute_x_axis_tolerance=0.001,
                        absolute_y_axis_tolerance=0.001,
                        absolute_z_axis_tolerance=0.001,
                        weight=1.0,
                    )
                ],
            )
        )
        goal.request.group_name = self.move_group_name
        goal.request.max_acceleration_scaling_factor = 0.1
        goal.request.max_velocity_scaling_factor = 0.1 #0.01
        goal.request.num_planning_attempts = 5 #1

        return self.move_group_action_client.send_goal_async(goal)






class LLM_Executor_Server(Node):
    def __init__(self):
        super().__init__("LLM_Executor_Server")

        self.llm_executor_service_ = self.create_service(BimanualJson, "llm_executor", self.llm_service_callback)


        # Add an instance of Move Group Action Client Node for each robot.
        self.move_group_left_action_client_node = MoveGroupActionClientNode(
            "move_group_left_action_client_node", "left"
        )

        self.move_group_right_action_client_node = MoveGroupActionClientNode(
            "move_group_right_action_client_node", "right"
        )

        # Make callback groups
        self.callback_group_left_ = rclpy.callback_groups.MutuallyExclusiveCallbackGroup
        self.callback_group_left_.add_entity(self.move_group_left_action_client_node)
        self.callback_group_right_ = rclpy.callback_groups.MutuallyExclusiveCallbackGroup
        self.callback_group_right_.add_entity(self.move_group_right_action_client_node)
        self.callback_group_llm_ = rclpy.callback_groups.MutuallyExclusiveCallbackGroup
        self.callback_group_llm_.add_entity(self.llm_executor_service_)


    def llm_service_callback(self, request, response):
        
        try:
            self.steps = json.loads(request)

            for step in self.steps:
                left_coordinates = self.steps[step]["left_ee_coor"]
                right_coordinates = self.steps[step]["right_ee_coor"]
                left_orientation = self.steps[step]["left_ee_orientation"]
                right_orientation = self.steps[step]["right_ee_orientation"]
                left_gripper = self.steps[step]["left_gripper_state"]
                right_gripper = self.steps[step]["right_gripper_state"]

                # Make plan for left arm and execute
                left_pose = Pose(
                        position=Point(x=left_coordinates[0], y=left_coordinates[1], z=left_coordinates[2]),
                        orientation=Quaternion(x=left_orientation[0], y=left_orientation[1], z=left_coordinates[2], w=left_orientation[3]),
                    )
                left_future = self.move_group_left_action_client_node.send_goal_async(left_pose)
                rclpy.spin_until_future_complete(
                        self.move_group_left_action_client_node, left_future
                    )  # gets stuck for invalid goals
                #
                #
                #
                # Make plan for right arm and execute
                right_pose = Pose(
                        position=Point(x=right_coordinates[0], y=right_coordinates[1], z=right_coordinates[2]),
                        orientation=Quaternion(x=right_orientation[0], y=right_orientation[1], z=right_coordinates[2], w=right_orientation[3]),
                    )
                right_future = self.move_group_right_action_client_node.send_goal_async(right_pose)
                rclpy.spin_until_future_complete(
                        self.move_group_right_action_client_node, right_future
                    )  # gets stuck for invalid goals




            response.success = True
            response.message = "Executed without error."

        except Exception as e:
            self.get_logger().error(f"Converting string to json faild with error: {e}")
            response.success = False
            response.message = str(e)

        

        return response





def main(args: List = None) -> None:
    rclpy.init(args=args)

    llm_node = LLM_Executor_Server()
    executer = rclpy.executors.MultiThreadedExecutor(5)

    executer.add_node(llm_node)

    executer.spin()


    rclpy.shutdown()


if __name__ == "__main__":
    main()
