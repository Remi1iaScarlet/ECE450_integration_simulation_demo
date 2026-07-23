#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


from std_srvs.srv import Trigger


from visual_grasp_msgs.srv import ExecuteTask



class ROS2PiperBridge(Node):

    def __init__(self):

        super().__init__(
            "ros2_piper_bridge"
        )


        # Service exposed to OpenHarmony

        self.server = self.create_service(
            Trigger,
            "/stage2_arm/move",
            self.handle_command
        )


        # Client for Piper

        self.client = self.create_client(
            ExecuteTask,
            "/visual_grasp/execute_task"
        )


        self.get_logger().info(
            "ROS2 Piper Bridge Ready"
        )



    def handle_command(
        self,
        request,
        response
    ):


        self.get_logger().info(
            "Received command from OpenHarmony"
        )


        if not self.client.wait_for_service(
            timeout_sec=5.0
        ):

            response.success = False

            response.message = (
                "Piper service unavailable"
            )

            return response



        piper_request = (
            ExecuteTask.Request()
        )


        #
        # Temporary command
        # Later replaced by LLM/object parser
        #

        piper_request.source_object = (
            "object"
        )

        piper_request.target_object = (
            "target"
        )


        future = self.client.call_async(
            piper_request
        )


        rclpy.spin_until_future_complete(
            self,
            future
        )


        result = future.result()


        response.success = (
            result.success
        )

        response.message = (
            result.message
        )


        return response




def main():

    rclpy.init()

    node = ROS2PiperBridge()

    rclpy.spin(node)


    node.destroy_node()

    rclpy.shutdown()



if __name__ == "__main__":
    main()
