from rclpy.node import Node
from std_srvs.srv import Trigger
from rclpy.callback_groups import ReentrantCallbackGroup

class ExampleTargetNode(Node):

    def __init__(self, node_name: str):

        super().__init__(node_name)

        self.create_service(Trigger, '_say_hello', lambda _, res : (
            self.get_logger().info("hello!"),
            setattr(res, 'success', True),
            res)[-1],
            callback_group=ReentrantCallbackGroup()
        )

        self.create_service(Trigger, '_say_goodbye', lambda _, res : (
            self.get_logger().info("goodbye!"),
            setattr(res, 'success', True),
            res)[-1],
            callback_group=ReentrantCallbackGroup()
        )

        self.get_logger().info(f'Started {node_name}')