from std_srvs.srv import Trigger
from std_authorised_srvs.srv import AuthorisedTrigger
from interceptor.interceptor_node import InterceptorNode
from interceptor.example.authorised_trigger_filter import AuthorisedTriggerFilter
from interceptor.example.example_target_node import ExampleTargetNode
from std_srvs.srv import SetBool
import rclpy
from rclpy.executors import SingleThreadedExecutor

class ExampleInterceptorNode(InterceptorNode):
    def __init__(self, node_name: str):
        
        super().__init__(node_name)
        
        self.say_hello_filter = AuthorisedTriggerFilter()

        self.create_filtered_service(
            interceptor_srv_type=AuthorisedTrigger,
            interceptor_srv_name='say_hello',
            target_srv_type=Trigger,
            target_srv_name='_say_hello',
            filter=self.say_hello_filter
        )

        self.create_service(
            srv_type=SetBool,
            srv_name='enable_say_hello_authoristion',
            callback= lambda req, res : (
                self.get_logger().info(f"Authorisation on : {req.data}"),
                self.say_hello_filter.enable_authorisation(req.data),
                setattr(res, 'success', True),
                setattr(res, 'message', 'Success'),
                res
            )[-1]
        )

        self.get_logger().info(f'Started {node_name}')

def main(args=None):
    rclpy.init()
    exe=SingleThreadedExecutor()
    try:
        exe.add_node(ExampleTargetNode('example_target_node'))
        exe.add_node(ExampleInterceptorNode('example_interceptor_node'))
        exe.spin()
    except KeyboardInterrupt as e:
        print('Goodbye!')

if __name__ == '__main__':
    main()