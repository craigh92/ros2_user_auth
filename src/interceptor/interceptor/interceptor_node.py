from rclpy.node import Node, QoSProfile, CallbackGroup, qos_profile_services_default, Client
from rclpy.service import Service
from rclpy.task import Future
from abc import abstractmethod, ABC
from typing import Generic, Optional, Protocol, TypeVar, Type
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.client import Client
from rclpy.executors import Executor

"""Type Hints"""
InterceptorSrvRequest = TypeVar('InterceptorSrvRequest', )
InterceptorSrvResponse = TypeVar('InterceptorSrvResponse')
class InterceptorSrv(Protocol[InterceptorSrvRequest, InterceptorSrvResponse]):
    Request = InterceptorSrvRequest
    Response = InterceptorSrvResponse
TargetSrvRequest = TypeVar('TargetSrvRequest')
TargetSrvResponse = TypeVar('TargetSrvResponse')
class TargetSrv(Protocol[TargetSrvRequest, TargetSrvResponse]):
    Request = TargetSrvRequest
    Response = TargetSrvResponse

class Filter(ABC, Generic[InterceptorSrvRequest, TargetSrvRequest, InterceptorSrvResponse, TargetSrvResponse]):
    
    @abstractmethod
    def request(self, interceptor_request : InterceptorSrvRequest) -> Optional[TargetSrvRequest]:
        """
        Process the request to the Interceptor. If the request should be filtered, return None,
        otherwise return the request to be delegated to the target
        """
        pass

    @abstractmethod
    def response(self, target_response : Optional[TargetSrvResponse]) -> InterceptorSrvResponse:
        """
        Generate a response from the Interceptor based on the (optional) response from
        the target.
        """
        pass



class InterceptorNode(Node):

    def __init__(self, node_name: str):
        super().__init__(node_name)

    def create_filtered_service(self,
        interceptor_srv_type: Type[InterceptorSrv],
        interceptor_srv_name: str,
        target_srv_type: Type[TargetSrv],
        target_srv_name: str,
        filter: Filter[InterceptorSrv.Request, TargetSrv.Request, InterceptorSrv.Response, TargetSrv.Response],
        *,
        qos_profile: QoSProfile = qos_profile_services_default,
    ) -> Service :

        target_client: Client = self.create_client(target_srv_type, target_srv_name,
            qos_profile=qos_profile,
            callback_group=ReentrantCallbackGroup()
        )
        
        # callback that filters the request and passes it the client
        def intercepted_service_callback(
            interceptor_request: InterceptorSrv.Request,
            interceptor_response: InterceptorSrv.Response, filter=filter
        ) -> InterceptorSrv.Response:

            self.get_logger().info(f'Received request {interceptor_request}')

            target_service_request: Optional[TargetSrv.Request] = filter.request(interceptor_request)
            target_service_response: Optional[TargetSrv.Response] = None
            if target_service_request != None:
                self.get_logger().info(f'Request authorised!')
                self.get_logger().info(f'Passing request to target {target_srv_name} {target_service_request}')
                # (1)
                target_service_response_future: Future = target_client.call_async(target_service_request)
                self.get_logger().info('Waiting for target response')
                self.executor.spin_until_future_complete(target_service_response_future, timeout_sec=1)
                if not target_service_response_future.done():
                    self.get_logger().error('Target did not respond')
                else:
                    target_service_response = target_service_response_future.result()
                self.get_logger().info(f'Received target response {target_service_response}')
            interceptor_response = filter.response(target_service_response)
            return interceptor_response

        # service to run the above callback
        srv = self.create_service(
            srv_type=interceptor_srv_type,
            srv_name=interceptor_srv_name,
            callback=intercepted_service_callback,
            qos_profile=qos_profile,
            callback_group=ReentrantCallbackGroup()
        )

        return srv

#(1)
# we have to call async, in case the target node is running in the same singlethreaded executor as the
# interceptor. If this is the case then the executor is being blocked by this service,
# so cannot process the target callback