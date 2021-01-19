from std_authorised_srvs.srv._authorised_trigger import AuthorisedTrigger_Request, AuthorisedTrigger_Response
from interceptor.interceptor_node import Filter
from typing import List, Optional
from std_srvs.srv._trigger import Trigger_Request, Trigger_Response
from unique_identifier_msgs.msg import UUID

class AuthorisedTriggerFilter(Filter):

    def __init__(self):
        super().__init__()
        self.__auth_enabled = True
        self.valid_tokens: List[UUID] = [
            UUID(uuid=[0]*16)
        ]

    def enable_authorisation(self, value: bool):
        self.__auth_enabled = value

    #overide
    def request(self, interceptor_request : AuthorisedTrigger_Request) -> Optional[Trigger_Request]:
        """
        Process the request to the Interceptor. If the request should be filtered, return None,
        otherwise return the request to be delegated to the target
        """
        if interceptor_request.auth_token in self.valid_tokens or self.__auth_enabled == False:
            return Trigger_Request()
        else: return None

    #overide
    def response(self, target_response : Optional[Trigger_Response]) -> AuthorisedTrigger_Response:
        """
        Generate a Response from the Interceptor based on the (optional) Response from
        the target.
        """
        interceptor_response = AuthorisedTrigger_Response()
        if target_response != None:
            interceptor_response.success = target_response.success
            interceptor_response.message = target_response.message
        else:
            interceptor_response.success = False
            interceptor_response.message = "user is not authorised"

        return interceptor_response