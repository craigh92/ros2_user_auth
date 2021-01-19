ROS2 User Auth is a collection of ROS2 packages for adding user permissions to allready existing Actions and Services. This functionality is acheived using the [Interceptor Design Pattern](https://www.tutorialspoint.com/design_pattern/intercepting_filter_pattern.htm).

### List of packages

| Name | Description |
| ---- | ----------- |
| interceptor |  A base class Interceptor Node for creating intercepted Actions and Services. The API is similar to the ROS2 base Node class. Also includes the abstract filter class used by the Interceptor Node. | 
| std_authorised_srvs | An authorised equivelent for all the `std_srvs`. The authorised versions have two extra request fields, a UUID representing the user that called the service, and the name of the service this message is being used for |
| user_authentication (ToDo) | Package for validating that a user is who they say they are using LDAP |
| user_authorisation (ToDo) | Implements the abstract filter from the interceptor package as an AuthorisingFilter. This checks that the user is permitted to call the Action or Service. The list of permitted Actions or Services for a user is looked up using LDAP |

### Motivations

The Interceptor Node is not coupled to the authorsation or authentication method, so a developer can switch these out if they want. The user_authentication and user_authorisation packages (using LDAP) are just one example of an implementation of the Interceptor Node.

### How it works

#### Interceptor

Instead of the client directly calling the action or service of the target Node, they instead call an intercepted version of this action or service. The intercepted service type does not have to be the same message type as the target service type as a custom conversion is performed. When the interceptor Node recieves a request, it uses a Filter to check if this request passes some condition and disards it if it does not. If it does pass the filter, it is converted to the target service type and delegated to the target Node. The optional response from the target (optional because discarded requests will not get a response) is then converted to an interceptor response and returned to the client.

Creating an intercepted service is similar to creating a regular service:

```python
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
```

The filter parameter is any object that implemements the abstract base class `Filter` from the `interceptor` package

```python
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
```

For example:

```python
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
```

The sequence of function calls and service calls is generalised in the below UML sequence diagram:

![](https://github.com/craigh92/ros2_user_auth/blob/main/src/interceptor/docs/interceptor.svg)

An example of how this can be used to implement authentication and authorisation is shown in the below UML sequence diagram:

![](https://github.com/craigh92/ros2_user_auth/blob/main/src/interceptor/docs/interceptor_crane.svg)
