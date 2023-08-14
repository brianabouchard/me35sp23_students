import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# Import the dock action
from irobot_create_msgs.action import Dock

# Define the class DockActionClient as a subclass of Node
class DockActionClient(Node):

    # Define a function to initalize the node
    def __init__(self):

        # Initialize a node the name dock_action_client
        super().__init__('dock_action_client')

        # Create an action client using the action type 'Dock' that we imported above 
        # with the action name 'dock' which can be found by running ros2 action list -t
        self._action_client = ActionClient(self, Dock, 'dock')

    # Define a function to send the goal to the action server which is already
    # running on the Create 3 robot. Since this action does not require any request value
    # as part of the goal message, the only argument of this function is self. 
    # For more details on this action, review  
    # https://github.com/iRobotEducation/irobot_create_msgs/blob/humble/action/Dock.action
    def send_goal(self):

        # Create a variable for the goal request message to be sent to the action server
        goal_msg = Dock.Goal()

        # Instruct the action client to wait for the action server to become available
        self._action_client.wait_for_server()

        # Sends goal request to the server, returns a future object to the _send_goal_future
        # attribute of the class, and creates a feedback callback as a new function which
        # we will define below as 'feedback_callback' 
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        # Creates a callback that executes a new function 'goal_response_callback'
        # when the future is complete. This function is defined below.
        # This happens when the action server accepts or rejects the request
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    # Define a response callback for when the future is complete. This will 
    # tell us if the goal request has been accepted or rejected by the server.
    # Note that because we have a future object we need to pass that in as 
    # an argument of this function.
    def goal_response_callback(self, future):

        # Store the result of the future as a new variable named 'goal_handle'
        goal_handle = future.result()

        # Perform an initial check to simply see if the goal was accepted or rejected 
        # and print this to the logger.
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # Now that we know the goal was accepted and we should expect a result,
        # ask for that result and return a future when received. 
        self._get_result_future = goal_handle.get_result_async()

        # Creates a callback that executes a new function 'get_result_callback'
        # when the future is complete. This function is defined below.
        # This happens when the action server accepts or rejects the request
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    # Define a result callback for when the future is complete. This will 
    # tell us the result sent to us from the server.
    # Note that because we have a future object we need to pass that in as 
    # an argument of this function.
    def get_result_callback(self, future):

        # Store the result from the server in a 'result' variable
        result = future.result().result

        # Print the result to the logger. We know what to ask for 'result.is_docked'
        # based on the action documentation for the dock action
        self.get_logger().info('Result: {0}'.format(result.is_docked))

        # Shut down rclpy
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.sees_dock))

def main(args=None):
    rclpy.init(args=args)

    action_client = DockActionClient()

    action_client.send_goal()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
