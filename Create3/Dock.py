'''
This file was heavily influenced by Sawyer Paccione's action dock file:
Tufts Create® 3 Educational Robot Example
This file is a simple action client file that will dock the robot if it is undocked. 
'''

'''
These statements allow the Node class to be used and actions to be performed. 
'''
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

'''
This statement imports the dock action.
'''
from irobot_create_msgs.action import Dock

'''
Input your namespace here as a global variable. 
'''

class DockActionClient(Node):
    '''
    This is an action client. Action clients send goal requests to action servers.
    We are defining a class "DriveServoActionClient" which is a subclass of Node. 
    '''

    def __init__(self):
        '''
        We initialize the class by calling the Node constructor then
        naming our node 'dock_action_client'
        '''
        super().__init__('dock_action_client')
        
        '''
        Here we initiate a new action server. We include where to add the action client
        (self), the type of action (DockServo), and the action name ('dock').
        '''  
        print('Initiating a new action server...')
        self._action_client = ActionClient(self, Dock,'/dock')

    def send_goal(self):
        '''
        This is the goal message.
        '''
        goal_msg = Dock.Goal()
        print('Goal message: ' + str(goal_msg))
        
        '''
        This method waits for the action server to be available.
        '''

        self._action_client.wait_for_server()
        
        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    dock_client = DockActionClient()
    '''
    Sends a goal to the server.
    '''
    print('Action server available. Sending dock goal to server.')
    future = dock_client.send_goal()
    '''
    When an action server accepts or rejects the goal, future is completed.
    '''
    rclpy.spin_until_future_complete(dock_client, future)
    print('Robot is docking. Shutting down action dock client node.')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
