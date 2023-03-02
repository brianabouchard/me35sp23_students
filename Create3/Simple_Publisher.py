''' Code from ROS2 Docs
Modifications by Briana Bouchard

This code creates a publisher node and publishes a string to a topic with a counter every 0.5 seconds
'''

import rclpy # imports rclpy client library 
from rclpy.node import Node # imports Node class of rclpy library

from std_msgs.msg import String # imports ROS2 built-in string message type

# Creates SimplePublisher class which is a subclass of Node 
class SimplePublisher(Node):

    # Defines class constructor
    def __init__(self):

        # Initializes and gives Node the name simple_publisher and inherits the Node class's attributes by using 'super()'
        super().__init__('simple_publisher')

        # Creates a publisher based on the message type "String" that has been imported from the std_msgs module above
        self.publisher_ = self.create_publisher(String, 'my_publisher', 10)

        # Set delay in seconds
        timer_period = 0.5  

        # Creates a timer that triggers a callback function after the set timer_period
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Sets initial counter to zero
        self.i = 0

    def timer_callback(self):
        # Assigns message type "String" that has been imported from the std_msgs module above
        msg = String() 

        # Defines string to publish
        msg.data = 'Hello World: %d' % self.i 

        # Publishes `msg` to topic 
        self.publisher_.publish(msg) 

        # Prints `msg.data` to console
        self.get_logger().info('Publishing: "%s"' % msg.data) 

        # Increments counter
        self.i += 1 


def main(args=None):
    # Initializes ROS2 communication and allows Nodes to be created
    rclpy.init(args=args)

    # Creates the SimplePublisher Node
    simple_publisher = SimplePublisher()

    try:
        # Spins the Node to activate the callbacks
        rclpy.spin(simple_publisher)

    # Stops the code if CNTL-C is pressed on the keyboard    
    except KeyboardInterrupt:
        print('\nCaught Keyboard Interrupt')

        # Destroys the node that was created
        simple_publisher.destroy_node()

        # Shuts down rclpy 
        rclpy.shutdown()


if __name__ == '__main__':
    # Runs the main function
    main()