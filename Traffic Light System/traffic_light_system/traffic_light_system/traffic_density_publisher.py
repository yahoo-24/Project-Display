import rclpy
from rclpy.node import Node
from random import randint
from std_msgs.msg import Int64MultiArray
from std_msgs.msg import String, Int64
from traffic_messages.msg import TrafficLightState
import time

class TrafficDensityPublisher(Node):

    def __init__(self):
        super().__init__('traffic_density_publisher')
        self.current_time = 0
        self.density = [5, 5, 5, 5]
        self.publisher_ = self.create_publisher(Int64MultiArray, 'density', 10)
        time_period = 20
        self.timer = self.create_timer(time_period, self.timer_callback)
        self.recorder_publisher_ = self.create_publisher(String, 'density_recorder', 10)
        self.subscriber = self.create_subscription(TrafficLightState, 'topic', self.sub_callback, 10)
        self.states = [True, False, False, False]

        self.density_feedback = self.create_subscription(Int64MultiArray, 'density_updater', self.feedback, 10)

        self.subscription = self.create_subscription(
            Int64,
            'universal_time',
            self.listener_callback,
            10
        )
        self.subscription

    def feedback(self, msg):
        self.density = msg.data

    def timer_callback(self):
        # Generate 4 random numbers representing the traffic density at each light
        for i in range(4):
            self.density[i] += randint(0, 12)
        
        # Publish the numbers
        msg = Int64MultiArray()
        msg.data = self.density
        self.publisher_.publish(msg)
        self.get_logger().info(f"Traffic density [{int(time.time())}]: 1. {self.density[0]}   2. {self.density[1]}   3. {self.density[2]}   4. {self.density[3]}\n")

        recorder_msg = String()
        recorder_msg.data = f"Traffic density [{self.current_time}]: 1. {self.density[0]}   2. {self.density[1]}   3. {self.density[2]}   4. {self.density[3]}"
        self.recorder_publisher_.publish(recorder_msg)

    def listener_callback(self, msg):
        self.current_time = msg.data

    def sub_callback(self, msg):
        self.states = msg.states

def main(args=None):
    rclpy.init(args=args)

    publisher = TrafficDensityPublisher()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()