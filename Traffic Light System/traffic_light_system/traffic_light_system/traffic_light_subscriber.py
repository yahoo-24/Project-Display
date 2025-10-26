import rclpy
from rclpy.node import Node

from traffic_messages.msg import TrafficLightState


class TrafficLightSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            TrafficLightState,                                               # CHANGE
            'topic',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f"I heard:\n1 {msg.states[0]}\n2 {msg.states[1]} \n3 {msg.states[2]} \n4 {msg.states[3]}")  # CHANGE


def main(args=None):
    rclpy.init(args=args)

    traffic_subscriber = TrafficLightSubscriber()

    rclpy.spin(traffic_subscriber)

    traffic_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()