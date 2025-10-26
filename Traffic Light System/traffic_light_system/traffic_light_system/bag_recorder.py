import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from std_msgs.msg import String

import rosbag2_py

class TrafficLightBagRecorder(Node):
    def __init__(self):
        super().__init__('traffic_light__bag_recorder')
        self.writer = rosbag2_py.SequentialWriter()

        storage_options = rosbag2_py.StorageOptions(
            uri='traffic_light_state',
            storage_id='mcap'
        )
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        topic_info = rosbag2_py.TopicMetadata(
            id=0,
            name='recorder',
            type='std_msgs/msg/String',
            serialization_format='cdr'
        )
        self.writer.create_topic(topic_info)

        self.subscription = self.create_subscription(
            String,
            'recorder',
            self.topic_callback,
            10
        )
        self.subscription

        self.subscription2 = self.create_subscription(
            String,
            'density_recorder',
            self.second_topic_callback,
            10
        )
        self.subscription2

    def topic_callback(self, msg):
        self.writer.write(
            'recorder',
            serialize_message(msg),
            self.get_clock().now().nanoseconds
        )

    def second_topic_callback(self, msg):
        self.writer.write(
            'density_recorder',
            serialize_message(msg),
            self.get_clock().now().nanoseconds
        )


def main(args=None):
    rclpy.init(args=args)
    sbr = TrafficLightBagRecorder()
    rclpy.spin(sbr)
    rclpy.shutdown()


if __name__ == '__main__':
    main()