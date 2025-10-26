import rclpy
import sys
from rclpy.node import Node
from traffic_messages.srv import LightChange

class TrafficLightClient(Node):

    def __init__(self):
        super().__init__('traffic_client')
        self.cli = self.create_client(LightChange, 'light_change')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = LightChange.Request()

    def send_request(self):
        self.req.direction = int(sys.argv[1])
        self.req.type = int(sys.argv[2])
        self.future = self.cli.call_async(self.req)

    
def main(args=None):
    rclpy.init(args=args)

    traffic_client = TrafficLightClient()
    traffic_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(traffic_client)
        if traffic_client.future.done():
            try:
                response = traffic_client.future.result()
            except Exception as e:
                traffic_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                if (response.states[0] or response.states[1]) and (response.states[2] or response.states[3]):
                    if not response.states[3]:
                        traffic_client.get_logger().info("Request denied due to existing pedestrian crossing")
                    elif not response.states[2]:
                        traffic_client.get_logger().info("Request denied due to incorrect input")
                    else:
                        traffic_client.get_logger().info("Request denied due to emergency vehicle")
                else:
                    traffic_client.get_logger().info(
                        f"Result of light change for {traffic_client.req.direction}:\n1 {response.states[0]}\n2 {response.states[1]} \n3 {response.states[2]} \n4 {response.states[3]}"
                    )
            break

    traffic_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
