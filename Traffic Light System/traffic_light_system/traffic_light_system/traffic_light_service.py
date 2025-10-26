import rclpy
from rclpy.node import Node
from traffic_messages.msg import TrafficLightState
from traffic_messages.srv import LightChange
from std_msgs.msg import Int64MultiArray
from std_msgs.msg import String, Int64
import time
import threading

class TrafficLightService(Node):

    def __init__(self):
        super().__init__('traffic_server')
        self.start_time = int(time.time())
        self.LightStates = [True, False, False, False]
        self.traffic_light_timers = [7, 7, 7, 7] # How long each light will be open for.
        self.current_light = 0 # Index for self.current_light
        self.before = time.time() # The initial time used to calculate if the traffic light must change
        self.prev_pedestrian_stop = time.time()
        self.scheduled_pedestrian = False
        self.declare_parameter('pedestrian_wait_time', 15) # How long it has to wait before the pedestrian lights can be called again
        self.declare_parameter('total_time_split', 15) # Splits 15 seconds among the traffic lights depending on density
        self.declare_parameter('pedestrian_duration', 7.0) # How long the pedestrian lights last
        self.declare_parameter('emergency_duration', 10.0) # How long the emergency vehicle lights last

        self.srv = self.create_service(LightChange, 'light_change', self.change_lights)

        self.publisher_ = self.create_publisher(TrafficLightState, 'topic', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.universal_time_publisher_ = self.create_publisher(Int64, 'universal_time', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.universal_timer_callback)

        self.publisher2_ = self.create_publisher(String, 'recorder', 10)
        
        self.subscription = self.create_subscription(
            Int64MultiArray,
            'density',
            self.listener_callback,
            10
        )
        self.subscription

        self.traffic_density = [5, 5, 5, 5]
        self.emergency_vehicle = False
        self.pedestrian_crossing = False

    def change_lights(self, request, response):
        if request.direction >= 0 and request.direction <= 3 and not(self.emergency_vehicle):
            go = True
            duration = 0.0
            reason = 'manual override'
            type = int(request.type)
            if type == 0: # Emergency Vehicle
                duration = self.get_parameter('emergency_duration').get_parameter_value().double_value
                self.emergency_vehicle = True
                reason = 'emergency vehicle'
            elif type == 1: # Pedestrian Crossings
                duration = self.get_parameter('pedestrian_duration').get_parameter_value().double_value
                reason = 'pedestrian crossing'
                # There is a delay before the next time a pedestrian crossing is allowed
                now = time.time()
                time_elapsed = now - self.prev_pedestrian_stop
                pedestrian_wait_time = self.get_parameter('pedestrian_wait_time').get_parameter_value().integer_value
                if time_elapsed < pedestrian_wait_time:
                    go = False # Lights are not ready to be change so it is scheduled
                else:
                    self.prev_pedestrian_stop = time.time() + duration # Accounts for the pedestrian crossing duration
                    self.pedestrian_crossing = True
            elif type <= 10: # Manual Override
                duration = float(request.type)

            if go:
                self.before += duration # Instead of pausing, self.before is increased to account for the time of the manual override
                self.LightStates = [False, False, False, False]
                if type != 1:
                    self.LightStates[request.direction] = True
                try:
                    self.play_timer.cancel()
                except AttributeError:
                    pass
                self.play_timer = threading.Timer(duration, self.manual_timer_callback)
                self.play_timer.start()

                # Update the recorder
                msg = String()
                output_time = int(time.time()) - self.start_time
                if reason == 'pedestrian crossing':
                    msg.data = f"Switching lights off due to {reason} at {output_time}"
                    self.get_logger().info('Incoming request to turn pedestrian crossing green')
                else:
                    msg.data = f"Switching light to {request.direction} due to {reason} at {output_time}"
                    self.get_logger().info('Incoming request to turn green: %d' % (request.direction))
                self.publisher2_.publish(msg)

                response.states = self.LightStates

                return response
            else:
                if self.pedestrian_crossing:
                    # Client side will understand that this is a denied request due to existing pedestrian crossing
                    response.states = [True, True, True, False]
                    return response
                else:
                    # Let the server know that a crossing is scheduled so it rejects other crossing requests
                    self.pedestrian_crossing = True
                
                def send_response():
                    self.delay_request.cancel()

                    if self.emergency_vehicle:
                        # Delay the request even further and check every second
                        self.scheduled_pedestrian = True
                        self.delay_request = self.create_timer(0.9, send_response)
                        self.delay_request.cancel()
                        self.delay_request.reset()       
                        return
                    
                    self.before += duration
                    self.LightStates = [False, False, False, False]
                    self.pedestrian_crossing = True
                    self.scheduled_pedestrian = False

                    try:
                        self.play_timer.cancel()
                    except AttributeError:
                        pass
                    self.play_timer = threading.Timer(duration, self.manual_timer_callback)
                    self.play_timer.start()

                    msg = String()
                    output_time = int(time.time()) - self.start_time
                    msg.data = f"Switching lights off due to {reason} at {output_time}"
                    self.publisher2_.publish(msg)
                    response.states = self.LightStates
                    self.prev_pedestrian_stop = time.time() + duration # Accounts for the pedestrian crossing duration
                    self.get_logger().info('Incoming request to turn pedestrian crossing green')
                
                self.delay_request = self.create_timer(pedestrian_wait_time - time_elapsed, send_response)
                self.delay_request.cancel()
                self.delay_request.reset()

                response.states = [False, False, False, False]
                return response
        else:
            if self.emergency_vehicle:
                response.states = [True, True, True, True] # Client side will understand that this is a denied request due to emergency vehicle
            else:
                response.states = [True, True, False, True] # Client side will understand that this is a denied request due to incorrect input
            return response
    
    def manual_timer_callback(self):
        self.LightStates = [False, False, False, False]
        self.emergency_vehicle = False
        self.pedestrian_crossing = False
        if not(self.scheduled_pedestrian):
            self.get_logger().info('Returning to original state')
            self.LightStates[self.current_light] = True

            # Update the recorder
            msg = String()
            output_time = int(time.time()) - self.start_time
            msg.data = f"Returning to original state -> {self.current_light} is on at {output_time}"
            self.publisher2_.publish(msg)
    
    def next_light(self):
        # Increment the index
        if self.current_light == 3:
            self.current_light = 0
        else:
            self.current_light += 1

    def timer_callback(self):
        # Change the traffic light time based on the traffic density
        total_traffic = sum(self.traffic_density)
        time_split = self.get_parameter('total_time_split').get_parameter_value().integer_value
        if total_traffic != 0:
            for i in range(4):
                if self.traffic_density[i] != 0:
                    self.traffic_light_timers[i] = 3 + (self.traffic_density[i] / total_traffic) * time_split
                else:
                    self.traffic_light_timers[i] = 0
        
        # Calculate the time elapsed and change the traffic state
        now = int(time.time())
        time_elapsed = now - self.before
        if time_elapsed >= self.traffic_light_timers[self.current_light]:
            self.LightStates[self.current_light] = False
            self.next_light()
            while self.traffic_light_timers[self.current_light] == 0:
                self.next_light()
            self.LightStates[self.current_light] = True
            self.before = now
            # Update the recorder
            msg = String()
            output_time = int(time.time()) - self.start_time
            msg.data = f"Switching light to {self.current_light} at {output_time}"
            self.publisher2_.publish(msg)

        # Publish the Traffic Light State
        msg = TrafficLightState()
        msg.states = self.LightStates
        self.publisher_.publish(msg)
        output_time = int(time.time()) - self.start_time
        self.get_logger().info(f"Publishing Traffic Light States [{output_time}]: \n1 {msg.states[0]} \n2 {msg.states[1]} \n3 {msg.states[2]} \n4 {msg.states[3]}\n")

    def listener_callback(self, msg):
        self.traffic_density = msg.data

    def universal_timer_callback(self):
        msg = Int64()
        output_time = int(time.time()) - self.start_time
        msg.data = output_time
        self.universal_time_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    traffic_service = TrafficLightService()

    rclpy.spin(traffic_service)

    traffic_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()