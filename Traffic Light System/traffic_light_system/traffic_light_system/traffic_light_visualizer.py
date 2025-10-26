import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from traffic_messages.msg import TrafficLightState
from geometry_msgs.msg import Point
from std_msgs.msg import Int64MultiArray
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Duration
import random
import math
import numpy as np
import subprocess


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class TrafficLightVisualizer(Node):
    def __init__(self):
        super().__init__('traffic_light_visualizer')
        self.sub = self.create_subscription(TrafficLightState, 'topic', self.callback, 10)
        self.density_sub = self.create_subscription(
            Int64MultiArray,
            'density',
            self.listener_callback,
            10
        )
        path = "/home/yahia/ros2_ws/src/traffic_light_system/description/sdf/car.sdf"
        self.car_sdf = open(path, 'r').read()
        self.unique_id = 0

        self.cars_point_tracker = [[], [], [], []]
        self.car_colour_tracker = [[], [], [], []]
        self.is_left_lane = [[], [], [], []]
        self.states = [True, False, False, False]
        self.remaining_cars = [(0, False), (0, False), (0, False), (0, False)]
        self.density = [5, 5, 5, 5]
        self.turning_cars = [0, 0, 0, 0]
        self.time = [[], [], [], []]
        self.pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)

        self.density_updater = self.create_publisher(Int64MultiArray, 'density_updater', 10)
        time_period = 5.0
        self.density_timer = self.create_timer(time_period, self.update_density)

        self.move_direction = {
            0: (0.0, -0.15),
            1: (-0.15, 0.0),
            2: (0.0, 0.15),
            3: (0.15, 0.0)
        }

        self.crossing_threshold = {
            0: (10000.0, 14.9),
            1: (14.9, 10000.0),
            2: (-10000.0, -14.9),
            3: (-14.9, -10000.0)
        }

        self.positions = {
            0: (0.0, 15.0, 5.0, 0.0),
            1: (15.0, 0.0, 5.0, math.radians(90)),
            2: (0.0, -15.0, 5.0, math.radians(180)),
            3: (-15.0, 0.0, 5.0, math.radians(270)),
            4: (0.0, 15.0, 3.8, 0.0),
            5: (15.0, 0.0, 3.8, math.radians(90)),
            6: (0.0, -15.0, 3.8, math.radians(180)),
            7: (-15.0, 0.0, 3.8, math.radians(270))
        }

        self.support_positions = {
            0: (0.0, 14.6, 4.0, 0.0),
            1: (14.6, 0.0, 4.0, math.radians(90)),
            2: (0.0, -14.6, 4.0, math.radians(180)),
            3: (-14.6, 0.0, 4.0, math.radians(270)),
            4: (0.0, 14.6, 1.25, 0.0),
            5: (14.6, 0.0, 1.25, math.radians(90)),
            6: (0.0, -14.6, 1.25, math.radians(180)),
            7: (-14.6, 0.0, 1.25, math.radians(270)),
        }

        self.side_offset = {
            0: (0.0, 1.0),
            1: (1.0, 0.0),
            2: (0.0, -1.0),
            3: (-1.0, 0.0)
        }

        marker_array = MarkerArray()

        # This is the road
        marker_road = Marker()
        marker_road.header.frame_id = "map"
        marker_road.header.stamp = self.get_clock().now().to_msg()
        marker_road.ns = "traffic_lights"
        marker_road.id = 999
        marker_road.type = Marker.CUBE
        marker_road.action = Marker.ADD
        marker_road.pose.position.x = 0.0
        marker_road.pose.position.y = 0.0
        marker_road.pose.position.z = 0.05
        marker_road.scale.x = 210.0
        marker_road.scale.y = 210.0
        marker_road.scale.z = 0.1
        marker_road.color.a = 1.0
        marker_road.color.r, marker_road.color.g, marker_road.color.b = 0.4, 0.4, 0.4
        marker_array.markers.append(marker_road)

        self.side_size = 90.0
        for i in range(4):
            # This is the side walk under the traffic light (left lane)
            marker_side = Marker()
            marker_side.header.frame_id = "map"
            marker_side.header.stamp = self.get_clock().now().to_msg()
            marker_side.ns = "traffic_lights"
            marker_side.id = 998 - i
            marker_side.type = Marker.CUBE
            marker_side.action = Marker.ADD

            x, y, _, yaw = self.support_positions[i]
            offset_x, offset_y = self.side_offset[i]
            marker_side.scale.x = 4.0
            marker_side.scale.y = self.side_size
            marker_side.scale.z = 0.5
            marker_side.pose.position.x = x + offset_x * (marker_side.scale.y / 2 - 1)
            marker_side.pose.position.y = y + offset_y * (marker_side.scale.y / 2 - 1)
            marker_side.pose.position.z = 0.25
            marker_side.color.a = 1.0
            marker_side.color.r, marker_side.color.g, marker_side.color.b = 0.793, 0.254, 0.328

            qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)
            marker_side.pose.orientation.x = qx
            marker_side.pose.orientation.y = qy
            marker_side.pose.orientation.z = qz
            marker_side.pose.orientation.w = qw

            marker_array.markers.append(marker_side)

            # This is the side walk on the right lane
            marker_end = Marker()
            marker_end.header.frame_id = "map"
            marker_end.header.stamp = self.get_clock().now().to_msg()
            marker_end.ns = "traffic_lights"
            marker_end.id = 998 - i - 4
            marker_end.type = Marker.CUBE
            marker_end.action = Marker.ADD

            marker_end.scale.x = 90.0
            marker_end.scale.y = 90.0
            marker_end.scale.z = 0.5
            marker_end.pose.position.x = x + offset_x * (marker_side.scale.y / 2 - 1) + y + offset_y * (marker_side.scale.y / 2 - 1)
            marker_end.pose.position.y = y + offset_y * (marker_side.scale.y / 2 - 1) - x - offset_x * (marker_side.scale.y / 2 - 1)
            marker_end.pose.position.z = 0.25
            marker_end.color.a = 1.0
            marker_end.color.r, marker_end.color.g, marker_end.color.b = 0.793, 0.254, 0.328

            marker_end.pose.orientation.x = qx
            marker_end.pose.orientation.y = qy
            marker_end.pose.orientation.z = qz
            marker_end.pose.orientation.w = qw

            marker_array.markers.append(marker_end)

            # These are the stop lines at the traffic light
            stop_line = Marker()
            stop_line.header.frame_id = "map"
            stop_line.header.stamp = self.get_clock().now().to_msg()
            stop_line.ns = "traffic_lights"
            stop_line.id = 998 - i - 8
            stop_line.type = Marker.LINE_STRIP
            stop_line.action = Marker.ADD

            stop_line.scale.x = 0.4
            stop_line.pose.position.x = x
            stop_line.pose.position.y = y
            if i == 0 or i == 2:
                p1 = Point(x=x-15, y=0.0, z=0.1)
                p2 = Point(x=x+15, y=0.0, z=0.1)
            else:
                p1 = Point(x=0.0, y=y+15, z=0.1)
                p2 = Point(x=0.0, y=y-15, z=0.1)
            stop_line.points = [p1, p2]
            stop_line.color.a = 1.0
            stop_line.color.r, stop_line.color.g, stop_line.color.b = 1.0, 1.0, 1.0

            marker_array.markers.append(stop_line)

            # These are the lane changing not continuous lines
            lane_changing_line = Marker()
            lane_changing_line.header.frame_id = "map"
            lane_changing_line.header.stamp = self.get_clock().now().to_msg()
            lane_changing_line.ns = "traffic_lights"
            lane_changing_line.id = 998 - i - 12
            lane_changing_line.type = Marker.LINE_LIST
            lane_changing_line.action = Marker.ADD

            lane_changing_line.scale.x = 0.4
            lane_changing_line.pose.position.x = x
            lane_changing_line.pose.position.y = y
            if i == 2 or i == 3:
                multiplier = -1.0
            else:
                multiplier = 1.0
            if i == 0 or i == 2:
                for i in range(86):
                    p = Point(x=7.7, y=i*multiplier, z=0.1)
                    lane_changing_line.points.append(p)
                for i in range(86):
                    p = Point(x=-7.7, y=i*multiplier, z=0.1)
                    lane_changing_line.points.append(p)
            else:
                for i in range(86):
                    p = Point(x=i*multiplier, y=7.7, z=0.1)
                    lane_changing_line.points.append(p)
                for i in range(86):
                    p = Point(x=i*multiplier, y=-7.7, z=0.1)
                    lane_changing_line.points.append(p)
            lane_changing_line.color.a = 1.0
            lane_changing_line.color.r, lane_changing_line.color.g, lane_changing_line.color.b = 1.0, 1.0, 1.0

            marker_array.markers.append(lane_changing_line)

        for i in range(4):
            # These are the poles for the traffic light made up of a cylinder and cube
            marker_cube = Marker()
            marker_cube.header.frame_id = "map"
            marker_cube.header.stamp = self.get_clock().now().to_msg()
            marker_cube.ns = "traffic_lights"
            marker_cube.id = 1000 + i
            marker_cube.type = Marker.CUBE
            marker_cube.action = Marker.ADD

            x, y, z, yaw = self.support_positions[i]
            marker_cube.pose.position.x = x
            marker_cube.pose.position.y = y
            marker_cube.pose.position.z = z
            marker_cube.scale.x = 1.3
            marker_cube.scale.y = 3.0
            marker_cube.scale.z = 0.8
            marker_cube.color.a = 1.0

            qx, qy, qz, qw = quaternion_from_euler(math.radians(90), 0, yaw)
            marker_cube.pose.orientation.x = qx
            marker_cube.pose.orientation.y = qy
            marker_cube.pose.orientation.z = qz
            marker_cube.pose.orientation.w = qw

            marker_cylinder = Marker()
            marker_cylinder.header.frame_id = "map"
            marker_cylinder.header.stamp = self.get_clock().now().to_msg()
            marker_cylinder.ns = "traffic_lights"
            marker_cylinder.id = 1000 + i + 4
            marker_cylinder.type = Marker.CYLINDER
            marker_cylinder.action = Marker.ADD

            x, y, z, yaw = self.support_positions[i + 4]
            marker_cylinder.pose.position.x = x
            marker_cylinder.pose.position.y = y
            marker_cylinder.pose.position.z = z
            marker_cylinder.scale.x = 0.6
            marker_cylinder.scale.y = 0.6
            marker_cylinder.scale.z = 2.5
            marker_cylinder.color.a = 1.0

            marker_cube.color.r, marker_cube.color.g, marker_cube.color.b = 0.5, 0.5, 0.5
            marker_cylinder.color.r, marker_cylinder.color.g, marker_cylinder.color.b = 0.5, 0.5, 0.5

            marker_array.markers.append(marker_cube)
            marker_array.markers.append(marker_cylinder)

        self.pub.publish(marker_array)
        self.update_cars()
        self.update_period = 0.02
        self.car_moving_timer = self.create_timer(self.update_period, self.move_cars)
    
    def callback(self, msg):
        self.states = msg.states
        marker_array = MarkerArray()

        for index in list(self.positions.keys()):
            # These are the cylinders representing the traffic light. There are 8 (4 green and 4 red).
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "traffic_lights"
            marker.id = index
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            x, y, z, yaw = self.positions[index]
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            marker.scale.x = 0.8
            marker.scale.y = 0.8
            marker.scale.z = 0.3
            marker.color.a = 1.0

            if index < 4:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
                if self.states[index]:
                    marker.color.a = 0.1
                else:
                    marker.color.a = 1.0
            else:
                marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0
                if self.states[index-4]:
                    marker.color.a = 1.0
                else:
                    marker.color.a = 0.1

            qx, qy, qz, qw = quaternion_from_euler(math.radians(90), 0, yaw)
            marker.pose.orientation.x = qx
            marker.pose.orientation.y = qy
            marker.pose.orientation.z = qz
            marker.pose.orientation.w = qw

            marker_array.markers.append(marker)

        self.pub.publish(marker_array)

    def listener_callback(self, msg):
        self.density = msg.data
        self.update_cars()

    def update_cars(self):
        marker_array = MarkerArray()
        density = [d if d < 20 else 20 for d in self.density]

        for i in range(4):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "cars"
            marker.id = (i + 1) * 2000
            marker.type = Marker.CUBE_LIST
            marker.action = Marker.ADD

            if i == 0 or i == 2:
                marker.scale.x = 3.0
                marker.scale.y = 4.0
            else:
                marker.scale.x = 4.0
                marker.scale.y = 3.0
            marker.scale.z = 2.5

            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0

            if i == 1 or i == 2:
                multiplier = 1.0
            else:
                multiplier = -1.0
            
            start_left = True
            d = density[i]
            present = self.remaining_cars[i][0]
            if self.remaining_cars[i][1]:
                start_left = False
            for index in range(present, d):
                even = index // 2
                if (index % 2 == 1 and start_left) or (index % 2 == 0 and not start_left):
                    second_lane = 5.0
                    self.is_left_lane[i].append(False)
                else:
                    second_lane = 0.0
                    self.is_left_lane[i].append(True)

                if i == 0 or i == 2:
                    y_p = (-50 - 4.5 * even - 2.0) * multiplier
                    x_p = 0.0 + (5.0 + second_lane) * multiplier
                else:
                    x_p = (50.0 + 4.5 * even + 2.0) * multiplier
                    y_p = 0.0 + (5.0 + second_lane) * multiplier
                p = Point(x=x_p, y=y_p, z=1.15)
                self.cars_point_tracker[i].append(p)

                r, g, b = random.random(), random.random(), random.random()
                self.car_colour_tracker[i].append(ColorRGBA(r=r, g=g, b=b, a=1.0))
                self.time[i].append(0.0)

            marker.points = self.cars_point_tracker[i]
            marker.colors = self.car_colour_tracker[i]
            marker_array.markers.append(marker)
       
        self.pub.publish(marker_array)

    def move_cars(self):
        marker_array = MarkerArray()
        for i in range(4):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "cars"
            marker.id = (i + 1) * 2000
            marker.type = Marker.CUBE_LIST
            marker.action = Marker.ADD

            if i == 0 or i == 2:
                marker.scale.x = 3.0
                marker.scale.y = 4.0
            else:
                marker.scale.x = 4.0
                marker.scale.y = 3.0
            marker.scale.z = 2.5

            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0

            threshold_x, threshold_y = self.crossing_threshold[i]
            x_move, y_move = self.move_direction[i]
            if i == 0 or i == 1:
                multiply = 1.0
            else:
                multiply = -1.0
            
            crossed = 0
            delete_index = 0
            # If it is red and has not crossed the intersection then all the next cars cannot move, hence break the loop
            # Keep track of how many cars are left behind at the light
            # Check that the car has not crossed a point where it will be deleted
            for index, point in enumerate(self.cars_point_tracker[i]):
                if self.check_out_of_bounds(i, point):
                    delete_index = index + 1
                # Check that the light is green or if the car has crossed into the intersection
                if (point.x * multiply < threshold_x * multiply and point.y * multiply < threshold_y * multiply):
                    self.time[i][index] += self.update_period
                    if not self.is_left_lane[i][index]:
                        self.move(point, self.time[i][index], x_move, y_move, index + 1)
                    crossed += 1
                elif self.states[i]:
                    self.time[i][index] += self.update_period
                    self.move(point, self.time[i][index], x_move, y_move, index + 1)
                else:
                    if self.expected_position(index - crossed, i, point):
                        point.x += x_move / 2
                        point.y += y_move / 2
                    self.time[i][index] = 0.0
            
            if crossed != 0 and len(self.is_left_lane) != 0:
                self.remaining_cars[i] = (len(self.cars_point_tracker[i]) - crossed, self.is_left_lane[i][crossed - 1])
            else:
                self.remaining_cars[i] = (len(self.cars_point_tracker[i]), self.remaining_cars[i][1])

            temp_col = []
            left_lane_col = []
            temp_point = []
            left_lane_point = []
            left_lane_time = []
            for j in range(crossed):
                if not self.is_left_lane[i][j]:
                    temp_col.append(self.car_colour_tracker[i][j])
                    temp_point.append(self.cars_point_tracker[i][j])
                else:
                    left_lane_point.append(self.cars_point_tracker[i][j])
                    left_lane_col.append(self.car_colour_tracker[i][j])
                    left_lane_time.append(self.time[i][j])
            temp_col.extend(self.car_colour_tracker[i][crossed:])
            temp_point.extend(self.cars_point_tracker[i][crossed:])
            #marker.colors = self.car_colour_tracker[i]
            #marker.points = self.cars_point_tracker[i]
            marker.colors = temp_col
            marker.points = temp_point
            marker_array.markers.append(marker)

            for index, point in enumerate(left_lane_point):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "cars"
                marker.id = 10000 * (i + 1) + index
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.lifetime = Duration(sec=0, nanosec=20_000_000)

                yaw = self.circular_move(point, left_lane_time[index], x_move, y_move, index + 1)
                x, y, z = point.x, point.y, point.z
                if i == 0 or i == 2:
                    marker.scale.x = 3.0
                    marker.scale.y = 4.0
                else:
                    marker.scale.x = 4.0
                    marker.scale.y = 3.0
                marker.scale.z = 2.5
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = z
                marker.color.a = 1.0
                colour = left_lane_col[index]
                marker.color.r, marker.color.g, marker.color.b = colour.r, colour.g, colour.b

                qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)
                marker.pose.orientation.x = qx
                marker.pose.orientation.y = qy
                marker.pose.orientation.z = qz
                marker.pose.orientation.w = qw

                marker_array.markers.append(marker)

            if delete_index != 0:
                self.car_colour_tracker[i] = self.car_colour_tracker[i][delete_index:]
                self.cars_point_tracker[i] = self.cars_point_tracker[i][delete_index:]
                self.time[i] = self.time[i][delete_index:]
                self.is_left_lane[i] = self.is_left_lane[i][delete_index:]

        self.pub.publish(marker_array)

    def check_out_of_bounds(self, i, point):
        if abs(point.y) > self.side_size + 10 or abs(point.x) > self.side_size + 10:
            return True
        return False
    
    def expected_position(self, index, i, point):
        if i == 1 or i == 2:
            multiplier = 1.0
        else:
            multiplier = -1.0
        even = index // 2
        if i == 0 or i == 2:
            y_p = (-15.0 - 4.5 * even - 2.0) * multiplier
            if abs(point.y) > abs(y_p):
                return True
        else:
            x_p = (15.0 + 4.5 * even + 2.0) * multiplier
            if abs(point.x) > abs(x_p):
                return True
        return False
            
    def move(self, point, time, x_move, y_move, index):
        factor = 0.15 * random.random() + 0.85
        x_move *= factor
        y_move *= factor
        if time / math.sqrt(index) > 1:
            point.x += x_move
            point.y += y_move
        else:
            point.x += x_move * time / math.sqrt(index)
            point.y += y_move * time / math.sqrt(index)

    def circular_move(self, point, time, x_move, y_move, index):
        yaw = 0
        x_move, y_move, yaw = self.adjust_circular_move_direction(point, x_move, y_move)

        if time / math.sqrt(index) > 1:
            point.x += x_move
            point.y += y_move
        else:
            point.x += x_move * time / math.sqrt(index)
            point.y += y_move * time / math.sqrt(index)
        return yaw

    def adjust_circular_move_direction(self, point, x_move, y_move):
        magnitude = x_move + y_move
        if x_move < 0.0:
            yaw = (10 - (point.x)) / 15 * math.pi / 2
            yaw = self.check_yaw(yaw)
            y_move = math.sin(yaw) * magnitude
            x_move *= math.cos(yaw)
        elif x_move > 0.0:
            yaw = (-10 - (point.x)) / -15 * math.pi / 2
            yaw = self.check_yaw(yaw)
            y_move = math.sin(yaw) * magnitude
            x_move *= math.cos(yaw)
        elif y_move < 0.0:
            yaw = (10 - (point.y)) / 15 * math.pi / 2
            yaw = self.check_yaw(yaw)
            x_move = -math.sin(yaw) * magnitude
            y_move *= math.cos(yaw)
        elif y_move > 0.0:
            yaw = (-10 - (point.y)) / -15 * math.pi / 2
            yaw = self.check_yaw(yaw)
            x_move = -math.sin(yaw) * magnitude
            y_move *= math.cos(yaw)
        return x_move, y_move, yaw
    
    def check_yaw(self, yaw):
        if yaw > math.pi / 2:
            yaw = math.pi / 2
        elif yaw < 0:
            yaw = 0
        return yaw
    
    def update_density(self):
        msg = Int64MultiArray()
        msg.data = [i[0] for i in self.remaining_cars]
        self.density_updater.publish(msg)

def main():
    rclpy.init()
    node = TrafficLightVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()