#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import numpy as np
import tf2_ros
import concurrent.futures
from aStar import AStar

# -1 is nothing
# 100 is an obstacle
obj_nothing = -1
obj_obstacle = 100
obj_threshold = 50
limo_radius = 10

class Planner(Node):
    def __init__(self):
        super().__init__('planning_node')
        self.subscription_map = self.create_subscription(OccupancyGrid, '/map', self.callback_map, 10)
        self.subscription_odom = self.create_subscription(Odometry, '/odometry', self.callback_odom, 10)
        self.target_publisher = self.create_publisher(Point, '/cur_target', 10)
        self.timer = self.create_timer(.5, self.timer_callback)
        self.cur_map = None
        self.cur_odom = None
        self.i = 0
        self.cur_target = None
        self.goal_pos = (1000, 1000)
        global limo_radius
        self.path_finder = AStar(agent_radius=limo_radius)
        self.path = None
        self.transform = tf2_ros.

    def timer_callback(self):
        # print(f"{self.i}")
        self.i += 1
        self.publish_target()

    def publish_target(self):
        print(self.path)
        if self.cur_map == None or self.cur_odom == None:
            return
        #get data from previous map and odom data
        origin = self.cur_map.info.origin.position
        resolution = self.cur_map.info.resolution
        width = self.cur_map.info.width
        height = self.cur_map.info.height
        cur_pos = (self.cur_odom.pose.pose.position.x, self.cur_odom.pose.pose.position.y)
        cur_pos_on_map = 
        data = self.cur_map.data
        #get objects in env from map
        objs = []
        global obj_threshold
        for i in range(height):
            for j in range(width):
                if data[(i*width)+j] > obj_threshold :
                    objs.append((origin.x+(j*resolution), origin.y+(i*resolution), resolution, resolution))
        #check if any new objects in prev path
        out = Point()
        #TODO implement cur path check but not priority will still work with gen new path every .5s
        #self.update_cur_target_idx()
        #collision = True
        #if self.path != None:
        #    collision = self.check_collisions()
        #if not collision:
        #    out.x = self.path[self.cur_target_idx][0]
        #    out.y = self.path[self.cur_target_idx][1]
        #    self.target_publisher.publish(out)
        #    return
        #if new objects in prev path gen new path
        self.path_finder.place_objects(objs)
        print(width, height, resolution)
        self.path_finder.generate_digital_graph(width, height, int(resolution*100))
        path = self.path_finder.generate_path(cur_pos, self.goal_pos)
        self.cut_target_idx = 0
        out.x = self.path[self.cur_target_idx][0]
        out.y = self.path[self.cur_target_idx][1]
        self.target_publisher.publish(out)

        

    def callback_odom(self, msg):
        self.cur_odom = msg

    def callback_map(self, msg):
        self.cur_map = msg

def main(args=None):
    print("PLANNER STARTED")
    rclpy.init(args=args)
    planner = Planner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
