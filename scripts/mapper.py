import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import numpy as np
import concurrent.futures

# -1 is nothing
# 100 is an obstacle
obj_nothing = -1
obj_obstacle = 100

class Mapper(Node):
    def __init__(self):
        super().__init__('map_listener_publisher')
        # subscriber for the /map topic
        self.subscription_map = self.create_subscription(OccupancyGrid, '/map', self.callback_map, 10)
        # subscriber for the /odom topic
        self.subscription_odom = self.create_subscription(Odometry, '/odometry', self.callback_odom, 10)
        # publisher for the /map_processed topic
        self.map_processed_publisher = self.create_publisher(OccupancyGrid, '/map_processed', 10)
        # initialize map data buffer (empty)
        self.map_data_buffer = None
        self.map_resolution = 0.05
        self.robot_position = None
        self.robot_orientation = None
        # set a timer to publish the map periodically
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # print(f"{self.i}")
        self.i += 1
        self.publish_map()

    def callback_odom(self, msg):
        self.get_logger().info('callback_odom')
        self.robot_position = msg.pose.pose.position
        self.robot_orientation = msg.pose.pose.orientation

    def callback_map(self, msg):
        self.get_logger().info('callback_map')
        # match incoming map dimensions
        if self.map_data_buffer is None or self.map_data_buffer.shape != (msg.info.height, msg.info.width):
           self.map_data_buffer = np.full((msg.info.height, msg.info.width), obj_nothing)
           self.map_resolution = msg.info.resolution
        incoming_map = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.update_map_normal(incoming_map, msg)
        # self.update_map_with_raycasting(incoming_map)

    def publish_map(self):
        self.get_logger().info('publish_map')
        if self.map_data_buffer is None:
            return
        map_msg = OccupancyGrid()
        map_msg.header = Header()
        # map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_data_buffer.shape[1]
        map_msg.info.height = self.map_data_buffer.shape[0]
        map_msg.info.origin = Pose(
            position=Point(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        # if self.robot_position != None and self.robot_orientation != None:
        #     map_msg.info.origin = Pose(
        #         position=Point(x=self.robot_position.x, y=self.robot_position.y, z=0.0),
        #         orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        #     )
        # smoothed_map = self.apply_low_pass_filter(self.map_data_buffer)
        normal_map = self.map_data_buffer
        # flatten buffer to a list for publishing
        map_msg.data = normal_map.flatten().tolist()
        self.map_processed_publisher.publish(map_msg)
        self.get_logger().info('publish_map (finished)')
    
    def update_map_normal(self, sensor_map, msg):
        self.get_logger().info('update_map_normal')
        for i in range(msg.info.height):
            for j in range(msg.info.width):
                if sensor_map[i, j] != -1:  # Only update known cells
                    distance = np.sqrt((i - 50)**2 + (j - 50)**2) * msg.info.resolution  # Example robot at center
                    if distance <= 5.0:
                        self.map_data_buffer[i, j] = sensor_map[i, j]

    def update_map_with_raycasting(self, sensor_map):
        self.get_logger().info('update_map_with_raycasting')
        height, width = sensor_map.shape
        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
            row_interval = 10
            futures = []
            for i in range(0, height, row_interval):
                futures.append(executor.submit(self.process_chunk, i, min(i + row_interval, height), width, sensor_map))
            concurrent.futures.wait(futures)

    def process_chunk(self, i_start, i_end, width, sensor_map):
        self.get_logger().info('process_chunk')
        # process a chunk of the map
        for i in range(i_start, i_end):
            for j in range(width):
                if sensor_map[i, j] != obj_nothing:
                    self.perform_raycast(i, j, sensor_map[i, j])

    def perform_raycast(self, x0, y0, cell_value):
        self.get_logger().info('perform_raycast')
        if self.map_data_buffer[x0, y0] == obj_obstacle:
            return
        # simplified raycasting using Bresenham’s line algorithm
        dx = abs(x0 - y0)
        dy = abs(y0 - x0)
        sx = 1 if x0 < y0 else -1
        sy = 1 if y0 < x0 else -1
        err = dx - dy
        while 0 <= x0 < self.map_data_buffer.shape[0] and 0 <= y0 < self.map_data_buffer.shape[1]:
            if cell_value == obj_obstacle:
                self.map_data_buffer[x0, y0] = obj_obstacle
                break
            else:
                self.map_data_buffer[x0, y0] = 0
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

    def apply_low_pass_filter(self, grid):
        self.get_logger().info('apply_low_pass_filter')
        # simple moving average filter for faster computation
        kernel = np.ones((3, 3)) / 9.0
        filtered_grid = np.zeros_like(grid)
        # apply kernel to smooth map
        for i in range(1, grid.shape[0] - 1):
            for j in range(1, grid.shape[1] - 1):
                sub_grid = grid[i-1:i+2, j-1:j+2]
                filtered_grid[i, j] = np.sum(sub_grid * kernel)
        return filtered_grid

def main(args=None):
    print("MAPPER STARTED")
    rclpy.init(args=args)
    mapper = Mapper()
    rclpy.spin(mapper)
    mapper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
