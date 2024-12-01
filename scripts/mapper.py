import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
import numpy as np
import concurrent.futures

# -1 is nothing
# 100 is an obstacle
obj_nothing = -1
obj_obstacle = 100

class MapListenerPublisher(Node):
    def __init__(self):
        super().__init__('map_listener_publisher')
        # publisher for the /map topic
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)
        # subscriber for the /map topic
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.listener_callback,
            qos_profile=rclpy.qos.qos_profile_sensor_data
        )
        # initialize map data buffer (empty)
        self.map_data_buffer = None
        self.map_resolution = None
        # set a timer to publish the map periodically
        self.timer = self.create_timer(1, self.publish_map)

    def listener_callback(self, msg):
        self.get_logger().info('Processing sensor data')
        # match incoming map dimensions
        if self.map_data_buffer is None or \
           self.map_data_buffer.shape != (msg.info.height, msg.info.width):
            self.map_data_buffer = np.full((msg.info.height, msg.info.width), obj_nothing)
            self.map_resolution = msg.info.resolution
        incoming_map = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.update_map_with_raycasting(incoming_map)

    def update_map_with_raycasting(self, sensor_map):
        # parallelize raycasting over map chunks
        height, width = sensor_map.shape
        with concurrent.futures.ThreadPoolExecutor() as executor:
            row_interval = 20
            for i in range(0, height, row_interval):
                executor.submit(self.process_chunk, i, min(i + row_interval, height), width, sensor_map)

    def process_chunk(self, i_start, i_end, width, sensor_map):
        # process a chunk of the map
        for i in range(i_start, i_end):
            for j in range(width):
                if sensor_map[i, j] != obj_nothing:
                    self.perform_raycast(i, j, sensor_map[i, j])

    def perform_raycast(self, x0, y0, cell_value):
        # simplified raycasting using Bresenham’s line algorithm
        dx = abs(x0 - y0)
        dy = abs(y0 - x0)
        sx = 1 if x0 < y0 else -1
        sy = 1 if y0 < x0 else -1
        err = dx - dy
        while True:
            if 0 <= x0 < self.map_data_buffer.shape[0] and 0 <= y0 < self.map_data_buffer.shape[1]:
                if cell_value == obj_obstacle:
                    self.map_data_buffer[x0, y0] = obj_obstacle
                    break
                else:
                    self.map_data_buffer[x0, y0] = 0
            if x0 == y0:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

    def apply_low_pass_filter(self, grid):
        # simple moving average filter for faster computation
        kernel = np.ones((3, 3)) / 9.0
        filtered_grid = np.zeros_like(grid)
        # apply kernel to smooth map
        for i in range(1, grid.shape[0] - 1):
            for j in range(1, grid.shape[1] - 1):
                sub_grid = grid[i-1:i+2, j-1:j+2]
                filtered_grid[i, j] = np.sum(sub_grid * kernel)
        return filtered_grid

    def publish_map(self):
        if self.map_data_buffer is None:
            return
        # create a new OccupancyGrid message
        map_msg = OccupancyGrid()
        # set the header information
        map_msg.header = Header()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        # define map metadata
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_data_buffer.shape[1]
        map_msg.info.height = self.map_data_buffer.shape[0]
        map_msg.info.origin = Pose(
            position=Point(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        # apply smoothing before publishing
        smoothed_map = self.apply_low_pass_filter(self.map_data_buffer)
        # flatten buffer to a list for publishing
        map_msg.data = smoothed_map.flatten().tolist()
        self.map_publisher.publish(map_msg)
        self.get_logger().info('Publishing smoothed map data')

def main(args=None):
    print("MAPPER STARTED")
    rclpy.init(args=args)
    map_listener_publisher = MapListenerPublisher()
    rclpy.spin(map_listener_publisher)
    map_listener_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
