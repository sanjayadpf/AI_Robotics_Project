#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose
import numpy as np
import matplotlib.pyplot as plt

class SimpleSLAM:
    def __init__(self):
        rospy.init_node('simple_slam')

        # Setup publisher and subscribers
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # SLAM components
        self.num_particles = 100
        self.particles = np.random.rand(self.num_particles, 3)  # [x, y, theta]
        self.weights = np.ones(self.num_particles) / self.num_particles
        self.map = np.zeros((100, 100))  # Simple grid map

    def scan_callback(self, msg):
        """Update particles based on scan data."""
        # Simulate scan data processing
        for i in range(self.num_particles):
            # Hypothetical scan matching algorithm
            self.particles[i, 0] += np.random.randn() * 0.01  # Update x
            self.particles[i, 1] += np.random.randn() * 0.01  # Update y
            self.particles[i, 2] += np.random.randn() * 0.01  # Update theta
        self.update_map()

    def odom_callback(self, msg):
        """Update particles based on odometry data."""
        # Simulate movement based on odometry
        dx = np.random.randn() * 0.05
        dy = np.random.randn() * 0.05
        dtheta = np.random.randn() * 0.01
        for i in range(self.num_particles):
            self.particles[i, 0] += dx
            self.particles[i, 1] += dy
            self.particles[i, 2] += dtheta

    def update_map(self):
        """Update occupancy grid based on particle positions."""
        for x, y, theta in self.particles:
            ix, iy = int(x*10), int(y*10)
            if 0 <= ix < 100 and 0 <= iy < 100:
                self.map[ix, iy] += 1
        self.publish_map()

    def publish_map(self):
        """Publish the map."""
        map_msg = OccupancyGrid()
        map_msg.header.stamp = rospy.Time.now()
        map_msg.header.frame_id = "map"
        map_msg.info.resolution = 0.1  # 10 cm per grid cell
        map_msg.info.width = 100
        map_msg.info.height = 100
        map_msg.info.origin = Pose()  # Centered at (0,0,0)
        map_msg.data = (self.map.flatten() > 1).astype(int).tolist() * 100  # Simple thresholding for occupancy
        self.map_pub.publish(map_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    slam = SimpleSLAM()
    slam.run()
