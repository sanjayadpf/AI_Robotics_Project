#!/usr/bin/env python3
import rospy
import actionlib
import numpy as np
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
import time

class Auto:
    def __init__(self):
        """ Initialize environment """
        self.rate = rospy.Rate(1)
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5.0))
        rospy.logdebug("move_base is ready")
        #End goal
        self.goal_x = 3
        self.goal_y = -2

        # Robot's current position
        self.robot_x = 0
        self.robot_y = 0

        # Subscribers
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.map = OccupancyGrid()
        time.sleep(8)

    def map_callback(self, data):
        """ Handle updates to the map """
        closest = None
        min_dist = float('inf')
        buffer_zone = 3  # number of cells to check around a potential goal for obstacles or unknowns

        resolution = data.info.resolution
        origin_x = data.info.origin.position.x
        origin_y = data.info.origin.position.y
        width = data.info.width

        def is_safe(x, y):
            """ Check the surroundings of the point to ensure it's free of obstacles and unknown areas """
            cell_x = int((x - origin_x) / resolution)
            cell_y = int((y - origin_y) / resolution)
            for dx in range(-buffer_zone, buffer_zone + 1):
                for dy in range(-buffer_zone, buffer_zone + 1):
                    nx, ny = cell_x + dx, cell_y + dy
                    if 0 <= nx < width and 0 <= ny < width:
                        idx = ny * width + nx
                        if data.data[idx] != 0:
                            return False
            return True

        for idx, value in enumerate(data.data):
            if value == 0:  # Free space
                col = idx % width
                row = idx // width
                x = col * resolution + origin_x
                y = row * resolution + origin_y

                if self.in_radius(x, y) and is_safe(x, y):
                    dist_to_robot = np.hypot(self.robot_x - x, self.robot_y - y)
                    if dist_to_robot <= 2.0:
                        dist_to_goal = np.hypot(self.goal_x - x, self.goal_y - y)
                        if dist_to_goal < min_dist:
                            min_dist = dist_to_goal
                            closest = (x, y)

        if closest:
            self.set_goal(*closest)

    def in_radius(self, x, y):
        """ Check if point is within a 2m radius of the robot """
        return np.hypot(self.robot_x - x, self.robot_y - y) <= 1

    def set_goal(self, x, y):
        """ Set a goal position for the robot """
        rospy.logdebug("Setting goal")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0
        rospy.logdebug(f"goal: {x}, {y}")
        self.move_base.send_goal(goal)
        self.wait_for_goal_completion()

    def wait_for_goal_completion(self):
        """ Wait for the robot to reach the goal """
        self.move_base.wait_for_result()
        if self.move_base.get_state() == GoalStatus.SUCCEEDED:
            rospy.logdebug("Goal reached successfully")
        else:
            rospy.logwarn("Failed to reach the goal")

    def odom_callback(self, data):
        """ Update the current position of the robot """
        self.robot_x = data.pose.pose.position.x
        self.robot_y = data.pose.pose.position.y

def main():
    rospy.init_node('explore', log_level=rospy.DEBUG)
    explorer = Explore()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
