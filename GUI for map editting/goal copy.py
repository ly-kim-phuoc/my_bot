#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Modified by AutomaticAddison.com
 
import time # Time library
 
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
import cv2
from robot_navigator import BasicNavigator, NavigationResult # Helper module
import csv
import pandas as pd
 
'''
Follow waypoints using the ROS 2 Navigation Stack (Nav2)
'''

MAP_FILE = "/home/phuoc/dev_ws/src/my_bot/maps/map.pgm"  # Path to your occupancy grid map file
MAP_FILE2 = "/home/phuoc/map2.png"  # Path to your occupancy grid map file
MAP_RESOLUTION = 0.05  # Meters per cell in the map
MAP_FRAME_ID = "map"  # Frame ID of the map
x_start = 0 #-4.51
y_start = 0 #-3.72
def main():
 
  # Start the ROS 2 Python Client Library
  rclpy.init()
  map_image = cv2.imread(MAP_FILE2, cv2.IMREAD_GRAYSCALE)
  df = pd.DataFrame(map_image)
  df[df != 254] = 1
  df = df.replace(to_replace=254,value=0)
  df.to_csv("map.csv",index=False,index_label=False,header=False)
  with open('map.csv', newline='') as f:
      reader = csv.reader(f,quoting = csv.QUOTE_NONNUMERIC)
      grid_map = list(reader)
# Initialize waypoints list
            # # Create and append PoseStamped message for waypoint
            # waypoint = PoseStamped()
            # waypoint.header.frame_id = MAP_FRAME_ID
            # waypoint.pose.position.x = x
            # waypoint.pose.position.y = y
            # waypoints.append(waypoint)
  cells = []
  visited = [[False for _ in row] for row in grid_map]  # Track visited cells

  for y in range(0,len(grid_map),6):
    for x in range(0,len(grid_map[0]),6):
      if not visited[y][x] and grid_map[y][x] == 0:
        cell = []
        dfs_stack = [(y, x)]
        while dfs_stack:
          y, x = dfs_stack.pop()
          cell.append([y, x])
          visited[y][x] = True

          # Explore neighbors in a DFS-like manner (iterative)
          for dy, dx in [(-6, 0), (6, 0), (0, -6), (0, 6)]:
            new_y, new_x = y + dy, x + dx
            if (new_y >= 0 and new_y < len(grid_map) and
                new_x >= 0 and new_x < len(grid_map[0]) and
                not visited[new_y][new_x] and grid_map[new_y][new_x] == 0):
              dfs_stack.append((new_y, new_x))

        cells.append(cell)
  # Launch the ROS 2 Navigation Stack
  navigator = BasicNavigator()
 
  # Wait for navigation to fully activate. Use this line if autostart is set to true.
  navigator.waitUntilNav2Active()
  previous_direction = None  # Initialize direction tracker
  previous_cell = None  # Initialize previous cell
  waypoints = []
  waypoint = PoseStamped()
  waypoint.header.frame_id = MAP_FRAME_ID
  waypoint.pose.position.x = cells[0][0][1]*MAP_RESOLUTION + x_start
  waypoint.pose.position.y = cells[0][0][0]*MAP_RESOLUTION + y_start
  waypoints.append(waypoint)
  for cell in cells[0]:
    if cell[0] != previous_cell[0] if previous_cell else False:
      waypoint = PoseStamped()
      waypoint.header.frame_id = MAP_FRAME_ID
      waypoint.pose.position.x = float(previous_cell[1])*MAP_RESOLUTION + x_start
      waypoint.pose.position.y = float(previous_cell[0])*MAP_RESOLUTION + y_start
      waypoint.pose.orientation.x = 0.0
      waypoint.pose.orientation.y = 0.0
      waypoint.pose.orientation.z = 1.0
      waypoint.pose.orientation.w = 0.0
      waypoints.append(waypoint)
      waypoint = PoseStamped()
      waypoint.header.frame_id = MAP_FRAME_ID
      waypoint.pose.position.x = float(cell[1])*MAP_RESOLUTION + x_start
      waypoint.pose.position.y = float(cell[0])*MAP_RESOLUTION + y_start
      waypoint.pose.orientation.x = 0.0
      waypoint.pose.orientation.y = 0.0
      waypoint.pose.orientation.z = 1.0
      waypoint.pose.orientation.w = 0.0
      waypoints.append(waypoint)
    # if cell[1] > (previous_cell[1] if previous_cell else 0) and previous_direction != "right":
    #   previous_direction = "right"
    # elif cell[1] < (previous_cell[1] if previous_cell else float('inf')) and previous_direction != "left":
    #   previous_direction = "left"
    previous_cell = cell

    # ... (existing code for processing the cell)
  waypoint = PoseStamped()
  waypoint.header.frame_id = MAP_FRAME_ID
  waypoint.pose.position.x = cells[0][-1][1]*MAP_RESOLUTION + x_start
  waypoint.pose.position.y = cells[0][-1][0]*MAP_RESOLUTION + y_start
  waypoints.append(waypoint)
  nav_start = navigator.get_clock().now()
  navigator.followWaypoints(waypoints)
  i = 0
  while not navigator.isNavComplete():
    ################################################
    #
    # Implement some code here for your application!
    #
    ################################################
 
    # Do something with the feedback
    i = i + 1
    feedback = navigator.getFeedback()
    if feedback and i % 5 == 0:
      print('Executing current waypoint: ' +
            str(feedback.current_waypoint + 1) + '/' + str(len(waypoints)))
      now = navigator.get_clock().now()
 
      # Some navigation timeout to demo cancellation
      if now - nav_start > Duration(seconds=100000000.0):
        navigator.cancelNav()
  # Do something depending on the return code
  result = navigator.getResult()
  if result == NavigationResult.SUCCEEDED:
    print('Goal succeeded!')
  elif result == NavigationResult.CANCELED:
    print('Goal was canceled!')
  elif result == NavigationResult.FAILED:
    print('Goal failed!')
  else:
    print('Goal has an invalid return status!')
 
 
  exit(0)
 
if __name__ == '__main__':
  main()
