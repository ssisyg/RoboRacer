#!/usr/bin/env python3

import cv2
import numpy as np
import csv
from rrt_waypoints import rrt_planner

# ----------------------------------------
# CONFIGURATION
# ----------------------------------------

pgm_filename = "f1_map_fast.pgm"

# Replace these values from your .yaml file
resolution = 0.05  # meters per pixel
origin = (-25.624998, -25.624998)

start_world = (0.0, 0.0)
goal_world = (-0.2, 0.08)

buffer_distance_m = 0.3

# ----------------------------------------
# LOAD MAP IMAGE
# ----------------------------------------

map_img = cv2.imread(pgm_filename, cv2.IMREAD_GRAYSCALE)

if map_img is None:
    raise FileNotFoundError(f"Could not load map image: {pgm_filename}")

height_px, width_px = map_img.shape
print(f"Map size: {width_px} x {height_px} pixels")

# ----------------------------------------
# CREATE BUFFER AROUND WALLS
# ----------------------------------------

buffer_px = int(buffer_distance_m / resolution)
print(f"Inflating obstacles by {buffer_px} pixels (~{buffer_distance_m*100:.0f} cm buffer)")

kernel_size = buffer_px * 2 + 1
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))

obstacles = cv2.threshold(map_img, 127, 255, cv2.THRESH_BINARY_INV)[1]
inflated_obstacles = cv2.dilate(obstacles, kernel, iterations=1)
inflated_map = cv2.threshold(inflated_obstacles, 127, 255, cv2.THRESH_BINARY_INV)[1]

cv2.imshow("Inflated Map", inflated_map)
cv2.waitKey(0)

# ----------------------------------------
# COORDINATE CONVERSIONS
# ----------------------------------------

def world_to_pixel(wx, wy, map_height, origin, resolution):
    px = int((wx - origin[0]) / resolution)
    py = int(map_height - ((wy - origin[1]) / resolution) - 1)
    return (px, py)

def pixel_to_world(px, py, map_height, origin, resolution):
    wx = origin[0] + px * resolution
    wy = origin[1] + (map_height - py - 1) * resolution
    return (wx, wy)

# ----------------------------------------
# LOAD WAYPOINTS
# ----------------------------------------

waypoints = []
with open("waypoints.csv", "r") as f:
    reader = csv.reader(f)
    for row in reader:
        wx, wy = float(row[0]), float(row[1])
        px, py = world_to_pixel(wx, wy, height_px, origin, resolution)
        waypoints.append((px, py))

print(f"Loaded {len(waypoints)} waypoints.")

# ----------------------------------------
# CONVERT START & GOAL
# ----------------------------------------

start_px = world_to_pixel(start_world[0], start_world[1], height_px, origin, resolution)
goal_px = world_to_pixel(goal_world[0], goal_world[1], height_px, origin, resolution)

print(f"Start pixel: {start_px}")
print(f"Goal pixel: {goal_px}")

# ----------------------------------------
# RUN RRT
# ----------------------------------------

path_px = rrt_planner(
    inflated_map,
    start_px,
    goal_px,
    max_iter=100000,
    goal_sample_rate=0.01,
    extend_length=5.0,
    waypoints=waypoints
)

if len(path_px) == 0:
    print("No path found.")
else:
    print(f"Found path with {len(path_px)} points.")

# ----------------------------------------
# CONVERT PATH BACK TO WORLD
# ----------------------------------------

path_world = [
    pixel_to_world(px, py, height_px, origin, resolution)
    for (px, py) in path_px
]

print("RRT path in world coordinates:")
for point in path_world:
    print(f"{point[0]:.3f}, {point[1]:.3f}")

# ----------------------------------------
# VISUALIZE PATH
# ----------------------------------------

map_rgb = cv2.cvtColor(map_img, cv2.COLOR_GRAY2BGR)

for (px, py) in path_px:
    cv2.circle(map_rgb, (px, py), 2, (0, 0, 255), -1)

cv2.circle(map_rgb, start_px, 5, (0, 255, 0), -1)
cv2.circle(map_rgb, goal_px, 5, (255, 0, 0), -1)

cv2.imshow("RRT Path", map_rgb)
cv2.waitKey(0)
cv2.destroyAllWindows()

