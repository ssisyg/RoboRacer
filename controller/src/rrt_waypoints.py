#!/usr/bin/env python3

import random
import math
import numpy as np

class Node:
    def __init__(self, x, y, theta=None, parent=None):
        self.x = x
        self.y = y
        self.theta = theta  # orientation in radians
        self.parent = parent

def is_free(x, y, map_img):
    h, w = map_img.shape[:2]
    if 0 <= x < w and 0 <= y < h:
        return np.all(map_img[y, x] > 200)  # Free if pixel is bright
    return False

def distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)

def nearest(nodes, rnd_node):
    return min(nodes, key=lambda node: distance(node, rnd_node))

def steer(from_node, to_node, extend_length=10.0):
    d = distance(from_node, to_node)
    if d < extend_length:
        theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        return Node(to_node.x, to_node.y, theta, from_node)
    theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
    new_x = int(from_node.x + extend_length * math.cos(theta))
    new_y = int(from_node.y + extend_length * math.sin(theta))
    return Node(new_x, new_y, theta, from_node)

def smooth_path(path_px, map_img, max_iter=200):
    """
    Try to shortcut between random pairs of points in the path
    if the direct connection is obstacle-free.
    """
    if len(path_px) < 3:
        return path_px  # nothing to smooth

    for _ in range(max_iter):
        # Pick two random indices i < j
        i = random.randint(0, len(path_px)-2)
        j = random.randint(i+1, len(path_px)-1)

        p1 = Node(path_px[i][0], path_px[i][1])
        p2 = Node(path_px[j][0], path_px[j][1])

        if check_path(p1, p2, map_img):
            # Replace intermediate path segment
            new_segment = path_px[:i+1] + path_px[j:]
            path_px = new_segment

    return path_px


def check_path(from_node, to_node, map_img):
    steps = int(distance(from_node, to_node))
    if steps == 0:
        return True
    for i in range(steps + 1):
        x = int(from_node.x + (to_node.x - from_node.x) * i / steps)
        y = int(from_node.y + (to_node.y - from_node.y) * i / steps)
        if not is_free(x, y, map_img):
            return False
    return True

def angle_diff(a, b):
    """Shortest difference between two angles [-pi, pi]"""
    return (a - b + math.pi) % (2 * math.pi) - math.pi

def desired_heading_at(x, y, waypoints, lookahead=5):
    """
    Given a map pixel coordinate (x, y), find nearest segment in waypoints
    and return the heading angle of that segment.
    """
    if len(waypoints) < 2:
        return None

    min_dist = float("inf")
    closest_idx = 0
    for i in range(len(waypoints) - 1):
        wx, wy = waypoints[i]
        dist = math.hypot(wx - x, wy - y)
        if dist < min_dist:
            min_dist = dist
            closest_idx = i

    idx2 = min(closest_idx + lookahead, len(waypoints) - 1)
    dx = waypoints[idx2][0] - waypoints[closest_idx][0]
    dy = waypoints[idx2][1] - waypoints[closest_idx][1]

    if dx == 0 and dy == 0:
        return None

    heading = math.atan2(dy, dx)
    return heading

def rrt_planner(map_img, start_px, goal_px,
                max_iter=10000, goal_sample_rate=0.1,
                extend_length=5.0, waypoints=None,
                max_allowed_heading_error=math.radians(45)):
    start_theta = None
    if waypoints is not None and len(waypoints) > 1:
        start_theta = desired_heading_at(start_px[0], start_px[1], waypoints)
    start_node = Node(*start_px, start_theta)
    goal_node = Node(*goal_px)
    nodes = [start_node]

    for _ in range(max_iter):
        if random.random() < goal_sample_rate:
            rnd_node = Node(goal_node.x, goal_node.y)
        else:
            rnd_node = Node(
                random.randint(0, map_img.shape[1] - 1),
                random.randint(0, map_img.shape[0] - 1)
            )
            if not is_free(rnd_node.x, rnd_node.y, map_img):
                continue

        nearest_node = nearest(nodes, rnd_node)
        new_node = steer(nearest_node, rnd_node, extend_length)

        if not is_free(new_node.x, new_node.y, map_img):
            continue
        if not check_path(nearest_node, new_node, map_img):
            continue

        # Heading constraint
        if waypoints is not None:
            desired_theta = desired_heading_at(new_node.x, new_node.y, waypoints)
            if desired_theta is not None:
                heading_diff = angle_diff(new_node.theta, desired_theta)
                if abs(heading_diff) > max_allowed_heading_error:
                    continue  # discard this node

        nodes.append(new_node)

        if distance(new_node, goal_node) < extend_length:
            if check_path(new_node, goal_node, map_img):
                goal_node.parent = new_node
                nodes.append(goal_node)
                break

    # Trace path
    path = []
    node = goal_node if goal_node in nodes else nodes[-1]
    while node:
        path.append((node.x, node.y))
        node = node.parent
    path =  path[::-1]
    
    # path = smooth_path(path, map_img)
    return path
    
def rrt_pathfinder(start_world, goal_world, map_img, resolution, origin, waypoints_px=None):
    img_height = map_img.shape[0]

    def world_to_map(x, y):
        dx = x - origin[0]
        dy = y - origin[1]
        map_x = int(dx / resolution)
        map_y = map_img.shape[0] - int(dy / resolution)
        return (map_x, map_y)

    def map_to_world_path(path_px):
        return [
            (origin[0] + px * resolution,
             origin[1] + (img_height - py) * resolution)
            for px, py in path_px
        ]

    start_px = world_to_map(*start_world)
    goal_px = world_to_map(*goal_world)

    path_px = rrt_planner(
        map_img,
        start_px,
        goal_px,
        waypoints=waypoints_px
    )
    path_world = map_to_world_path(path_px)

    return start_px, goal_px, path_px, path_world

