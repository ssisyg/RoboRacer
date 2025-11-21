#!/usr/bin/env python3
import rospy
from enum import Enum
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point
from scipy.interpolate import splprep, splev
from rcpsl_f1_10 import TrackedObjectArray, TrackedObject

class FSMState(Enum):
    LANE_FOLLOWING = 1
    AVOID_STATIC = 2
    OVERTAKE_DYNAMIC = 3

class BehavioralPlannerFSM:
    def __init__(self):
        rospy.init_node('behavioral_planner_fsm_node')

        # --- Parameters (CRITICAL TO TUNE!) ---
        self.decision_lookahead_dist = rospy.get_param('~decision_lookahead_dist', 5.0) # m, how far ahead to look for obstacles
        self.lateral_safety_margin = rospy.get_param('~lateral_safety_margin', 0.5) # m, side clearance for avoidance
        self.overtake_prediction_time = rospy.get_param('~overtake_prediction_time', 1.5) # s, how far in time to predict opponent's move
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.path_num_points = 50 # Number of points in the generated local path

        # --- Internal Variables ---
        self.current_state = FSMState.LANE_FOLLOWING
        self.global_path = None
        self.current_pose = None
        self.obstacles = []
        self.target_obstacle = None

        # --- ROS Subscribers/Publishers ---
        self.global_path_sub = rospy.Subscriber('/racing_line', Path, self.global_path_callback, queue_size=1)
        self.pose_sub = rospy.Subscriber('/amcl_pose', PoseStamped, self.pose_callback, queue_size=1) # Or your localization topic
        self.obstacles_sub = rospy.Subscriber('/lidar_obstacles', TrackedObjectArray, self.obstacles_callback, queue_size=1)
        
        self.target_path_pub = rospy.Publisher('/target_path', Path, queue_size=1)
        
        # Main FSM loop runs at 10 Hz
        rospy.Timer(rospy.Duration(0.1), self.run_fsm)

    def global_path_callback(self, msg):
        self.global_path = msg

    def pose_callback(self, msg):
        self.current_pose = msg.pose

    def obstacles_callback(self, msg):
        self.obstacles = msg.objects

    def run_fsm(self, event):
        if self.global_path is None or self.current_pose is None:
            rospy.loginfo_throttle(5, "Waiting for global path and pose...")
            return

        self.check_transitions()
        self.execute_current_state()
            
    def check_transitions(self):
        self.target_obstacle = self.find_closest_obstacle_on_path()
        
        new_state = FSMState.LANE_FOLLOWING
        if self.target_obstacle:
            if self.target_obstacle.class_name == "STATIC":
                new_state = FSMState.AVOID_STATIC
            elif self.target_obstacle.class_name == "DYNAMIC":
                new_state = FSMState.OVERTAKE_DYNAMIC

        if self.current_state != new_state:
            rospy.loginfo(f"FSM Transition: {self.current_state.name} -> {new_state.name}")
            self.current_state = new_state

    def execute_current_state(self):
        if self.current_state == FSMState.LANE_FOLLOWING:
            self.execute_lane_following()
        elif self.current_state == FSMState.AVOID_STATIC:
            self.execute_avoid_static()
        elif self.current_state == FSMState.OVERTAKE_DYNAMIC:
            self.execute_overtake_dynamic()

    def execute_lane_following(self):
        # Simply publish the global path
        self.target_path_pub.publish(self.global_path)

    def execute_avoid_static(self):
        if self.target_obstacle is None:
            self.current_state = FSMState.LANE_FOLLOWING
            return

        # Simple logic: always try to pass on the left
        # A better implementation would check map boundaries
        obstacle_pos = self.target_obstacle.pose.position
        
        waypoints = self.generate_avoidance_waypoints(obstacle_pos, is_dynamic=False)
        local_path = self.generate_spline_path(waypoints)
        self.target_path_pub.publish(local_path)

    def execute_overtake_dynamic(self):
        if self.target_obstacle is None:
            self.current_state = FSMState.LANE_FOLLOWING
            return
        
        # Predict opponent's future position
        obstacle_pos = self.target_obstacle.pose.position
        obstacle_vel = self.target_obstacle.velocity.linear
        predicted_pos = Point()
        predicted_pos.x = obstacle_pos.x + obstacle_vel.x * self.overtake_prediction_time
        predicted_pos.y = obstacle_pos.y + obstacle_vel.y * self.overtake_prediction_time
        
        waypoints = self.generate_avoidance_waypoints(predicted_pos, is_dynamic=True)
        local_path = self.generate_spline_path(waypoints)
        self.target_path_pub.publish(local_path)

    def generate_avoidance_waypoints(self, obstacle_pos, is_dynamic):
        start_point = np.array([self.current_pose.position.x, self.current_pose.position.y])
        
        # Find a point on the global path to merge back into
        merge_point_dist = self.decision_lookahead_dist * 2.0
        merge_point = self.get_point_on_path(merge_point_dist)
        
        # Calculate a point to the side of the obstacle
        # This assumes the obstacle is roughly aligned with the path
        path_angle = np.arctan2(merge_point[1] - start_point[1], merge_point[0] - start_point[0])
        lateral_offset_angle = path_angle + np.pi / 2.0
        
        side_point = np.array([
            obstacle_pos.x + self.lateral_safety_margin * np.cos(lateral_offset_angle),
            obstacle_pos.y + self.lateral_safety_margin * np.sin(lateral_offset_angle)
        ])
        
        return [start_point, side_point, merge_point]

    def find_closest_obstacle_on_path(self):
        # A simplified check. A robust version would check lateral distance to the path.
        closest_dist = float('inf')
        target_obs = None
        
        my_pos = self.current_pose.position
        
        for obs in self.obstacles:
            obs_pos = obs.pose.position
            dist_to_car = np.hypot(my_pos.x - obs_pos.x, my_pos.y - obs_pos.y)
            
            if dist_to_car < self.decision_lookahead_dist and dist_to_car < closest_dist:
                # Check if obstacle is generally in front of the car
                # This is a simple dot product check
                vec_to_obs = np.array([obs_pos.x - my_pos.x, obs_pos.y - my_pos.y])
                
                q = self.current_pose.orientation
                yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                car_heading_vec = np.array([np.cos(yaw), np.sin(yaw)])
                
                if np.dot(vec_to_obs, car_heading_vec) > 0: # It's in front
                    closest_dist = dist_to_car
                    target_obs = obs
                    
        return target_obs

    def get_point_on_path(self, dist_ahead):
        path_points = np.array([[p.pose.position.x, p.pose.position.y] for p in self.global_path.poses])
        my_pos = np.array([self.current_pose.position.x, self.current_pose.position.y])
        
        # Find closest point on path to the car
        dists = np.linalg.norm(path_points - my_pos, axis=1)
        start_idx = np.argmin(dists)
        
        # Travel along the path
        total_dist = 0
        for i in range(start_idx, len(path_points) - 1):
            total_dist += np.linalg.norm(path_points[i+1] - path_points[i])
            if total_dist >= dist_ahead:
                return path_points[i+1]
        
        return path_points[-1] # Return last point if not found

    def generate_spline_path(self, waypoints):
        if len(waypoints) < 2:
            return self.global_path # Fallback

        waypoints = np.array(waypoints)
        x = waypoints[:, 0]
        y = waypoints[:, 1]
        
        try:
            # Fit B-spline
            tck, u = splprep([x, y], s=0, k=min(len(waypoints)-1, 3))
            
            # Evaluate spline
            u_new = np.linspace(u.min(), u.max(), self.path_num_points)
            x_new, y_new = splev(u_new, tck, der=0)

            # Create Path message
            local_path = Path()
            local_path.header.stamp = rospy.Time.now()
            local_path.header.frame_id = self.map_frame
            for i in range(len(x_new)):
                pose = PoseStamped()
                pose.header = local_path.header
                pose.pose.position.x = x_new[i]
                pose.pose.position.y = y_new[i]
                local_path.poses.append(pose)
            
            return local_path
        except ValueError:
            # Fallback to global path if spline fails
            return self.global_path

if __name__ == '__main__':
    try:
        node = BehavioralPlannerFSM()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass