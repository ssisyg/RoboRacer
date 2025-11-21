#include "initial/f1_controller.h"
#include <tf/tf.h>
#include <cmath>
#include <limits>
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>
#include <type_traits>
#include <cstdlib>

template <typename T, typename L, typename H>
static auto clamp(T value, L lowerBound, H upperBound) -> std::common_type_t<T, L, H>
{
	using CommonType = std::common_type_t<T, L, H>;
	CommonType copyValue = static_cast<CommonType>(value);
	CommonType copyLowerBound = static_cast<CommonType>(lowerBound);
	CommonType copyHigherBound = static_cast<CommonType>(upperBound);
	
	return (copyValue < copyLowerBound) ? copyLowerBound : ( (copyValue > copyHigherBound) ? copyHigherBound : copyValue);
}

F1Controller::F1Controller(ros::NodeHandle& nh) 
{
	ros::NodeHandle pnh("~");
	pnh.param("kp", kp, 1.0);
	pnh.param("ki", ki, 0.0);
	pnh.param("kd", kd, 0.0);
	pnh.param("angle_kp", angleKp, 0.25);
	pnh.param("lookahead_kp", lookaheadKp, 0.5);
	pnh.param("lookahead_angle_kp", lookaheadAngleKp, 0.1);
	pnh.param("correction_factor", correctionFactor, 0.1);
	pnh.param("lookahead_time", lookaheadTime, 0.5);
	pnh.param("wheelbase", wheelbase, 0.325);
	double maxSteerDeg = 0.0;
	pnh.param("max_steer_deg", maxSteerDeg, 30.0);
	
	maxSteerRad = maxSteerDeg * M_PI / 180.0;
	prevError = 0.0;
	prevPoseExists = locality = false;
	prevRacePoint = 0;
	velocity = 0.0;
	carX = carY = carYaw = 0.0;
	nodeKilled = false;
	
	poseSub = nh.subscribe("/kalman_slam", 1, &F1Controller::PoseCallback, this);
	pathSub = nh.subscribe("/racing_line", 1, &F1Controller::PathCallback, this);
	
	drivePub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

void F1Controller::DriveCar(double vel, double steering)
{	
	/* remap steering to 0 ~ 1 */ 
	double steerNormalized = (steering / maxSteerRad + 1) / 2.0;

	geometry_msgs::Twist drive;
	drive.linear.x = vel;
	drive.angular.z = steerNormalized;
	
	drivePub.publish(drive);
}

void F1Controller::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	if (racingLine.empty()) return;
	
	/* if !racingLine.empty(), means we can kill wall_pid node */
	if (!nodeKilled)
	{
		ROS_INFO("RACING LINE AVAILABLE, KILLING WALL PID");
		ros::Duration(0.1).sleep();
		system("rosnode kill /wall_follow_node");
	}

	/* current pose */
	carX = msg->pose.position.x;
	carY = msg->pose.position.y;
	carYaw = tf::getYaw(msg->pose.orientation);
	double dt = 0.0;
	
	if (prevPoseExists)
	{
		double dx = carX - prevPose.pose.position.x;
		double dy = carY - prevPose.pose.position.y;
		dt = (msg->header.stamp - prevTime).toSec();
		
		if (dt > 0.01)
			velocity = std::sqrt(dx * dx + dy * dy) / dt;
		ROS_INFO("SPEED: %.2f", velocity);
	}
	
	/* update prevPose */
	prevPose = *msg;
	prevPoseExists = true;	
	
	size_t nearestIndex = ExtractNearestLateralPoint();
	
	/* calculate lateral error */
	double lateralErrorX = racingLine[nearestIndex].x - carX;
	double lateralErrorY = racingLine[nearestIndex].y - carY;
	double lateralError = -std::sin(carYaw) * lateralErrorX + std::cos(carYaw) * lateralErrorY;
	/* error in how the car should be facing (-pi <= angle error < pi) */
	double angleError = angles::shortest_angular_distance(carYaw, racingLine[nearestIndex].yaw);
	
	
	
	/* raw PID */
	double steerRaw; 
	if (dt != 0.0)
		steerRaw = kp * lateralError + ki * lateralError * dt + kd * (lateralError - prevError) / dt;
	else
		steerRaw = kp * lateralError;
	
	steerRaw += angleKp * angleError;
	steerRaw = clamp(steerRaw, -maxSteerRad, maxSteerRad);
	
	/* calculate lateral error of lookahead distance */
	double angularDisp = std::tan(steerRaw) * lookaheadTime / wheelbase;
	double predictedX = carX + velocity * lookaheadTime * std::cos(carYaw + angularDisp);
	double predictedY = carY + velocity * lookaheadTime * std::sin(carYaw + angularDisp);
	double predictedYaw = carYaw + angularDisp;
	
	size_t nearestLookaheadIndex = ExtractNearestLateralPoint(nearestIndex);
	const RacingPoint& lookaheadRacingPoint = racingLine[nearestLookaheadIndex];
	
	double lookaheadLateralErrorX = lookaheadRacingPoint.x - predictedX;
	double lookaheadLateralErrorY = lookaheadRacingPoint.y - predictedY;
	double lookaheadLateralError = -std::sin(predictedYaw) * lookaheadLateralErrorX + 
					std::cos(predictedYaw) * lookaheadLateralErrorY;
	
	double lookaheadAngleError = angles::shortest_angular_distance(predictedYaw, lookaheadRacingPoint.yaw);
	
	/* correct the steerRaw based on possible future errors from lookahead */ 
	double correction = lookaheadKp * lookaheadLateralError + lookaheadAngleKp * lookaheadAngleError;
	double correctedSteer = steerRaw + correctionFactor * correction;
	correctedSteer = clamp(correctedSteer, -maxSteerRad, maxSteerRad);
	
	/* update ros Time */
	prevTime = msg->header.stamp;
	
	/* Publish Ackermann */
	DriveCar(0.6, correctedSteer);
	
	/* update the racing point the car is closest to */
	prevRacePoint = nearestIndex;
	prevError = lateralError;
}

void F1Controller::PathCallback(const nav_msgs::Path& msg)
{
	racingLine.clear();
		
		for (const auto& pose : msg.poses)
		{
			double yaw = tf::getYaw(pose.pose.orientation);
			
			racingLine.emplace_back(pose.pose.position.x, pose.pose.position.y, yaw);
		}
}

/* extract nearest lateral point */
size_t F1Controller::ExtractNearestLateralPoint(void)
{
	double minDist = std::numeric_limits<double>::max();
	int racingLineLength = racingLine.size();
	int searchWindow = clamp(int(velocity * racingLineLength * lookaheadTime/ 10.0), 
						racingLineLength / 10.0, racingLineLength / 3.0);
	size_t nearestIndex = prevRacePoint;
	
	// if first iteration, must do brute force search as we have no locality to take advantage of
	if (!locality)
	{
		for (size_t i = 0; i < racingLineLength; ++i)
		{
			double dx = racingLine[i].x - carX;
			double dy = racingLine[i].y - carY;
			double squared = dx * dx + dy * dy;
			
			if (squared < minDist)
			{
				minDist = squared;
				nearestIndex = i;
			}
		}	
		locality = true;
	}
	else
	{
		for (int i = -searchWindow; i <= searchWindow; ++i)
		{
			int index = (prevRacePoint + i + racingLineLength) %  racingLineLength;
			double dx = racingLine[index].x - carX;
			double dy = racingLine[index].y - carY;
			double squared = dx * dx + dy * dy;
			
			if (squared < minDist)
			{
				minDist = squared;
				nearestIndex = index;
			}
		}
	}
	return nearestIndex;
}

/* extract nearest lateral point */
size_t F1Controller::ExtractNearestLateralPoint(size_t currentIndex)
{
	double minDist = std::numeric_limits<double>::max();
	int racingLineLength = racingLine.size();
	int searchWindow = clamp(int(velocity * racingLineLength * lookaheadTime/ 10.0), 
						racingLineLength / 10.0, racingLineLength / 3.0);
	size_t nearestIndex = currentIndex;

	for (int i = -searchWindow; i <= searchWindow; ++i)
	{
		int index = (currentIndex + i + racingLineLength) %  racingLineLength;
		double dx = racingLine[index].x - carX;
		double dy = racingLine[index].y - carY;
		double squared = dx * dx + dy * dy;
		
		if (squared < minDist)
		{
			minDist = squared;
			nearestIndex = index;
		}
	}
	return nearestIndex;
}



