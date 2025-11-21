#ifndef F1_CONTROLLER_H
#define F1_CONTROLLER_H

// This is a PID Path following controller
// It should subscribe to the ideal racing line, and obstacle avoiding racing line, and other movement controls
// It is responsible for switching to different modes of control appropriately

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <CGAL/Simple_cartesian.h>

struct RacingPoint
{
	double x, y, yaw;
	
	RacingPoint(double xVal, double yVal, double yawVal) : x(xVal), y(yVal), yaw(yawVal)
	{}
};

class F1Controller
{
	/* subscribers */
	ros::Subscriber poseSub;
	ros::Subscriber pathSub; 
	
	/* pubs */
	ros::Publisher drivePub;
	
	/* Other ros */
	ros::Time prevTime;
	geometry_msgs::PoseStamped prevPose, currentPose;
	nav_msgs::Path currentPath;
	
	/* PID PARAMETERS */
	double kp, ki, kd, angleKp, lookaheadKp, lookaheadAngleKp, correctionFactor;
	double prevError;
	double lookaheadTime;
	
	/* CAR SPECS */
	double wheelbase, maxSteerRad;
	
	/* other data */
	double velocity;
	std::vector<RacingPoint> racingLine;
	// current pose of car
	double carX, carY, carYaw;
	/* flags: 
		prevPoseExists means previous kalman pose reference is available 
		locality means previous racing point index is available 
	*/
	bool prevPoseExists, locality;
	/* flag for killing wall pid (default controller if racing line not available */
	bool nodeKilled;
	size_t prevRacePoint;

public:
	/* CONSTRUCTOR */
	F1Controller(ros::NodeHandle& nh);
	
	/* FUNCTION DEFS */
	void DriveCar(double dutyCycle, double steering);
	void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void PathCallback(const nav_msgs::Path& msg);
	

private:
	size_t ExtractNearestLateralPoint(void);
	size_t ExtractNearestLateralPoint(size_t currentIndex);
};


#endif // F1_CONTROLLER_H
