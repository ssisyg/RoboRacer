#ifndef RACING_LINE_PLANNER_H
#define RACING_LINE_PLANNER_H

#include <ros/ros.h>
#include <vector>
#include <memory>
#include <cfloat>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/create_straight_skeleton_2.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>


/* TYPEDEFS */
using Kernel = CGAL::Simple_cartesian<double>;
using Point2 = Kernel::Point_2;

constexpr double GRAVITY = 9.80665;
constexpr double PI = 3.1415926535897932385;

using K = CGAL::Simple_cartesian<double>;
using Point2 = K::Point_2;
using Poly2 = CGAL::Polygon_2<K>;
using SsPtr = boost::shared_ptr<CGAL::Straight_skeleton_2<K> >;

class RacingLinePlanner
{
	// coefficient of friction
	double mu;
	// wheelbase of f1_tenth car (default 32.5 cm)
	double wheelbase;
	// max steering angle (default 30 deg)
	double maxSteerDeg;
	// maximum curvature the car can turn (calculated from parameters)
	double maxCurve;
	
	// Received outer and inner racing lines
	Poly2 outerLine, innerLine;
	
	/* ROS SUB AND PUB */
	ros::Subscriber outerLineSub, innerLineSub;
	ros::Publisher racingLinePub, rawCenterLinePub;
	
public:
	/* CONSTRUCTOR */
	RacingLinePlanner(ros::NodeHandle& nh);
	
	/* form the Polygon for the outer line */
	void OuterLineCallback(const nav_msgs::Path& msg);

	/* form the Polygon for the inner line */
	void InnerLineCallback(const nav_msgs::Path& msg);
	
	void ComputeRacingLine();
		
/* HELPERS */
private:

	/* BULK OF ALGORITHM */
	void ComputeCenterLine(std::vector<Point2>& centerLine, const Poly2& outer, const Poly2& inner);
	
	/* SMOOTHER (OPTIONAL) */
	void SmoothLine(std::vector<Point2>& smoothLine, const std::vector<Point2>& line, int window=5);

	void LineToPath(nav_msgs::Path& path, const std::vector<Point2>& line);	
	
	void ResampleLine(std::vector<Point2>& resampledLine, const Poly2& line, size_t resampleLength);
	
	static void PathToPoly2(const nav_msgs::Path& path, Poly2& poly2);
	
	
};

#endif // RACING_LINE_PLANNER_H
