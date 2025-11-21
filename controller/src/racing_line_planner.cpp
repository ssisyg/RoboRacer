#include "initial/racing_line_planner.h"
#include <limits>
#include <cmath>
#include <algorithm>
#include <cassert>
#include <tf/tf.h>

RacingLinePlanner::RacingLinePlanner(ros::NodeHandle& nh)
{
	ros::NodeHandle pnh("~");
	
	// Read parameters
	pnh.param("friction", mu, 0.5);
	pnh.param("base_length", wheelbase, 0.325);
	pnh.param("steer_max_deg", maxSteerDeg, 30.0);
	
	// Curvature
	double maxSteerRad = maxSteerDeg * PI / 180.0;
	maxCurve = 1.0 / (wheelbase / std::tan(maxSteerRad));
	
	// make sure polygon is empty
	outerLine.clear();
	innerLine.clear();

	outerLineSub = nh.subscribe("outer_geometry_path", 1, &RacingLinePlanner::OuterLineCallback, this);
	innerLineSub = nh.subscribe("inner_geometry_path", 1, &RacingLinePlanner::InnerLineCallback, this);
	
	rawCenterLinePub = nh.advertise<nav_msgs::Path>("raw_center_line", 1, true); 
	racingLinePub = nh.advertise<nav_msgs::Path>("racing_line", 1, true); 
}

/* CALLBACKS */

void RacingLinePlanner::OuterLineCallback(const nav_msgs::Path& msg)
{
	PathToPoly2(msg, this->outerLine);
	
	if (innerLine.size() > 0)
		ComputeRacingLine();
}


void RacingLinePlanner::InnerLineCallback(const nav_msgs::Path& msg)
{
	PathToPoly2(msg, this->innerLine);
	
	if (outerLine.size() > 0)
		ComputeRacingLine();
}


/* MAIN Publisher */
void RacingLinePlanner::ComputeRacingLine()
{
	if (outerLine.is_empty() || innerLine.is_empty()) return;
	
	std::vector<Point2> centerLine, smoothCenterLine;
	nav_msgs::Path rawPath, racePath;
	
	ComputeCenterLine(centerLine, outerLine, innerLine);
	SmoothLine(smoothCenterLine, centerLine);
	
	LineToPath(rawPath, centerLine);
	LineToPath(racePath, smoothCenterLine);

	rawCenterLinePub.publish(rawPath);
	racingLinePub.publish(racePath);
}

void RacingLinePlanner::ComputeCenterLine(std::vector<Point2>& centerLine, const Poly2& outer, const Poly2& inner)
{
	/* resample points */
	size_t resampleLength = (outer.size() + inner.size()) / 2.0 * 1.5;
	
	
	/* resample line */
	std::vector<Point2> outerResampled, innerResampled;
	
	ResampleLine(outerResampled, outer, resampleLength);
	ResampleLine(innerResampled, inner, resampleLength);
	
	assert(outerResampled.size() == innerResampled.size());
	
	centerLine.clear();
	centerLine.reserve(resampleLength);
	for (size_t i = 0; i < resampleLength; ++i)
	{
		double centerX = (outerResampled[i].x() + innerResampled[i].x()) / 2.0;
		double centerY = (outerResampled[i].y() + innerResampled[i].y()) / 2.0;
		centerLine.emplace_back(centerX, centerY);
	}
	
}

/* HELPER DEFINITIONS */
void RacingLinePlanner::PathToPoly2(const nav_msgs::Path& path, Poly2& poly)
{
	poly.clear();
	
	for (auto pose : path.poses)
	{
		poly.push_back(Point2(pose.pose.position.x, pose.pose.position.y));
	}
}

void RacingLinePlanner::LineToPath(nav_msgs::Path& path, const std::vector<Point2>& line)
{
	size_t lineSize = line.size();
	
	path.header.frame_id = "map";
	path.poses.reserve(lineSize);
	
	for (size_t i = 0; i < lineSize; ++i)
	{
		const auto& point = line[i];
		geometry_msgs::PoseStamped pose;
		
		pose.header.frame_id = "map";
		pose.pose.position.x = point.x();
		pose.pose.position.y = point.y();
		pose.pose.position.z = 0;
		
		
		/* compute angle via trig*/
		size_t nextPoint = (i + 1) % lineSize;
		double dx = line[nextPoint].x() - point.x();
		double dy = line[nextPoint].y() - point.y();
		double yaw = std::atan2(dy, dx);
		
		pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
		path.poses.push_back(pose);
	}
}

void RacingLinePlanner::ResampleLine(std::vector<Point2>& resampleLine, const Poly2& origPoly, size_t resampleLength)
{

	// cumulative arc length at each vertex
	std::vector<double> arcLengths;
	arcLengths.reserve(origPoly.size() + 1);
	arcLengths.push_back(0.0);
	size_t polyLen = origPoly.size();
	
	for (size_t i = 1; i < polyLen; ++i)
	{
		double dx = origPoly[i].x() - origPoly[i - 1].x();
		double dy = origPoly[i].y() - origPoly[i - 1].y();
		arcLengths.push_back(arcLengths.back() + std::hypot(dx, dy));
	}
	
	// Connect last vertex to the first vertex to complete loop
	double dx = origPoly[0].x() - origPoly[polyLen - 1].x();
	double dy = origPoly[0].y() - origPoly[polyLen - 1].y();
	arcLengths.push_back(arcLengths.back() + std::hypot(dx, dy));
	
	double totalLength = arcLengths.back();
	// smoothing increment
	double step = totalLength / (resampleLength - 1);
	 
	resampleLine.clear();
	resampleLine.reserve(resampleLength);
	
	size_t segment = 0;
	
	for (size_t i = 0; i < resampleLength; ++i)
	{
		// target smoothing line segment
		double target = i * step;
		
		// increment smoothing line segment until you hit target smoothing length
		while (segment + 1 < arcLengths.size() && arcLengths[segment + 1] < target) ++segment;
		
		// once you hit target segment length
		double d0 = arcLengths[segment];
		double d1 = arcLengths[segment + 1];
		double alpha = (target - d0) / (d1 - d0);
		
		/* interpolate */
		const Point2& p0 = origPoly[segment % polyLen];
		const Point2& p1 = origPoly[(segment + 1) % polyLen];
		
		double x = (1.0 - alpha) * p0.x() + alpha * p1.x();
		double y = (1.0 - alpha) * p0.y() + alpha * p1.y();
		resampleLine.emplace_back(x, y); 
	}
	
}	

/* MOVING AVERAGE SMOOTHER */
void RacingLinePlanner::SmoothLine(std::vector<Point2>& smoothLine, const std::vector<Point2>& line, int window)
{
	smoothLine.clear();
	size_t length = line.size();
	
	smoothLine.reserve(length);
	
	for (size_t i = 0; i < length; ++i)
	{
		double smoothX = 0.0, smoothY = 0.0;
		int neighborCount = 0;
		
		for (int j = -window; j <= window; ++j)
		{
			int index = (i + j + length) % length;
			smoothX += line[index].x();
			smoothY += line[index].y();
			++neighborCount;
		}
		
		// take average
		smoothLine.emplace_back(smoothX / neighborCount, smoothY / neighborCount);
		
	} 
}


