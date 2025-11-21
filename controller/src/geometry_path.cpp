#include <ros/console.h>
#include <algorithm> 
#include <CGAL/centroid.h>
#include <geometry_msgs/PoseStamped.h>
#include "initial/geometry_path.h"

using namespace initial;

constexpr double MIN_AREA_THRESH = 1.0;


GeometryPath::GeometryPath(ros::NodeHandle& nh)
{
	ros::NodeHandle privateNodeHandle("~");
	/* padding parameter, default 0.0*/
	privateNodeHandle.param("car_width", margin, 0.0);
	
	mapSub = nh.subscribe("map", 1, &GeometryPath::MapCallback, this);
	pathPub = nh.advertise<nav_msgs::Path>("outer_geometry_path", 1, true);
	path2Pub = nh.advertise<nav_msgs::Path>("inner_geometry_path", 1, true);
}

void GeometryPath::MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	/* Create a binary map */
	int w = msg->info.width, h = msg->info.height;
	if (w == 0 || h == 0 || msg->data.empty())
	{
		ROS_INFO("NO DATA FROM OCCUPANCY_GRID! (w: %d, h: %d)", w, h);
		return;
	}
	ROS_INFO("MAPCALLBACK: %d, %d, res %f", w, h, msg->info.resolution);
	cv::Mat binMap = cv::Mat::zeros(h, w, CV_8UC1);
	
	if (!binMap.data)
	{
		ROS_INFO ("CV BIN_MAP FAILED!");
		return;
	}
	for (int y = 0; y < h; ++y) {
	    for (int x = 0; x < w; ++x) {
		int idx = y * w + x;
		binMap.at<uchar>(y,x) = (msg->data[idx] == 0 ? 255 : 0);
	    }
	}
	
	/* inflate obstacles */
	cv::Mat inflatedMask = (binMap == 0);
	cv::Mat inflatedMap;
	double inflate = 0.18;
	double res = msg->info.resolution;
	int inflatePx = static_cast<int>(inflate / res + 0.5);
	
	cv::Mat k = cv::getStructuringElement(cv::MORPH_ELLIPSE, 
							cv::Size(2 * inflatePx + 1, 2 * inflatePx + 1),
							cv::Point(inflatePx, inflatePx)
							);
	cv::dilate(inflatedMask, inflatedMap, k);
	binMap.setTo(0, inflatedMap);
	
	/* Find largest contour */
	std::vector<std::vector<cv::Point> > outerContours, innerContours;
	int innerMaxContourIndex, outerMaxContourIndex = -1;
	
	if (!FindContours(outerContours, innerContours, binMap, outerMaxContourIndex, innerMaxContourIndex)) return;
	
	/* Build polygon from contour */
	Poly2 outerPoly, hole;
	
	if (!BuildCGALPoly(outerPoly, hole, msg, outerContours, innerContours, outerMaxContourIndex, innerMaxContourIndex)) 
		return;
		
	ROS_INFO("OUTER POLY: %zu vertices. First: (%.2f, %.2f)", outerPoly.size(), outerPoly[0].x(), outerPoly[0].y());
	ROS_INFO("INNER POLY: %zu vertices, First: (%.2f, %.2f)", hole.size(), hole[0].x(), hole[0].y());
	/* Generate path */
	std::vector<Point2> outerLinePoints, innerLinePoints;
	
	BuildCenterline(outerLinePoints, innerLinePoints, outerPoly, hole, innerMaxContourIndex);
	if (outerLinePoints.empty())
	{
		ROS_INFO("outer line no path main");
		return;
	}
	else if(innerLinePoints.empty())
	{
		ROS_INFO("inner line no path main");
		return;
	}
	
	/* Publish Path msg */
	nav_msgs::Path outerPath, innerPath;
	outerPath.header.stamp = ros::Time::now();
	outerPath.header.frame_id = innerPath.header.frame_id = "map";
	innerPath.header.stamp = ros::Time::now();
	
	for (auto &p : outerLinePoints)
	{
		geometry_msgs::PoseStamped poseStamped;
		poseStamped.header = outerPath.header;
		poseStamped.pose.position.x = p.x();
		poseStamped.pose.position.y = p.y();
		poseStamped.pose.position.z = 0.0;
		
		poseStamped.pose.orientation.x = 0.0;
		poseStamped.pose.orientation.y = 0.0;
		poseStamped.pose.orientation.z = 0.0;
		poseStamped.pose.orientation.w = 1.0;
		outerPath.poses.push_back(poseStamped);
	}
	
	for (auto &p : innerLinePoints)
	{
		geometry_msgs::PoseStamped poseStamped;
		poseStamped.header = innerPath.header;
		poseStamped.pose.position.x = p.x();
		poseStamped.pose.position.y = p.y();
		poseStamped.pose.position.z = 0.0;
		
		poseStamped.pose.orientation.x = 0.0;
		poseStamped.pose.orientation.y = 0.0;
		poseStamped.pose.orientation.z = 0.0;
		poseStamped.pose.orientation.w = 1.0;
		innerPath.poses.push_back(poseStamped);
	}
	
	pathPub.publish(outerPath);
	path2Pub.publish(innerPath);
}

bool GeometryPath::FindContours(std::vector<std::vector<cv::Point> >& outerContours, std::vector<std::vector<cv::Point> >& innerContours, const cv::Mat& binMap, int& outerMaxContourIndex, int& innerMaxContourIndex)
{
	std::vector<std::vector<cv::Point> > allContours;
	std::vector<cv::Vec4i> hierarchy;
	
	if (binMap.empty())
	{
		ROS_INFO("empty bin map in maxContour");
		return false;
	}
	
	/* Extract CONTOURS */
	cv::findContours(binMap, allContours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
	if (allContours.empty()) 
	{
		ROS_INFO ("NO CONTOURS!"); 
		return false;
	}
	
	outerContours.clear();
	innerContours.clear();
	
	/* CLASSIFY OUTER CONTOURS */
	double outerMaxArea, innerMaxArea;
	int maxOuterIdx, maxInnerIdx, outerIdx, innerIdx;
	int size = static_cast<int>(allContours.size());
	
	maxOuterIdx = maxInnerIdx = innerIdx = outerIdx = -1;
	outerMaxArea = innerMaxArea = 0.0;
	for (int i = 0; i < size; ++i)
	{
		int h = hierarchy[i][3];
		// != -1 -> holes (inner walls)
		if (h != -1) 
		{
			double tmpInnerArea = std::fabs(cv::contourArea(allContours[i]));
			innerContours.push_back(allContours[i]);
			++innerIdx;
			
			if (tmpInnerArea > innerMaxArea) 
			{
				innerMaxArea = tmpInnerArea;
				maxInnerIdx = innerIdx; 
			}
			
			continue;
		}
		
		else
		{
			double tmpOuterArea = std::fabs(cv::contourArea(allContours[i]));
			outerContours.push_back(allContours[i]);
			++outerIdx;
			
			if (tmpOuterArea > outerMaxArea)
			{
				outerMaxArea = tmpOuterArea;
				maxOuterIdx = outerIdx;
			}
		}
		
	}
	
	// Inner contour or outer contour failed. 
	if (innerIdx < 0 || outerIdx < 0) 
	{
		ROS_INFO("ILLEGAL CONTOURS: INNER: %d, OUTER: %d", innerIdx, outerIdx);
		return false;
	}
	
	outerMaxContourIndex = maxOuterIdx;
	innerMaxContourIndex = maxInnerIdx;
	return true;
}
	
bool GeometryPath::BuildCGALPoly(Poly2& outer, Poly2& hole, const nav_msgs::OccupancyGrid::ConstPtr& msg, 
	const std::vector<std::vector<cv::Point> >& outerContours, const std::vector<std::vector<cv::Point> >& innerContours, 
	int outerMaxContourIndex, int innerMaxContourIndex)
{
	double res = msg->info.resolution;
	double origX = msg->info.origin.position.x;
	double origY = msg->info.origin.position.y;
	
	outer.clear();
	hole.clear();
	
	int contourIndex = 0;
	
	/* Draw outer wall polygon */
	const auto& outerWallContour = outerContours[outerMaxContourIndex];
	Point2 prevPoint;
	bool firstIter = true;
	 	
	/* build polygon out of cv contour */
	for (const auto& point : outerWallContour)
	{
		double toWorldX = origX + (point.x + 0.5) * res;
		double toWorldY = origY + (point.y + 0.5) * res;
		Point2 currPoint(toWorldX, toWorldY);
		
		// only add non-duplicates
		if (firstIter || currPoint != prevPoint)
			outer.push_back(currPoint);
			
		prevPoint = currPoint;
		firstIter = false;
	}
	
	if (!CGAL::is_simple_2(outer.vertices_begin(), outer.vertices_end()))
	{
		ROS_INFO("OUTER WALL: Not simple poly!");
		return false;
	}
	
	if (!outer.is_counterclockwise_oriented())
	{
		outer.reverse_orientation();
	}
	
	/* INNER WALLS */
	const auto& innerWallContour = innerContours[innerMaxContourIndex];
	firstIter = true;
	
	for (const auto& point : innerWallContour)
	{
		double toWorldX = origX + (point.x + 0.5) * res;
		double toWorldY = origY + (point.y + 0.5) * res;
		Point2 currPoint(toWorldX, toWorldY);
		
		// only add non-duplicates
		if (firstIter || currPoint != prevPoint)
			hole.push_back(currPoint);
			
		prevPoint = currPoint;
		firstIter = false;
	}
	
	if (!CGAL::is_simple_2(hole.vertices_begin(), hole.vertices_end()))
	{
		ROS_INFO("OUTER WALL: Not simple poly!");
		return false;
	}
	
	if (!hole.is_counterclockwise_oriented())
	{
		hole.reverse_orientation();
	}
	
	return true;
}

void GeometryPath::BuildCenterline(std::vector<Point2>& outerLinePoints, std::vector<Point2>& innerLinePoints, const Poly2& outerPoly, Poly2& hole, int innerMaxContourIndex)
{
	outerLinePoints.clear();
	innerLinePoints.clear();
	
	if (outerPoly.size() < 3 || !CGAL::is_simple_2(outerPoly.vertices_begin(), outerPoly.vertices_end())) 
	{
		ROS_INFO("INVALID OUTER POLY"); 
		return;
	}
	
	
	if (hole.size() < 3 || !CGAL::is_simple_2(hole.vertices_begin(), hole.vertices_end()))
	{
		ROS_INFO("INVALID HOLE");
		return;
	}
	
	
	/* CREATE SKELETONS FOR OUTER LINE AND INNER LINE GEOMETRY */
	SsPtr outerSS = CGAL::create_interior_straight_skeleton_2(outerPoly.vertices_begin(), outerPoly.vertices_end());
	
	SsPtr innerSS = CGAL::create_interior_straight_skeleton_2(hole.vertices_begin(), hole.vertices_end());
	
	
	if (!outerSS || !innerSS) 
	{
		ROS_INFO("NO SKELETON!! FAILED");
		return;
	}
	
	/* EXTRACT OUTER AND INNER LINES */
	std::vector<boost::shared_ptr<Poly2> > outerLines, innerLines;
	
	try
	{
		outerLines = CGAL::create_offset_polygons_2<Poly2>(0.01, *outerSS);
		innerLines = CGAL::create_offset_polygons_2<Poly2>(0.01, *innerSS);
		
	}
	catch (const CGAL::Precondition_exception& e)
	{
		ROS_INFO("precondition error");
		return;
	}
	if (outerLines.empty() || innerLines.empty())
	{
		ROS_INFO("NO VALID OUTER LINES!! FAILED");
		return;
	}
	else if (innerLines.empty())
	{
		ROS_INFO("NO VALID INNER LINES!! FAILED");
		return;
	}
	
	/* COPY RELEVANT DATA */
	auto outerLine = outerLines[0];
	auto innerLine = innerLines[0];
	
	// assuming the first line is the outer line
	for (auto vertex = outerLine->vertices_begin(); vertex != outerLine->vertices_end(); ++vertex)
	{
		outerLinePoints.push_back(*vertex);
	}
	
	for (auto vertex = innerLine->vertices_begin(); vertex != innerLine->vertices_end(); ++vertex)
	{
		innerLinePoints.push_back(*vertex);
	}
	
} 

