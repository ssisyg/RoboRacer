#ifndef GEOMETRY_PATH_H
#define GEOMETRY_PATH_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <CGAL/Polygon_2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <memory>
#include <set>
#include <map>
#include <vector>
#include <boost/shared_ptr.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Triangulation_vertex_base_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Triangulation_data_structure_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_face_base_2.h>
#include <CGAL/Straight_skeleton_2.h>
#include <CGAL/create_straight_skeleton_2.h>
#include <CGAL/create_offset_polygons_2.h>



namespace initial 
{

struct FaceInfo
{
	int nest = 0;
	bool in_domain() const
	{
		return nest % 2 == 1;
	}
};

/* TYPEDEFS */
using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point2 = Kernel::Point_2;
using Poly2 = CGAL::Polygon_2<Kernel>;

//using SCGAL_Ptr = boost::shared_ptr<CGAL::Straight_skeleton_2<Cartes> >;

/* Delaunay Typedefs */
using Vb = CGAL::Triangulation_vertex_base_2<Kernel>;
using Fb_info = CGAL::Triangulation_face_base_with_info_2<FaceInfo, Kernel>;
using Fb = CGAL::Constrained_triangulation_face_base_2<Kernel, Fb_info>;
using Tds = CGAL::Triangulation_data_structure_2<Vb, Fb>;
using CDT = CGAL::Constrained_Delaunay_triangulation_2<Kernel, Tds>;
using SsPtr = boost::shared_ptr<CGAL::Straight_skeleton_2<Kernel> >;

class GeometryPath
{
public:
	/* CONSTRUCTOR (sub and pub) */
	GeometryPath(ros::NodeHandle& nh);

private:
	/* SUB AND PUB */
	ros::Subscriber mapSub;
	ros::Publisher pathPub, path2Pub;
	
	/* padding for robot geometry width */
	double margin;

	void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	
	bool FindContours(std::vector<std::vector<cv::Point> >& outerContours, std::vector<std::vector<cv::Point> >& innerContours, const cv::Mat& binMap, int& outerMaxContourIndex, int& innerMaxContourIndex);
	
	bool BuildCGALPoly(Poly2& outer, Poly2& hole, const nav_msgs::OccupancyGrid::ConstPtr& msg, const std::vector<std::vector<cv::Point> >& outerContours, const std::vector<std::vector<cv::Point> >& innerContours, int outerMaxContourIndex, int innerMaxContourIndex);
	
	void BuildCenterline(std::vector<Point2>& path, std::vector<Point2>& path2, const Poly2& outerPoly, Poly2& holes, int innerMaxContourIndex);
};

} // namespace initial

#endif // GEOMETRY_PATH_H


