#ifndef SLAM_MATH_H_
#define SLAM_MATH_H_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>

#include "const_value.h"

namespace SlamCommon
{
	double PointToLineDistance(const double pt_x, const double pt_y, const double start_x, const double start_y,
		const double end_x, const double end_y, const bool to_stright_line_distance);
    Point2D PointToLineSubpoint(const Point2D &p,
       const Point2D &p1,const Point2D &p2);
    double PointToLineVectorDistance(const double pt_x, const double pt_y,
                                     const double start_x, const double start_y,
                                     const double end_x, const double end_y,
                                     const bool to_stright_line_distance);

    bool IsPointInRectangle(const Pose3D &pt, const Pose3D &rect_p1, const Pose3D &rect_p2,
                         const Pose3D &rect_p3, const Pose3D &rect_p4);
    bool isPointInPolygon(const SlamCommon::Pose3D &p,
                          const std::vector<SlamCommon::Pose3D> &end_pts);

    inline double LawOfCosinesForSide(const double side1, const double side2,
                                      const double angle_diff)
	{
		return std::sqrt(side1 * side1 + side2 * side2
			- 2 * side1 * side2 * std::cos(angle_diff));
	}
    inline double LawOfSinesForAngle(const double side1, const double angle1,
                                     const double side2)
    {
        if(0 == side1)
        {
            return M_PI;
        }
        return side2 * sin(angle1) / side1;
    }
    inline double LawOfSinesForSide(const double side1, const double angle1,
                                    const double angle2)
    {
        if(0 == angle1)
        {
            return 0;
        }
        return side1  * sin(angle2) / sin(angle1);
    }
    inline void PointProjectOnLine(const double pt_x, const double pt_y,
                                   const double start_x, const double start_y,
                                   const double end_x, const double end_y,
                                   double &line_pt_x, double &line_pt_y)
	{
		double A = pt_x - start_x;
		double B = pt_y - start_y;
		double C = end_x - start_x;
		double D = end_y - start_y;
		double param = (A * C + B * D) / (C * C + D * D);

		line_pt_x = start_x + param * C;
		line_pt_y = start_y + param * D;

	}
	/**********************************************************************************
	function: using vector cross product to judge the position relationship of point and line,
		if point on the right of line, return true
	**********************************************************************************/
    inline bool CheckPointOnTheRightOfLine(const double pt_x, const double pt_y,
                                           const double start_x, const double start_y,
                                           const double end_x, const double end_y)
	{
		double x1 = start_x - pt_x;
		double y1 = start_y - pt_y;
		double x2 = end_x - start_x;
		double y2 = end_y - start_y;
		double value = x1 * y2 - x2 * y1;
		return value <= 0 ? true : false;
	}

    //将角度归一化到（-pi, pi]
//	inline double NormalizeAngle(const double z)
//	{
//		return atan2(sin(z), cos(z));
//	}
    template <typename T>
    T NormalizeAngle(T z)
    {
        const T kPi = T(M_PI);
        while (z > kPi) z -= 2. * kPi;
        while (z < -kPi) z += 2. * kPi;
        return z;
    }

    //求两条射线的夹角
    inline double AngleDiff(const double dest, const double from)
    {
		double dest_angle = NormalizeAngle(dest);
		double from_angle = NormalizeAngle(from);
        double d1 = dest_angle - from_angle;
        double d2 = 2 * M_PI - fabs(d1);
		if (d1 > 0)
		{
			d2 *= -1.0;
		}
		if (fabs(d1) < fabs(d2))
		{
			return(d1);
		}
		else
		{
			return(d2);
		}
	}

    //求两条直线的最小夹角
    inline double AngleMinDiff(const double dest, const double from)
    {
        double d1(dest), d2(from);
        if(d1 > M_PI_2)
        {
            d1 -= M_PI;
        }
        else if(d1 < -M_PI_2)
        {
            d1 += M_PI;
        }

        if(d2 > M_PI_2)
        {
            d2 -= M_PI;
        }
        else if(d2 < -M_PI_2)
        {
            d2 += M_PI;
        }

        return fabs(d2 - d1);
    }

	/**********************************************************************************
	function: check whether point is beyond the end_pt
		beyond the end_pt, return true
        in or at the endpoint, return false
	**********************************************************************************/
	inline bool CheckPointBeyondEndPoint(const Pose3D &pt, const Pose3D &start_pt,
		const Pose3D &end_pt)
	{
		Pose3D vector1 = start_pt - end_pt;
		Pose3D vector2 = pt - end_pt;
		return (vector1.x * vector2.x + vector1.y * vector2.y) < 0 ? true : false;
	}

	/**********************************************************************************
	function: check whether point is among the other two point
        among the other two points or overlapping at the other two point, return true
	**********************************************************************************/
	inline bool CheckPointAmongOtherTwoPoint(const Pose3D &pt, const Pose3D &start_pt,
		const Pose3D &end_pt)
	{
		if (CheckPointBeyondEndPoint(pt, start_pt, end_pt))
		{
			return false;
		}
		if (CheckPointBeyondEndPoint(pt, end_pt, start_pt))
		{
			return false;
		}
		return true;
	}

	template <typename T>
	T Hypot(T a, T b)
	{
		return std::sqrt(a * a + b * b);
	}

	template <typename T>
	T Sign(T val)
	{
		return val >= 0 ? 1 : -1;
    }

    int SearchByDichotomy(const std::vector<int> &que, const int target);

    inline int ManhattanDistance(const int x1, const int y1, const int x2, const int y2)
    {
        return abs(x1 - x2) + abs(y1 - y2);
    }

    inline double ManhattanDistance(const double x1, const double y1,
                                    const double x2, const double y2)
    {
        return fabs(x1 - x2) + fabs(y1 - y2);
    }

    inline float ManhattanDistance(const float x1, const float y1,
                                   const float x2, const float y2)
    {
        return fabsf(x1 - x2) + fabsf(y1 - y2);
    }

    inline double ComputeDistance(const Pose3D &pose1, const Pose3D &pose2)
    {
        return std::hypot(pose1.x - pose2.x, pose1.y - pose2.y);
    }

} // namespace SlamCommon

#endif // !SLAM_MATH_H_
