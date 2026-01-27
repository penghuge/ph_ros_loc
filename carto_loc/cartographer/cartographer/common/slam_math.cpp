#include "slam_math.h"

namespace SlamCommon
{
/**********************************************************************************
function: Point to line distance without direction
    params: pt_x / pt_y => point
            (start_x, start_y) => start point of line
            (end_x, end_y) => end point of line
            to_stright_line_distance:
                true => point to straight line distance, the length of line is unlimited;
                false => point to segment line distance, if point is beyond endpoint of segment,
                    the distance from point to line is the distance from point to endpoint of segment
    return: distance from point to line, which is always positive
**********************************************************************************/
double PointToLineDistance(const double pt_x, const double pt_y, const double start_x, const double start_y,
    const double end_x, const double end_y, const bool to_stright_line_distance)
{
    double A = pt_x - start_x;
    double B = pt_y - start_y;
    double C = end_x - start_x;
    double D = end_y - start_y;
	double param = (A * C + B * D) / (C * C + D * D);

	double line_pt_x, line_pt_y;

	if (to_stright_line_distance)
	{
        line_pt_x = start_x + param * C;
        line_pt_y = start_y + param * D;
	} 
	else
	{
		if (param < 0)
		{
            line_pt_x = start_x;
            line_pt_y = start_y;
		}
		else if (param > 1)
		{
            line_pt_x = end_x;
            line_pt_y = end_y;
		}
		else
		{
            line_pt_x = start_x + param * C;
            line_pt_y = start_y + param * D;
		}
	}

    return Hypot(line_pt_x - pt_x, line_pt_y - pt_y);
}

/**********************************************************************************
function: subpoint of point p to line ,
    params: p1 and p1 are point on line
            p is point
    return: subpoint
**********************************************************************************/

Point2D PointToLineSubpoint(const Point2D &p, const Point2D &p1,const Point2D &p2)
{
    Point2D subpoint;
    double l = ((p.x - p1.x) * (p2.x - p1.x)
             + (p.y - p1.y) * (p2.y - p1.y))
    / ((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
    subpoint.x = l * (p2.x - p1.x) + p1.x;
    subpoint.y = l * (p2.y - p1.y) + p1.y;

    return subpoint;
}

/**********************************************************************************
function: Point to line distance with direction, 
    params: pt_x / pt_y => point
            (start_x, start_y) => start point of line
            (end_x, end_y) => end point of line
            to_stright_line_distance:
                true => point to straight line distance, the length of line is unlimited;
                false => point to segment line distance, if point is beyond endpoint of segment,
                    the distance from point to line is the distance from point to endpoint of segment
    return: distance from point to line,
                distance is positive while point is on the left side of line,
                distance is negative while point is on the right side of line
**********************************************************************************/
double PointToLineVectorDistance(const double pt_x, const double pt_y, const double start_x,
    const double start_y, const double end_x, const double end_y, const bool to_stright_line_distance)
{
	double A = pt_x - start_x;
	double B = pt_y - start_y;
	double C = end_x - start_x;
	double D = end_y - start_y;

	double dot = A * C + B * D;
	double len_sq = C * C + D * D;
	double param = dot / len_sq;

	double line_pt_x, line_pt_y;

	if (to_stright_line_distance)
	{
		line_pt_x = start_x + param * C;
		line_pt_y = start_y + param * D;
	} 
	else
	{
		if (param < 0)
		{
			line_pt_x = start_x;
			line_pt_y = start_y;
		}
		else if (param > 1)
		{
			line_pt_x = end_x;
			line_pt_y = end_y;
		}
		else
		{
			line_pt_x = start_x + param * C;
			line_pt_y = start_y + param * D;
		}
	}

	double dist = Hypot(line_pt_x - pt_x, line_pt_y - pt_y);
	if (CheckPointOnTheRightOfLine(pt_x, pt_y, start_x, start_y, end_x, end_y))
	{
		dist *= -1.;
	}
	return dist;
}

/**********************************************************************************
function: Point is in the rectangle or not
    params: pt => point
            (rect_p1, rect_p2, rect_p3, rect_p4) => four endpoint of rectangle
    return: whether is in rectangle or not
                in rectangle or at the side of rectangle, return true
                out of rectangle, return false
**********************************************************************************/
bool IsPointInRectangle(const Pose3D &pt, const Pose3D &rect_p1, const Pose3D &rect_p2,
                     const Pose3D &rect_p3, const Pose3D &rect_p4)
{
    std::map<double, Pose3D> pose_map;
    double dist = std::fabs(rect_p1.x - rect_p2.x) + std::fabs(rect_p1.y - rect_p2.y);
    pose_map[dist] = rect_p2;
    dist = std::fabs(rect_p1.x - rect_p3.x) + std::fabs(rect_p1.y - rect_p3.y);
    pose_map[dist] = rect_p3;
    dist = std::fabs(rect_p1.x - rect_p4.x) + std::fabs(rect_p1.y - rect_p4.y);
    pose_map[dist] = rect_p4;

    std::map<double, Pose3D>::iterator iter = pose_map.begin();
    for (size_t i = 0; i < 2; ++i)
    {
        if (!CheckPointAmongOtherTwoPoint(pt, rect_p1, iter->second))
        {
            return false;
        }
        std::advance(iter, 1);
    }
    return true;
}

/**********************************************************************************
function: Point is in the polygon or not
    in polygon or at the side of polygon, return true
    out of polygon, return false
**********************************************************************************/
bool isPointInPolygon(const SlamCommon::Pose3D &p,
                      const std::vector<SlamCommon::Pose3D> &end_pts)
{
    int vertices_cnt = end_pts.size();
    bool is_intersect_on_section = false;
    int cross = 0;
    for (int i = 0; i < vertices_cnt; ++i)
    {
        SlamCommon::Pose3D p1 = end_pts[i];
        SlamCommon::Pose3D p2 = end_pts[(i + 1) % vertices_cnt];

        if (p1.y == p2.y)
        {
            continue;
        }
        if (p.y < std::min(p1.y, p2.y))
        {
            continue;
        }
        if (p.y > std::max(p1.y, p2.y))
        {
            continue;
        }
        double x = (p.y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;

        if (fabs(x - p.x) < 10e-4)
        {
            is_intersect_on_section = true;
            break;
        }
        if (x > p.x)
        {
            cross++;
        }
    }
    if (is_intersect_on_section || cross % 2 == 1)
    {
        return true;
    }
    return false;
}

/**********************************************************************************
function: search object in vector by dichotomy method
**********************************************************************************/
int SearchByDichotomy(const std::vector<int> &que, const int target)
{
    if (que.empty())
    {
        return -1;
    }
    int start_index = 0;
    int end_index = (int)que.size() - 1;
    int mid_index = (int)(0.5 * end_index);
    while (true)
    {
        if (start_index >= end_index)
        {
            if (target == que[start_index])
            {
                break;
            }
            else
            {
                return -2;
            }
        }
        if (target > que[mid_index])
        {
            start_index = mid_index + 1;
            mid_index = (int)(start_index + 0.5 * (end_index - start_index));
            continue;
        }
        else if (target < que[mid_index])
        {
            end_index = mid_index - 1;
            mid_index = (int)(start_index + 0.5 * (end_index - start_index));
            continue;
        }
        else
        {
            break;
        }
    }
    return mid_index;
}

} // namespace SlamCommon
