
#ifndef REF_LOC_NODE_INCLUDE_SCAN_DATA_H
#define REF_LOC_NODE_INCLUDE_SCAN_DATA_H

#include <deque>
#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>

namespace location {
// 扫描数据
struct RawScanData {
  RawScanData() : angle(0.0), range(0.0), intensity(0.0) {}
  RawScanData(double a, double r, double i)
      : angle(a), range(r), intensity(i) {}
  RawScanData(double a, double r) : angle(a), range(r), intensity(0) {}

  double angle;
  double range;
  double intensity;
};

class MatchedReflector : public RawScanData {
 public:
  MatchedReflector() {}
  MatchedReflector(double x, double y) : x_(x), y_(y) {}
  MatchedReflector(double x, double y, double mx, double my)
      : x_(x), y_(y), mx_(mx), my_(my) {}
  // MatchedReflector(double x, double y, double a, double l) : x_(x), y_(y) {
  //   this->angle = a;
  //   this->range = l;
  // }
  void setx(double x) { x_ = x; }
  void sety(double y) { y_ = y; }
  void setmx(double x) { mx_ = x; }
  void setmy(double y) { my_ = y; }

  const double x() { return x_; }
  const double y() { return y_; }
  const double mx() { return mx_; }
  const double my() { return my_; }

 private:
  double x_;
  double y_;
  double mx_;
  double my_;
};

typedef std::vector<RawScanData> ScanData;
// typedef std::vector<RawScanData> ReflectorScan;

class Point {
 public:
  Point() : x_(0.), y_(0.), th_(0.) {}
  Point(double x, double y) : x_(x), y_(y), th_(0.) {}
  bool operator==(Point point) {
    return point.x() == this->x_ && point.y() == this->y_;
  }
  void setX(double x) { x_ = x; }
  void setY(double y) { y_ = y; }
  void setPoint(double x, double y) {
    x_ = x;
    y_ = y;
  }
  void setPoint(double x, double y, double th) {
    x_ = x;
    y_ = y;
    th_ = th;
  }
  void setTh(double th) { th_ = th; }
  std::string toString() {
    std::string str = "(" + std::to_string(x_) + "," + std::to_string(y_) +
                      "," + std::to_string(th_);
    return str;
  }

  double DistanceTo(Point pt) {
    return std::hypot(this->x() - pt.x(), this->y() - pt.y());
  }

  double x() { return x_; }
  double y() { return y_; }
  double th() { return th_; }

 private:
  double x_;
  double y_;
  double th_;
};

/*
 // 点-二维向量
class Point : public Eigen::Vector2d {
 public:
  Point() : Eigen::Vector2d() {}

  Point(double x, double y) : Eigen::Vector2d() { *this << x, y; }

  bool operator==(const Point &p) {
    return p(0) == (*this)[0] && p(1) == (*this)[1];
  }

  double get_x() { return (*this)[0]; }
  double get_y() { return (*this)[1]; }
  void set_value(double x_, double y_) { *this << x_, y_; }
};
*/

// 位姿-三维向量
class Pose : public Eigen::Vector3d {
 public:
  Pose() : Eigen::Vector3d() {}

  Pose(double x, double y, double theta) : Eigen::Vector3d() {
    *this << x, y, theta;
  }

  double x() { return (*this)[0]; }
  double y() { return (*this)[1]; }
  double theta() { return (*this)[2]; }
};

class PointSet : public std::vector<Point> {
 public:
  // // 默认构造函数
  // PointSet() = default;

  // // 拷贝构造函数
  // PointSet(const PointSet& other)
  //     : std::vector<Point>(other),  // 拷贝父类部分
  //       dist_m(other.dist_m)        // 拷贝 Eigen 矩阵
  // {}

  // // 拷贝赋值运算符
  // PointSet& operator=(const PointSet& other) {
  //     if (this != &other) {
  //         std::vector<Point>::operator=(other); // 调用父类赋值
  //         dist_m = other.dist_m;                // 赋值 Eigen 矩阵
  //     }
  //     return *this;
  // }

  // // 移动构造函数
  // PointSet(PointSet&& other) noexcept
  //     : std::vector<Point>(std::move(other)),
  //       dist_m(std::move(other.dist_m))
  // {}

  // // 移动赋值运算符
  // PointSet& operator=(PointSet&& other) noexcept {
  //     if (this != &other) {
  //         std::vector<Point>::operator=(std::move(other));
  //         dist_m = std::move(other.dist_m);
  //     }
  //     return *this;
  // }

  // // 析构函数
  // ~PointSet() = default; // Eigen 矩阵会自动释放内存

 
  // 生成边矩阵
  void GeneralEdgeMatrix() {
    if (this->empty()) return;
    // std::cout << "PointSet size: " << size() << std::endl;
    dist_m.resize(this->size(), this->size());
    for (uint32_t i = 0; i < this->size(); i++) {
      for (uint32_t j = 0; j < this->size(); j++) {
        if (i == j) {
          dist_m(i, j) = 0;
        } else if (i < j) {
          dist_m(i, j) = std::hypot(this->at(i).x() - this->at(j).x(),
                                    this->at(i).y() - this->at(j).y());
        } else {
          dist_m(i, j) = dist_m(j, i);
        }
      }
    }
  }
  // 找到最大边长度的两个点
  PointSet GetPointsWithMaxEdge() {
    PointSet pts;
    if (dist_m.size() == 0) return pts;

    Eigen::Index max_row, max_col;
    dist_m.maxCoeff(&max_row, &max_col);
    pts.push_back(this->at(static_cast<size_t>(max_row)));
    pts.push_back(this->at(static_cast<size_t>(max_col)));
    return pts;
  }

  // 根据距离查找匹配的两个点
  std::vector<PointSet> GetCandidateWithEdge(double distance) {
    std::vector<PointSet> candidates;
    for (Eigen::Index i = 0; i < dist_m.rows(); i++) {
      for (Eigen::Index j = 0; j < dist_m.cols(); j++) {
        if (std::fabs(dist_m(i, j) - distance) < 0.08) {
          PointSet candidate;
          candidate.push_back(this->at(static_cast<size_t>(i)));
          candidate.push_back(this->at(static_cast<size_t>(j)));
          candidates.push_back(candidate);
        }
      }
    }
    // ROS_INFO_STREAM("candidates size:" << candidates.size());
    return candidates;
  }

  // // 是否匹配
  // bool AddpointCorrespond(PointSet pts, const Point &pt) {
  //   pts.push_back(pt);
  //   for (uint32_t i = 0; i < this->size() - 1; i++) {
  //     auto distPts =
  //         std::hypot(pts.back().x() - pts[i].x(), pts.back().y() - pts[i].y());
  //     auto distThis = std::hypot((*this).back().x() - (*this)[i].x(),
  //                                (*this).back().y() - (*this)[i].y());
  //     if (std::abs(distPts - distThis) >= 0.25) {
  //       return false;
  //     }
  //   }
  //   return true;
  // }

  // 是否匹配
  bool AddpointCorrespond(PointSet pts, const Point &pt) {
    double dist_threshold = 0.25;// 0.1
    pts.push_back(pt);
    for (uint32_t i = 0; i < this->size() - 1; i++) {
      auto distPts = std::hypot(pts.back().x() - pts[i].x(), pts.back().y() - pts[i].y());
      auto distThis = std::hypot((*this).back().x() - (*this)[i].x(), (*this).back().y() - (*this)[i].y());
      if (std::abs(distPts - distThis) >= dist_threshold) {
        if (i == 0 && this->size() > 2) {
          auto distThis1 = std::hypot((*this).back().x() - (*this)[1].x(), (*this).back().y() - (*this)[1].y());
          if (std::abs(distPts - distThis1) >= dist_threshold){
            return false;
          } else {
            // swap p0 and p1: [p0, p1] and [p1, p0] are same edge, sharing the same max distance
            Point pt0 = (*this)[0];
            (*this)[0] = (*this)[1];
            (*this)[1] = pt0;

            continue;
          }
        }
      }
    }
    return true;
  }

  bool ContainPoint(const Point &pt) {
    for (auto element : *this) {
      if (element == pt) return true;
    }
    return false;
  }

  Eigen::MatrixXd Dist_m() { return dist_m; }
  double MaxDistance() { return dist_m.maxCoeff(); }

  std::vector<PointSet> choose(int k) {
    std::vector<int> input(k);
    std::vector<std::vector<int>> output;
    Cij(this->size(), k, &input, k, &output);

    std::vector<PointSet> ptSetList;
    for (auto i : output) {
      PointSet ptSet;
      for (auto j : i) {
        ptSet.push_back(this->at(j));
      }
      ptSetList.push_back(ptSet);
    }
    return ptSetList;
  }

  // 拉普拉斯展开 余子式
  void Cij(int i, int j, std::vector<int> *r, int num,
           std::vector<std::vector<int>> *result) {
    if (j == 1) {
      for (int a = 0; a < i; a++) {
        std::vector<int> temp(num);
        (*r)[num - 1] = a;
        for (int b = 0; b < num; b++) {
          temp[b] = r->at(b);
        }
        result->push_back(temp);
      }
    } else if (j == 0) {
      ;
    } else {
      for (int a = i; a >= j; a--) {
        (*r)[j - 2] = a - 1;
        Cij(a - 1, j - 1, r, num, result);
      }
    }
  }

  PointSet PointsFilter(double threshold) {
    PointSet tmpts;
    tmpts.insert(tmpts.begin(), this->begin(), this->end());
    PointSet result;
    while (!tmpts.empty()) {
      if (result.empty()) {
        result.push_back(tmpts.front());
        if(!tmpts.empty()) tmpts.erase(tmpts.begin());
      }
      if (std::hypot(result.back().x() - tmpts.front().x(),
                     result.back().y() - tmpts.front().y()) > threshold) {
        result.push_back(tmpts.front());
        if(!tmpts.empty()) tmpts.erase(tmpts.begin());
      } else {
        if(!tmpts.empty()) tmpts.erase(tmpts.begin());
      }
      if (!tmpts.empty()) continue;
    }
    tmpts.clear();
    return result;
  }

 private:
  // distance matrix
  Eigen::MatrixXd dist_m;
};

class MatchedReflectorList : public std::vector<MatchedReflector> {
 public:
  PointSet ToPointSet() {
    PointSet tmp_list;
    if (this->empty()) return tmp_list;
    for (auto ref : *this) {
      Point pt(ref.x(), ref.y());
      tmp_list.emplace_back(pt);
    }
    return tmp_list;
  }
};
class ReflectorScan : public std::vector<RawScanData> {
 public:
  PointSet ToPointSet() {
    PointSet tmp_list;
    if (this->empty()) return tmp_list;
    for (auto ref : *this) {
      Point pt(ref.range * std::cos(ref.angle),
               ref.range * std::sin(ref.angle));
      tmp_list.emplace_back(pt);
    }
    return tmp_list;
  }
};

}  // namespace location

#endif  // REF_LOC_NODE_INCLUDE_SCAN_DATA_H