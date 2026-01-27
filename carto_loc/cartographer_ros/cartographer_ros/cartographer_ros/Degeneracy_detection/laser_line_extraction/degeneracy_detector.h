// degeneracy_detector.h (embedded copy for cartographer_ros integration)
#pragma once
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include "laser_line_extraction/line.h"
#include "laser_line_extraction/line_extraction.h"

namespace laser_line_extraction {

class DegeneracyDetector {
public:
  struct Params {
    double long_line_ratio = 0.6;
    double min_long_line_length = 1.0;
    double angle_consistency_thresh = 0.1;
    double coverage_ratio = 0.6;
  };

  DegeneracyDetector();
  explicit DegeneracyDetector(const Params &p);
  bool detect(const sensor_msgs::LaserScan &scan);
  const std::vector<line_extraction::Line>& getLines() const { return lines_; }

  struct SimpleLine {
    double start[2];
    double end[2];
    double angle = 0.0;
    double length = 0.0;
  };
  const std::vector<SimpleLine>& getProcessedLines() const { return processed_lines_; }

  void setParams(const Params &p) { params_ = p; }
  void SetLidarParam(float angle_max, float angle_min, float angle_increment);
  void SetLineExtractionFirstParam(double bearing_std_dev, double range_std_dev,
                                   double least_sq_angle_thresh,
                                   double least_sq_radius_thresh,
                                   double max_line_gap, double min_line_length);
  void SetLineExtractionSecondParam(int min_line_points, double min_range,
                                    double max_range, double min_split_dist,
                                    double outlier_dist);

private:
  Params params_;
  line_extraction::LineExtraction extractor_;
  std::vector<line_extraction::Line> lines_;
  std::vector<SimpleLine> processed_lines_;
  double computeTotalLineLength();
  bool checkSingleLongLine();
  bool checkAngleConsistency();
};

} // namespace laser_line_extraction
