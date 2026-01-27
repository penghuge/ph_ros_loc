// degeneracy_detector.cpp (embedded)
#include "laser_line_extraction/degeneracy_detector.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <ros/ros.h>

using namespace laser_line_extraction;

DegeneracyDetector::DegeneracyDetector() : params_()
{
}
DegeneracyDetector::DegeneracyDetector(const Params& p) : params_(p)
{
}

bool DegeneracyDetector::detect(const sensor_msgs::LaserScan& scan)
{
    std::vector<double> ranges(scan.ranges.begin(), scan.ranges.end());
    extractor_.setRangeData(ranges);

    int valid_ranges = 0;
    for (double r : ranges)
        if (std::isfinite(r) && r > 0.0)
            ++valid_ranges;
    ROS_DEBUG_STREAM("[degeneracy_detector] valid_ranges=" << valid_ranges << " total_ranges=" << ranges.size());

    // 规则：有效点数过少（<3）直接判定为退化，避免误检与后续计算
    if (valid_ranges < 3) {
        lines_.clear();  // 避免沿用上一帧的线段
        ROS_WARN_THROTTLE(1.0, "[degeneracy_detector] valid_ranges=%d < 3 -> DEGENERATE", valid_ranges);
        return true;
    }

    lines_.clear();
    extractor_.extractLines(lines_);
    if (lines_.empty())
        return false;

    // 过滤 线段（后处理阈值） 看情况过滤
    const double kMinLen = 0.0;
    std::vector<line_extraction::Line> filtered;
    filtered.reserve(lines_.size());
    for (const auto& l : lines_) if (l.length() >= kMinLen) filtered.push_back(l);

    // 合并角度相近且平行距离<=0.5m 的线段（以主线切向量为基准，端点沿切向量投影直接首尾连接）
    processed_lines_.clear();
    // 记录每条组合直线由哪些原始线段（按 filtered/lines_ 索引）组成
    std::vector<std::vector<size_t>> groups;
    const double ang_thresh = params_.angle_consistency_thresh;
    const double cos_thresh = std::cos(ang_thresh);
    const double parallel_thresh = 0.5;
    std::vector<bool> used(filtered.size(), false);
    auto get_dir = [](const line_extraction::Line& L) {
        const double sx = L.getStart()[0], sy = L.getStart()[1];
        const double ex = L.getEnd()[0],   ey = L.getEnd()[1];
        double vx = ex - sx, vy = ey - sy;
        const double n = std::hypot(vx, vy);
        if (n <= 1e-9) return std::array<double,2>{1.0, 0.0};
        return std::array<double,2>{vx / n, vy / n};
    };
    auto dot2 = [](double ax, double ay, double bx, double by){ return ax*bx + ay*by; };
    auto rot90 = [](double x, double y){ return std::array<double,2>{-y, x}; };

    for (size_t i = 0; i < filtered.size(); ++i) {
        if (used[i]) continue;
        // 选择该组中最长为主线
        size_t max_idx = i;
        for (size_t j = i + 1; j < filtered.size(); ++j) {
            if (used[j]) continue;
            if (filtered[j].length() > filtered[max_idx].length()) max_idx = j;
        }
    const auto main_line = filtered[max_idx];
        used[max_idx] = true;
    std::vector<size_t> member_idx;
    member_idx.push_back(max_idx);

        // 主线切向量 t、法向量 n、锚点 a（主线起点）与主线中点 m
        const double asx = main_line.getStart()[0], asy = main_line.getStart()[1];
        const double aex = main_line.getEnd()[0],   aey = main_line.getEnd()[1];
        const auto t_dir = get_dir(main_line); // 切向量
        const auto n_dir = rot90(t_dir[0], t_dir[1]); // 法向量
        const double ax = asx, ay = asy; // 锚点
        const double mx = 0.5 * (asx + aex);
        const double my = 0.5 * (asy + aey);

        // 收集同组端点
        std::vector<std::array<double,2>> pts;
        pts.push_back({asx, asy});
        pts.push_back({aex, aey});

        for (size_t j = 0; j < filtered.size(); ++j) {
            if (used[j]) continue;
            const auto& Lj = filtered[j];
            const auto tj = get_dir(Lj);
            double cosang = std::fabs(dot2(t_dir[0], t_dir[1], tj[0], tj[1]));
            if (cosang >= cos_thresh) {
                const double jmx = 0.5 * (Lj.getStart()[0] + Lj.getEnd()[0]);
                const double jmy = 0.5 * (Lj.getStart()[1] + Lj.getEnd()[1]);
                double pd = std::fabs(dot2(jmx - mx, jmy - my, n_dir[0], n_dir[1]));
                if (pd <= parallel_thresh) {
                    pts.push_back({Lj.getStart()[0], Lj.getStart()[1]});
                    pts.push_back({Lj.getEnd()[0], Lj.getEnd()[1]});
                    used[j] = true;
                    member_idx.push_back(j);
                }
            }
        }

        // 端点沿切向量以主线起点为锚点投影，取最小/最大并重建首尾（直接首尾连接）
        double min_s = 1e18, max_s = -1e18;
        for (const auto& p : pts) {
            const double s = dot2(p[0] - ax, p[1] - ay, t_dir[0], t_dir[1]);
            if (s < min_s) min_s = s;
            if (s > max_s) max_s = s;
        }
        SimpleLine sl{};
        sl.angle = std::atan2(t_dir[1], t_dir[0]);
        sl.length = std::max(0.0, max_s - min_s);
        sl.start[0] = ax + min_s * t_dir[0];
        sl.start[1] = ay + min_s * t_dir[1];
        sl.end[0]   = ax + max_s * t_dir[0];
        sl.end[1]   = ay + max_s * t_dir[1];
    processed_lines_.push_back(sl);
    groups.push_back(std::move(member_idx));
    }

    // 打印统计
    double tb = 0.0; for (const auto& l : lines_) tb += l.length();
    double ta = 0.0; for (const auto& l : processed_lines_) ta += l.length;
    // ROS_INFO_STREAM("[degeneracy_detector] lines(before=" << lines_.size() << ", sum=" << tb
    //                 << ") processed(" << processed_lines_.size() << ", sum=" << ta << ")");

    // 角度差（忽略正反方向），以pi为周期，返回[0,pi/2]
    auto angle_diff_pi = [](double a, double b) {
        double d = std::fabs(a - b);
        // 将差异归一到[0,pi]
        while (d > M_PI) d -= M_PI;
        if (d > M_PI_2) d = M_PI - d;
        return d;
    };
    const double k5deg = 5.0 * M_PI / 180.0;
    const double k10deg = 10.0 * M_PI / 180.0;

    // 规则二：
    // （1）处理后的线段集合仅有一条直线L，且长度>2m；
    // （2）原始集合 lines_ 中除参与组合直线L的线段以外，其余线段需同时满足：
    //      与L的平行距离 < 1m 且 线段长度 < 1m；
    if (processed_lines_.size() == 1) {
        const auto& L = processed_lines_[0];
        if (L.length > 2.0) {
            // L 的法向与中点，用于计算平行距离
            const double nx = std::cos(L.angle + M_PI_2);
            const double ny = std::sin(L.angle + M_PI_2);
            const double Lmx = 0.5 * (L.start[0] + L.end[0]);
            const double Lmy = 0.5 * (L.start[1] + L.end[1]);
            // 构建参与组合的索引掩码（filtered 与 lines_ 顺序一致）
            std::vector<uint8_t> in_group(lines_.size(), 0);
            if (!groups.empty()) {
                for (size_t idx : groups[0]) if (idx < in_group.size()) in_group[idx] = 1;
            }
            bool ok = true;
            for (size_t i = 0; i < lines_.size(); ++i) {
                if (in_group[i]) continue;  // 跳过参与组合的线段
                const auto& l = lines_[i];
                const double len = l.length();
                const double cmx = 0.5 * (l.getStart()[0] + l.getEnd()[0]);
                const double cmy = 0.5 * (l.getStart()[1] + l.getEnd()[1]);
                const double pd = std::fabs((cmx - Lmx) * nx + (cmy - Lmy) * ny);
                if (!(pd < 1.0 && len < 1.0)) { ok = false; break; }
            }
            if (ok) return true;
        }
    }

    // 规则三：
    // （1）处理后的线段集合仅有两条直线且两者平行(角度差<=阈值)、平行距离>1m，且两条线长度均>2m；
    // （2）原始集合 lines_ 中所有直线与这两条直线（相同朝向）夹角<=5度；
    if (processed_lines_.size() == 2) {
        const auto& a = processed_lines_[0];
        const auto& b = processed_lines_[1];
        // 平行性（允许小角度误差，这里采用5度）
        const double parallel_diff = angle_diff_pi(a.angle, b.angle);
        if (parallel_diff <= k5deg) {
            // 长度约束
            if (a.length > 2.0 && b.length > 2.0) {
                // 平行距离（使用a的法向）
                const double nx = std::cos(a.angle + M_PI_2);
                const double ny = std::sin(a.angle + M_PI_2);
                const double ax = 0.5 * (a.start[0] + a.end[0]);
                const double ay = 0.5 * (a.start[1] + a.end[1]);
                const double bx = 0.5 * (b.start[0] + b.end[0]);
                const double by = 0.5 * (b.start[1] + b.end[1]);
                const double pd = std::fabs((bx - ax) * nx + (by - ay) * ny);
                if (pd > 1.0) {
                    bool all_aligned = true;
                    for (const auto& l : lines_) {
                        const double diff = angle_diff_pi(a.angle, l.getAngle());
                        if (diff > k5deg) { all_aligned = false; break; }
                    }
                    if (all_aligned) return true;
                }
            }
        }
    }

    // 保留原有启发式
    //if (checkSingleLongLine()) return true;
    //if (checkAngleConsistency()) return true;
    return false;
}

double DegeneracyDetector::computeTotalLineLength()
{
    double sum = 0.0;
    for (const auto& l : lines_)
        sum += l.length();
    return sum;
}

bool DegeneracyDetector::checkSingleLongLine()
{
    double total = computeTotalLineLength();
    if (total <= 0)
        return false;
    double maxlen = 0.0;
    for (const auto& l : lines_)
        if (l.length() > maxlen)
            maxlen = l.length();
    if (maxlen < params_.min_long_line_length)
        return false;
    return (maxlen / total) >= params_.long_line_ratio;
}

bool DegeneracyDetector::checkAngleConsistency()
{
    std::vector<double> angs;
    angs.reserve(lines_.size());
    for (const auto& l : lines_)
        angs.push_back(l.getAngle());
    for (auto& a : angs) {
        while (a > M_PI_2)
            a -= M_PI;
        while (a <= -M_PI_2)
            a += M_PI;
    }
    double sx = 0.0, sy = 0.0;
    for (double a : angs) {
        sx += std::cos(a);
        sy += std::sin(a);
    }
    double mean_ang = std::atan2(sy, sx);
    int cnt = 0;
    for (double a : angs) {
        double diff = std::fabs(a - mean_ang);
        if (diff > M_PI_2)
            diff = std::fabs(diff - M_PI);
        if (diff <= params_.angle_consistency_thresh)
            ++cnt;
    }
    double ratio = static_cast<double>(cnt) / static_cast<double>(angs.size());
    return ratio >= params_.coverage_ratio;
}

void DegeneracyDetector::SetLidarParam(float angle_max, float angle_min, float angle_increment)
{
    std::vector<double> bearings, cb, sb;
    std::vector<unsigned int> idx;
    const std::size_t n = std::ceil((angle_max - angle_min) / angle_increment);
    for (std::size_t i = 0; i < n; ++i) {
        const double b = angle_min + i * angle_increment;
        bearings.push_back(b);
        cb.push_back(std::cos(b));
        sb.push_back(std::sin(b));
        idx.push_back(i);
    }
    extractor_.setCachedData(bearings, cb, sb, idx);
}

void DegeneracyDetector::SetLineExtractionFirstParam(double bearing_std_dev, double range_std_dev,
                                                     double least_sq_angle_thresh, double least_sq_radius_thresh,
                                                     double max_line_gap, double min_line_length)
{
    extractor_.setBearingVariance(bearing_std_dev * bearing_std_dev);
    extractor_.setRangeVariance(range_std_dev * range_std_dev);
    extractor_.setLeastSqAngleThresh(least_sq_angle_thresh);
    extractor_.setLeastSqRadiusThresh(least_sq_radius_thresh);
    extractor_.setMaxLineGap(max_line_gap);
    extractor_.setMinLineLength(min_line_length);
}

void DegeneracyDetector::SetLineExtractionSecondParam(int min_line_points, double min_range, double max_range,
                                                      double min_split_dist, double outlier_dist)
{
    extractor_.setMinRange(min_range);
    extractor_.setMaxRange(max_range);
    extractor_.setMinSplitDist(min_split_dist);
    extractor_.setOutlierDist(outlier_dist);
    extractor_.setMinLinePoints(static_cast<unsigned int>(min_line_points));
}
