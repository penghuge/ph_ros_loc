#include "ceres_sm.h"

#include <ceres/cubic_interpolation.h>
#include <glog/logging.h>

namespace cartographer {
namespace mapping {
namespace scan_matching {

namespace {

class PoseDeltaCostFunctor
{
public:
    static ceres::CostFunction* CreateCostFunction(
      const double xy_weight, const double angle_weight,
      const double x, const double y, const double angle)
    {
        return new ceres::AutoDiffCostFunction<PoseDeltaCostFunctor, 3/*残差数组大小*/, 1, 1, 1>(
            new PoseDeltaCostFunctor(xy_weight, angle_weight, x, y, angle));
    }
    template <typename T>
    bool operator()(const T* const x, const T* const y, const T* const angle,
                  T* residual) const
    {
        residual[0] = (*x - _x) * _xy_weight;
        residual[1] = (*y - _y) * _xy_weight;
        residual[2] = (*angle - _angle) * _angle_weight;
        return true;
    }
private:
    PoseDeltaCostFunctor(const double xy_weight, const double angle_weight,
                       const double x, const double y, const double angle)
      : _x(x), _y(y), _xy_weight(xy_weight),
        _angle(angle), _angle_weight(angle_weight) {}
    PoseDeltaCostFunctor(const PoseDeltaCostFunctor&) = delete;
    PoseDeltaCostFunctor& operator=(const PoseDeltaCostFunctor&) = delete;

    const double _x, _y, _xy_weight;
    const double _angle, _angle_weight;
};

class OccupiedSpaceCostFunctor
{
public:
    static ceres::CostFunction* CreateOccupiedSpaceCostFunction2D(
      const double weight,
      const sensor::PointCloud& point_cloud,
      const Grid2D& grid)
    {
        return new ceres::AutoDiffCostFunction<
                OccupiedSpaceCostFunctor,
                ceres::DYNAMIC/*残差数组大小，因激光数会变化，大小不定，所以采用动态，此时需要指定大小*/,
                1, 1, 1>(new OccupiedSpaceCostFunctor(weight, point_cloud, grid),
                         point_cloud.size()/*指定残差数组大小*/);
    }
    template <typename T>
    bool operator()(const T* const pose_x,
                  const T* const pose_y,
                  const T* const pose_angle,
                  T* residual) const
    {
        Eigen::Matrix<T, 2, 1> translation(*pose_x, *pose_y);
        Eigen::Rotation2D<T> rotation(*pose_angle);
        Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();
        Eigen::Matrix<T, 3, 3> transform;
        transform << rotation_matrix, translation, T(0.), T(0.), T(1.);

        const GridArrayAdapter adapter(_grid);
        ceres::BiCubicInterpolator<GridArrayAdapter> interpolator(adapter);
        const MapLimits& limits = _grid.limits();

        for (size_t i = 0; i < _point_cloud.size(); ++i) {
          const Eigen::Matrix<T, 3, 1> point((T(_point_cloud[i].position.x())),
                                             (T(_point_cloud[i].position.y())),
                                             T(1.));
          const Eigen::Matrix<T, 3, 1> world = transform * point;
          interpolator.Evaluate(
              //将激光点在全局地图中的坐标转换到栅格地图中
              (limits.max().x() - world[0]) / limits.resolution() - 0.5,
              (limits.max().y() - world[1]) / limits.resolution() - 0.5,
              &residual[i]);
          residual[i] = _weight * residual[i];
        }
        return true;
    }
private:
    class GridArrayAdapter
    {
    public:
        enum { DATA_DIMENSION = 1 };
        explicit GridArrayAdapter(const Grid2D& grid) : _grid(grid) {}
        void GetValue(const int row, const int column, double* const value) const
        {
            //如果越界，返回最大的损失率
            if (row < 0 || column < 0 || row >= NumRows() || column >= NumCols())
            {
                *value = kMaxCorrespondenceCost;
            }
            else
            {
                //读取表中记录的损失值
                *value = static_cast<double>(_grid.GetCorrespondenceCost(
                    Eigen::Array2i(column, row)));
            }
        }
        int NumRows() const
        {
            return _grid.limits().cell_limits().num_y_cells + 2;
        }
        int NumCols() const
        {
            return _grid.limits().cell_limits().num_x_cells + 2;
        }
    private:
        const Grid2D& _grid;
    };
    private:
    OccupiedSpaceCostFunctor(const double weight,
                           const sensor::PointCloud& point_cloud,
                           const Grid2D& grid)
      : _point_cloud(point_cloud), _weight(weight), _grid(grid) {}
    OccupiedSpaceCostFunctor(const OccupiedSpaceCostFunctor&) = delete;
    OccupiedSpaceCostFunctor operator=(const OccupiedSpaceCostFunctor&) = delete;

    const sensor::PointCloud _point_cloud;
    const double _weight;
    const Grid2D& _grid;
};
} // namespace

CeresSM::CeresSM()
{
    _ceres_solver_options.linear_solver_type = ceres::DENSE_QR;
    _ceres_solver_options.num_threads = 4;
    _ceres_solver_options.max_num_iterations = 10;
    _ceres_solver_options.use_nonmonotonic_steps = true;

    _occupied_space_weight = 20.;
    _translation_weight = 10.;
    _rotation_weight = 1.;
}

CeresSM::~CeresSM() {}

void CeresSM::Match(const transform::Rigid2d& initial_pose_estimate,
                    const sensor::PointCloud& point_cloud,
                    const Grid2D& grid,
                    transform::Rigid2d* const pose_estimate) const
{
    double pose_x = initial_pose_estimate.translation().x();
    double pose_y = initial_pose_estimate.translation().y();
    double pose_angle = initial_pose_estimate.rotation().angle();

    ceres::Problem problem;
    CHECK(grid.GetGridType() == GridType::PROBABILITY_GRID);

    problem.AddResidualBlock(
      OccupiedSpaceCostFunctor::CreateOccupiedSpaceCostFunction2D(
          _occupied_space_weight /
              std::sqrt(static_cast<double>(point_cloud.size())),
          point_cloud,
          grid),
      nullptr, &pose_x, &pose_y, &pose_angle);

    problem.AddResidualBlock(
      PoseDeltaCostFunctor::CreateCostFunction(
          _translation_weight, _rotation_weight, pose_x, pose_y, pose_angle),
      nullptr, &pose_x, &pose_y, &pose_angle);

    ceres::Solver::Summary summary;
    ceres::Solve(_ceres_solver_options, &problem, &summary);
    LOG(INFO) << summary.FullReport();
    *pose_estimate = transform::Rigid2d({pose_x, pose_y}, pose_angle);
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
