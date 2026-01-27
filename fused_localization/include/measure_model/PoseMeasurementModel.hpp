#ifndef KALMAN_EXAMPLES_ROBOT1_POSEMEASUREMENTMODEL_HPP_
#define KALMAN_EXAMPLES_ROBOT1_POSEMEASUREMENTMODEL_HPP_

#include <ros/ros.h>

#include <kalman/LinearizedMeasurementModel.hpp>

#include "system_model.h"

namespace location {

/**
 * @brief Measurement vector measuring the robot pose
 *
 * @param T Numeric scalar type
 */
template <typename T>
class PoseMeasurement : public Kalman::Vector<T, 3> {
 public:
  KALMAN_VECTOR(PoseMeasurement, T, 3)

  //  x
  static constexpr size_t X = 0;
  //  y
  static constexpr size_t Y = 1;
  //  theta
  static constexpr size_t THETA = 2;

  T x() const { return (*this)[X]; }          // x
  T y() const { return (*this)[Y]; }          // y
  T theta() const { return (*this)[THETA]; }  // theta

  T& x() { return (*this)[X]; }
  T& y() { return (*this)[Y]; }
  T& theta() { return (*this)[THETA]; }
};

/**
 * @brief Measurement model for measuring the pose of the robot
 *
 * This is the measurement model for measuring the position of the robot.
 * The measurement is given by two landmarks in the space, whose positions are
 * known. The robot can measure the direct distance to both the landmarks, for
 * instance through visual localization techniques.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance
 * representation (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template <typename T,
          template <class> class CovarianceBase = Kalman::StandardBase>
class PoseMeasurementModel
    : public Kalman::LinearizedMeasurementModel<State<T>, PoseMeasurement<T>,
                                                CovarianceBase> {
 public:
  //! State type shortcut definition
  typedef State<T> S;

  //! Measurement type shortcut definition
  typedef PoseMeasurement<T> M;

  /**
   * @brief Constructor
   *
   * @param pose The x-position of landmark 1
   */
  PoseMeasurementModel() {
    pose << 0., 0., 0.;

    // Setup noise jacobian. As this one is static, we can define it once
    // and do not need to update it dynamically
    this->V.setIdentity();
  }

  void SetPose(const PoseMeasurement<T>& p) { pose = p; }

  std::map<std::string, double> GetPose() {
    std::map<std::string, double> p;
    p.emplace("x", pose.x());
    p.emplace("y", pose.y());
    p.emplace("theta", pose.theta());
    return p;
  }

  void SetTransform(const State<T>& transf) { transform = transf; }

  void SetNoise(T x, T y, T theta) {
    this->V(0, 0) = x;
    this->V(1, 1) = y;
    this->V(2, 2) = theta;
  }

  /**
   * @brief Definition of (possibly non-linear) measurement function
   *
   * This function maps the system state to the measurement that is expected
   * to be received from the sensor assuming the system is currently in the
   * estimated state.
   *
   * @param [in] x The system state in current time-step
   * @returns The (predicted) sensor measurement for the system state
   */
  M h(const S& x) const {
    M measurement;
    measurement << x.x(), x.y(), x.theta();
    return measurement;
  }

 protected:
  //! pose  given as (x,y,angle)-measurement
  PoseMeasurement<T> pose;

  // transform of pose  given as (x,y,angle)-measurement
  Kalman::Vector<T, 3> transform;

 protected:
  /**
   * @brief
   * @note
   */
  void updateJacobians(const S& x) {
    // H = dh/dx (Jacobian of measurement function w.r.t. the state)
    // this->H.setZero();
    this->H.setIdentity();
  }
};

}  // namespace location

#endif