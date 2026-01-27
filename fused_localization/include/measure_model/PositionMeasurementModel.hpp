#ifndef KALMAN_EXAMPLES_ROBOT1_POSITIONMEASUREMENTMODEL_HPP_
#define KALMAN_EXAMPLES_ROBOT1_POSITIONMEASUREMENTMODEL_HPP_

#include <ros/ros.h>

#include <kalman/LinearizedMeasurementModel.hpp>

#include "system_model.h"

namespace location {

/**
 * @brief Measurement vector measuring the robot position
 *
 * @param T Numeric scalar type
 */
template <typename T>
class PositionMeasurement : public Kalman::Vector<T, 2> {
 public:
  KALMAN_VECTOR(PositionMeasurement, T, 2)

  // landmark x
  static constexpr size_t X = 0;
  // landmark y
  static constexpr size_t Y = 1;

  T x() const { return (*this)[X]; }  // x
  T y() const { return (*this)[Y]; }  // y

  T& x() { return (*this)[X]; }
  T& y() { return (*this)[Y]; }
};

/**
 * @brief Measurement vector measuring the reflector
 *
 * @param T Numeric scalar type
 */
template <typename T>
class ReflectorMeasurement : public Kalman::Vector<T, 4> {
 public:
  KALMAN_VECTOR(ReflectorMeasurement, T, 4)

  // landmark x
  static constexpr size_t X = 0;
  // landmark y
  static constexpr size_t Y = 1;
  // landmark x
  static constexpr size_t Angle = 2;
  // landmark y
  static constexpr size_t Length = 3;

  T x() const { return (*this)[X]; }            // x
  T y() const { return (*this)[Y]; }            // y
  T angle() const { return (*this)[Angle]; }    // Angle
  T length() const { return (*this)[Length]; }  // Length

  T& x() { return (*this)[X]; }
  T& y() { return (*this)[Y]; }
  T& angle() { return (*this)[Angle]; }
  T& length() { return (*this)[Length]; }
};

/**
 * @brief Measurement model for measuring the position of the robot
 *        using two beacon-landmarks
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
class PositionMeasurementModel
    : public Kalman::LinearizedMeasurementModel<
          State<T>, PositionMeasurement<T>, CovarianceBase> {
 public:
  //! State type shortcut definition
  typedef State<T> S;

  //! Measurement type shortcut definition
  typedef PositionMeasurement<T> M;

  /**
   * @brief Constructor
   *
   * @param landmark1x The x-position of landmark 1
   */
  PositionMeasurementModel() {
    landmark << 0., 0., 0., 0.;

    // Setup noise jacobian. As this one is static, we can define it once
    // and do not need to update it dynamically
    this->V.setIdentity();
  }

  void SetLandmark(const ReflectorMeasurement<T>& reflector) {
    landmark = reflector;
    // ROS_DEBUG_STREAM("[SetLandmark] x:" << landmark[0]
    //                                     << " y: " << landmark[1]);
  }

  std::map<std::string, double> GetLandmark() {
    std::map<std::string, double> lk;
    lk.emplace("x", landmark.x());
    lk.emplace("y", landmark.y());
    return lk;
  }

  void SetTransform(const State<T>& transf) { transform = transf; }

  void SetNoise(T x, T y) {
    this->V.setIdentity();
    this->V(0, 0) = x;
    this->V(1, 1) = y;
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
#if 0
    ROS_DEBUG_STREAM("[measurement h] S:(" << x.x() << " " << x.y() << " "
                                           << x.theta());
    ROS_DEBUG_STREAM("[measurement h] transform:(" << transform[0] << " "
                                                   << transform[1] << " "
                                                   << transform[2] << ")");
#endif
    Kalman::Vector<T, 2> position;
    position << x.x(), x.y();

    Kalman::Vector<T, 2> landmark_V;
    landmark_V << landmark.x(), landmark.y();
    // landmark_V << landmark.length() * cos(landmark.angle()),
    //     landmark.length() * sin(landmark.angle());

    Eigen::Rotation2D<T> rot_pose(x.theta());

    Kalman::Vector<T, 2> body_trans;
    body_trans << transform[0], transform[1];
    Eigen::Rotation2D<T> rot_body(x.theta() + transform[2]);

    // M measurement = position + rot_pose.toRotationMatrix() * landmark_V;
    M measurement = position + rot_pose.toRotationMatrix() * body_trans +
                    rot_body.toRotationMatrix() * landmark_V;

    return measurement;
  }

 protected:
  //! Position of landmark  given as (x,y,angle,length)-measurement
  ReflectorMeasurement<T> landmark;

  // transform of landmark  given as (x,y,angle)-measurement
  Kalman::Vector<T, 3> transform;

 protected:
  /**
   * @brief Update jacobian matrices for the system state transition function
   * using current state
   *
   * This will re-compute the (state-dependent) elements of the jacobian
   * matrices to linearize the non-linear measurement function \f$h(x)\f$ around
   * the current state \f$x\f$.
   *
   * @note This is only needed when implementing a LinearizedSystemModel,
   *       for usage with an ExtendedKalmanFilter or
   * SquareRootExtendedKalmanFilter. When using a fully non-linear filter such
   * as the UnscentedKalmanFilter or its square-root form then this is not
   * needed.
   *
   * @param x The current system state around which to linearize
   * @param u The current system control input
   */
  void updateJacobians(const S& x) {
    // H = dh/dx (Jacobian of measurement function w.r.t. the state)
    this->H.setZero();

    // Robot position as (x,y)-vector
    // This uses the Eigen template method to get the first 2 elements of the
    // vector
    // Kalman::Vector<T, 2> position = x.template head<2>();

    // Distance of robot to landmark 1
    // Kalman::Vector<T, 2> delta1 = position - landmark1;

    // Distance of robot to landmark 2
    // Kalman::Vector<T, 2> delta2 = position - landmark2;

    // Distances
    // T d1 = std::sqrt(delta1.dot(delta1));
    // T d2 = std::sqrt(delta2.dot(delta2));

    // partial derivative of meas.d1() w.r.t. x.x()
    // this->H(M::x, S::X) = delta1[0] / d1;
    // partial derivative of meas.d1() w.r.t. x.y()
    // this->H(M::D1, S::Y) = delta1[1] / d1;

    // partial derivative of meas.d1() w.r.t. x.x()
    // this->H(M::y, S::X) = delta2[0] / d2;
    // partial derivative of meas.d1() w.r.t. x.y()
    // this->H(M::D2, S::Y) = delta2[1] / d2;
    /*
        this->H(M::X, S::X) = 1;
        this->H(M::X, S::Y) = 0;
        this->H(M::X, S::THETA) = -transform[0] * sin(x.theta()) -
                                  transform[1] * cos(x.theta()) -
                                  landmark.length() * cos(landmark.angle()) *
                                      sin(x.theta() + transform[2]) -
                                  landmark.length() * sin(landmark.angle()) *
                                      cos(x.theta() + transform[2]);

        this->H(M::Y, S::X) = 0;
        this->H(M::Y, S::Y) = 1;
        this->H(M::Y, S::THETA) = transform[0] * cos(x.theta()) -
                                  transform[1] * sin(x.theta()) +
                                  landmark.length() * cos(landmark.angle()) *
                                      cos(x.theta() + transform[2]) -
                                  landmark.length() * sin(landmark.angle()) *
                                      sin(x.theta() + transform[2]);

        */
    /*
        this->H(M::X, S::X) = 1;
        this->H(M::X, S::Y) = 0;
        this->H(M::X, S::THETA) =
            -landmark.x() * sin(x.theta()) - landmark.y() * cos(x.theta());

        this->H(M::Y, S::X) = 0;
        this->H(M::Y, S::Y) = 1;
        this->H(M::Y, S::THETA) =
            landmark.x() * cos(x.theta()) - landmark.y() * sin(x.theta());
    */

    this->H(M::X, S::X) = 1;
    this->H(M::X, S::Y) = 0;
    this->H(M::X, S::THETA) =
        transform[0] * cos(x.theta()) - transform[1] * sin(x.theta()) +
        -landmark.x() * sin(x.theta()) - landmark.y() * cos(x.theta());

    this->H(M::Y, S::X) = 0;
    this->H(M::Y, S::Y) = 1;
    this->H(M::Y, S::THETA) =
        transform[0] * cos(x.theta()) - transform[1] * sin(x.theta()) +
        landmark.x() * cos(x.theta()) - landmark.y() * sin(x.theta());
  }
};

}  // namespace location

#endif