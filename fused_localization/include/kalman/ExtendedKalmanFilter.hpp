// The MIT License (MIT)
//
// Copyright (c) 2015 Markus Herb
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
#ifndef KALMAN_EXTENDEDKALMANFILTER_HPP_
#define KALMAN_EXTENDEDKALMANFILTER_HPP_

#include <angles/angles.h>
#include <ros/ros.h>

#include <iostream>

#include "KalmanFilterBase.hpp"
#include "LinearizedMeasurementModel.hpp"
#include "LinearizedSystemModel.hpp"
#include "StandardFilterBase.hpp"

namespace Kalman {

/**
 * @brief Extended Kalman Filter (EKF)
 *
 * This implementation is based upon [An Introduction to the Kalman
 * Filter](https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf) by Greg
 * Welch and Gary Bishop.
 *
 * @param StateType The vector-type of the system state (usually some type
 * derived from Kalman::Vector)
 */
template <class StateType>
class ExtendedKalmanFilter : public KalmanFilterBase<StateType>,
                             public StandardFilterBase<StateType> {
 public:
  //! Kalman Filter base type
  typedef KalmanFilterBase<StateType> KalmanBase;
  //! Standard Filter base type
  typedef StandardFilterBase<StateType> StandardBase;

  //! Numeric Scalar Type inherited from base
  using typename KalmanBase::T;

  //! State Type inherited from base
  using typename KalmanBase::State;

  //! Linearized Measurement Model Type
  template <class Measurement, template <class> class CovarianceBase>
  using MeasurementModelType =
      LinearizedMeasurementModel<State, Measurement, CovarianceBase>;

  //! Linearized System Model Type
  template <class Control, template <class> class CovarianceBase>
  using SystemModelType = LinearizedSystemModel<State, Control, CovarianceBase>;
  std::mutex ekf_pose_mutex;

 protected:
  //! Kalman Gain Matrix Type
  template <class Measurement>
  using KalmanGain = Kalman::KalmanGain<State, Measurement>;

 protected:
  //! State Estimate
  using KalmanBase::x;
  //! State Covariance Matrix
  using StandardBase::P;

 public:
  /**
   * @brief Constructor
   */
  ExtendedKalmanFilter() {
    // Setup state and covariance
    P.setIdentity();
  }

  //lyy
  void init(const State& initialState)
  {
      std::unique_lock<std::mutex> lock(ekf_pose_mutex, std::defer_lock);
      if(lock.try_lock()){
          x = initialState;
          x.theta() = angles::normalize_angle(x.theta());
          P.setIdentity();
      }
  }

  /**
   * @brief get the trace of P(State Covariance Matrix)
   */
  double getErrorRange() { return P.trace(); }

  /**
   * @brief Perform filter prediction step using system model and no control
   * input (i.e. \f$ u = 0 \f$)
   *
   * @param [in] s The System model
   * @return The updated state estimate
   */
  template <class Control, template <class> class CovarianceBase>
  const State& predict(SystemModelType<Control, CovarianceBase>& s) {
    // predict state (without control)
    Control u;
    u.setZero();
    return predict(s, u);
  }

  /**
   * @brief Perform filter prediction step using control input \f$u\f$ and
   * corresponding system model
   *
   * @param [in] s The System model
   * @param [in] u The Control input vector
   * @return The updated state estimate
   */
  template <class Control, template <class> class CovarianceBase>
  const State& predict(SystemModelType<Control, CovarianceBase>& s, const Control& u) {
    std::unique_lock<std::mutex> lock(ekf_pose_mutex, std::defer_lock);
    if(lock.try_lock()){
      s.updateJacobians(x, u);

      // predict state
      x = s.f(x, u);
      x.theta() = angles::normalize_angle(x.theta());

      // predict covariance
      P = (s.F * P * s.F.transpose()) +
          (s.W * s.getCovariance() * s.W.transpose());
      
      // std::stringstream ss;
      // ss << "lyy0417: predict: s.W: \n" << s.W <<"\n, s.getCovariance: \n" << s.getCovariance() << "\n, s.F: \n" << s.F
      //                 << "\n, x: \n" << x << "\n, P: \n" << P;
      // ROS_INFO("%s", ss.str().c_str());
    }
    // return state prediction
    return this->getState();
  }

  /**
   * @brief Perform filter update step using measurement \f$z\f$ and
   * corresponding measurement model
   *
   * @param [in] m The Measurement model
   * @param [in] z The measurement vector
   * @return The updated state estimate
   */
  template <class Measurement, template <class> class CovarianceBase>
  const State& update(MeasurementModelType<Measurement, CovarianceBase>& m, const Measurement& z) {
    std::unique_lock<std::mutex> lock(ekf_pose_mutex, std::defer_lock);
    if(lock.try_lock()){
      m.updateJacobians(x);

      // COMPUTE KALMAN GAIN
      // compute innovation covariance
      Covariance<Measurement> S = (m.H * P * m.H.transpose()) + (m.V * m.getCovariance() * m.V.transpose());

      // compute kalman gain
      KalmanGain<Measurement> K = P * m.H.transpose() * S.inverse();

      // UPDATE STATE ESTIMATE AND COVARIANCE
      // Update state using computed kalman gain and innovation

      Eigen::Quaterniond x_quat(Eigen::AngleAxisd(x.theta(), Eigen::Vector3d::UnitZ()));
      Eigen::Quaterniond hx_quat(Eigen::AngleAxisd(m.h(x).theta(), Eigen::Vector3d::UnitZ()));
      Eigen::Quaterniond z_quat(Eigen::AngleAxisd(z.theta(), Eigen::Vector3d::UnitZ()));
      double predict_x = x.x();
      // x += K * (z - m.h(x));
      Measurement delta_measurement = z - m.h(x);
      delta_measurement.theta() = (z_quat * hx_quat.inverse()).normalized().toRotationMatrix().eulerAngles(0, 1, 2)[2];

      // std::stringstream ss;
      // ss << "lyy0417: update1: m.V: \n" << m.V <<"\n, m.getCovariance: \n" << m.getCovariance() << "\n, m.H: \n" << m.H << "\n, P: \n" << P
      //    << "\n, S: \n" << S << "\n, K: \n" << K << "\n, z - m.h( x ): \n" << delta_measurement << "\n, x: \n" << x;
      // ROS_INFO("%s", ss.str().c_str());

      x += K * delta_measurement;

      // x.theta() = angles::normalize_angle(x.theta());
      double ratio = 1.0 - K(2, 2);
      Eigen::Quaterniond fuse_quat = z_quat.slerp(ratio, x_quat);
      ROS_INFO("lyy0417: update1: predict_x: %f, z.x: %f, fuse.x: %f, k-x-y-th: %f, %f, %f", predict_x, z.x(), x.x(), K(0,0), K(1,1), K(2,2));
      x.theta() = fuse_quat.normalized().toRotationMatrix().eulerAngles(0, 1, 2)[2];

      // Update covariance
      P -= K * m.H * P;
      // std::stringstream ss2;
      // ss2 << "lyy0417: update2: x: \n" << x <<"\n, P: \n" << P;
      // ROS_INFO("%s", ss2.str().c_str());
    }

    // return updated state estimate
    return this->getState();
  }
};

}  // namespace Kalman

#endif  //
