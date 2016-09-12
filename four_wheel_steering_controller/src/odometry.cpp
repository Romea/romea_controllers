#include <ackermann_controller/odometry.h>

#include <boost/bind.hpp>

namespace ackermann_controller
{
  namespace bacc = boost::accumulators;

  Odometry::Odometry(size_t velocity_rolling_window_size)
  : timestamp_(0.0)
  , x_(0.0)
  , y_(0.0)
  , heading_(0.0)
  , linear_(0.0)
  , angular_(0.0)
  , wheel_separation_(0.0)
  , wheel_radius_(0.0)
  , wheel_base_(0.0)
  , left_wheel_old_pos_(0.0)
  , right_wheel_old_pos_(0.0)
  , wheel_old_pos_(0.0)
  , velocity_rolling_window_size_(velocity_rolling_window_size)
  , linear_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , angular_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , integrate_fun_(boost::bind(&Odometry::integrateExact, this, _1, _2))
  {
  }

  void Odometry::init(const ros::Time& time)
  {
    // Reset accumulators and timestamp:
    resetAccumulators();
    timestamp_ = time;
  }

  bool Odometry::update(double left_pos, double right_pos, const ros::Time &time)
  {
    /// Get current wheel joint positions:
    const double left_wheel_cur_pos  = left_pos  * wheel_radius_;
    const double right_wheel_cur_pos = right_pos * wheel_radius_;

    /// Estimate velocity of wheels using old and current position:
    const double left_wheel_est_vel  = left_wheel_cur_pos  - left_wheel_old_pos_;
    const double right_wheel_est_vel = right_wheel_cur_pos - right_wheel_old_pos_;

    /// Update old position with current:
    left_wheel_old_pos_  = left_wheel_cur_pos;
    right_wheel_old_pos_ = right_wheel_cur_pos;

    /// Compute linear and angular diff:
    const double linear  = (right_wheel_est_vel + left_wheel_est_vel) * 0.5 ;
    const double angular = (right_wheel_est_vel - left_wheel_est_vel) / wheel_separation_;

    /// Integrate odometry:
    integrate_fun_(linear, angular);

    /// We cannot estimate the speed with very small time intervals:
    const double dt = (time - timestamp_).toSec();
    if (dt < 0.0001)
      return false; // Interval too small to integrate with

    timestamp_ = time;

    /// Estimate speeds using a rolling mean to filter them out:
    linear_acc_(linear/dt);
    angular_acc_(angular/dt);

    linear_ = bacc::rolling_mean(linear_acc_);
    angular_ = bacc::rolling_mean(angular_acc_);

    return true;
  }


  bool Odometry::update(double wheel_angular_pos, double wheel_angular_vel, double front_steering, const ros::Time &time)
  {
    /// Get current wheel joint positions:
    const double wheel_cur_pos  = wheel_angular_pos  * wheel_radius_;
    /// Estimate velocity of wheels using old and current position:
    const double wheel_est_vel  = wheel_cur_pos  - wheel_old_pos_;
    /// Update old position with current:
    wheel_old_pos_  = wheel_cur_pos;

    const double angular = wheel_est_vel * tan(front_steering) / wheel_base_;
    /// Integrate odometry:
    integrate_fun_(wheel_est_vel, angular);

    linear_ = wheel_angular_vel*wheel_radius_;
    angular_ = linear_ * tan(front_steering) / wheel_base_;

    return true;
  }

  void Odometry::updateOpenLoop(double linear, double angular, const ros::Time &time)
  {
    /// Save last linear and angular velocity:
    linear_ = linear;
    angular_ = angular;

    /// Integrate odometry:
    const double dt = (time - timestamp_).toSec();
    timestamp_ = time;
    integrate_fun_(linear * dt, angular * dt);
  }

  void Odometry::setWheelParams(double wheel_separation, double wheel_radius, double wheel_base)
  {
    wheel_separation_ = wheel_separation;
    wheel_radius_     = wheel_radius;
    wheel_base_       = wheel_base;
  }

  void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
  {
    velocity_rolling_window_size_ = velocity_rolling_window_size;

    resetAccumulators();
  }

  void Odometry::integrateRungeKutta2(double linear, double angular)
  {
    const double direction = heading_ + angular * 0.5;

    /// Runge-Kutta 2nd order integration:
    x_       += linear * cos(direction);
    y_       += linear * sin(direction);
    heading_ += angular;
  }

  /**
   * \brief Other possible integration method provided by the class
   * \param linear
   * \param angular
   */
  void Odometry::integrateExact(double linear, double angular)
  {
    if (fabs(angular) < 1e-6)
      integrateRungeKutta2(linear, angular);
    else
    {
      /// Exact integration (should solve problems when angular is zero):
      const double heading_old = heading_;
      const double r = linear/angular;
      heading_ += angular;
      x_       +=  r * (sin(heading_) - sin(heading_old));
      y_       += -r * (cos(heading_) - cos(heading_old));
    }
  }

  void Odometry::resetAccumulators()
  {
    linear_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    angular_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
  }

} // namespace ackermann_controller
