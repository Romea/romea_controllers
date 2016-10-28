#include <ackermann_controller/odometry.h>

#include <boost/bind.hpp>

namespace ackermann_controller
{
  namespace bacc = boost::accumulators;

  Odometry::Odometry(size_t velocity_rolling_window_size)
  : timestamp_(0.0)
  , last_update_timestamp_(0.0)
  , x_(0.0)
  , y_(0.0)
  , heading_(0.0)
  , linear_(0.0)
  , angular_(0.0)
  , track_(0.0)
  , front_wheel_radius_(0.0)
  , rear_wheel_radius_(0.0)
  , wheel_base_(0.0)
  , left_wheel_old_pos_(0.0)
  , right_wheel_old_pos_(0.0)
  , wheel_old_pos_(0.0)
  , velocity_rolling_window_size_(velocity_rolling_window_size)
  , linear_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , angular_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  {
  }

  void Odometry::init(const ros::Time& time)
  {
    // Reset accumulators and timestamp:
    resetAccumulators();
    timestamp_ = time;
    last_update_timestamp_ = time;
  }

  bool Odometry::update(double front_wheel_angular_pos, double front_wheel_angular_vel,
                        double rear_wheel_angular_pos, double rear_wheel_angular_vel,
                        double front_steering, const ros::Time &time)
  {
    /// Get current wheel joint positions:
    const double wheel_cur_pos  = rear_wheel_angular_pos * rear_wheel_radius_;
    /// Estimate pos evolution of wheels using old and current position:
    const double wheel_est_diff_pos  = wheel_cur_pos  - wheel_old_pos_;
    /// Update old position with current:
    wheel_old_pos_ = wheel_cur_pos;

    const double angular_diff = wheel_est_diff_pos * tan(front_steering) / wheel_base_;
    /// Integrate odometry:
    integrateExact(wheel_est_diff_pos, angular_diff);

    linear_ = rear_wheel_angular_vel*rear_wheel_radius_;
    angular_ = linear_ * tan(front_steering) / wheel_base_;

    /// Compute x, y and heading using velocity
//    const double dt = (time - last_update_timestamp_).toSec();
//    last_update_timestamp_ = time;
//    /// Integrate odometry:
//    integrateExact(linear_*dt, angular_*dt);

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
    integrateExact(linear * dt, angular * dt);
  }

  void Odometry::setWheelParams(double track, double front_wheel_radius, double rear_wheel_radius, double wheel_base)
  {
    track_ = track;
    front_wheel_radius_     = front_wheel_radius;
    rear_wheel_radius_     = rear_wheel_radius;
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
    {
      integrateRungeKutta2(linear, angular);
    }
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
