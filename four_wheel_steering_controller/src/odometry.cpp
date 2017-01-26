#include <four_wheel_steering_controller/odometry.h>

#include <boost/bind.hpp>

namespace four_wheel_steering_controller
{
  namespace bacc = boost::accumulators;

  Odometry::Odometry(size_t velocity_rolling_window_size)
  : timestamp_(0.0)
  , last_update_timestamp_(0.0)
  , x_(0.0)
  , y_(0.0)
  , heading_(0.0)
  , linear_(0.0)
  , linear_x_(0.0)
  , linear_y_(0.0)
  , angular_(0.0)
  , track_(0.0)
  , wheel_radius_(0.0)
  , wheel_base_(0.0)
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
  }


  bool Odometry::update(const double &fl_speed, const double &fr_speed,
                        const double &rl_speed, const double &rr_speed,
                        double front_steering, double rear_steering, const ros::Time &time)
  {
//    const double front_tmp = cos(front_steering)*(tan(front_steering)-tan(rear_steering))/wheel_base_;
//    const double front_linear_speed = wheel_radius_ * copysign(1.0, fl_speed+fr_speed)*
//        sqrt((pow(fl_speed,2)+pow(fr_speed,2))/(2+pow(track_*front_tmp,2)/2.0));

    const double rear_tmp = cos(rear_steering)*(tan(front_steering)-tan(rear_steering))/wheel_base_;
    const double rear_linear_speed = wheel_radius_ * copysign(1.0, rl_speed+rr_speed)*
        sqrt((pow(rl_speed,2)+pow(rr_speed,2))/(2+pow(track_*rear_tmp,2)/2.0));

    angular_ = rear_linear_speed*rear_tmp;

    linear_x_ = rear_linear_speed*cos(rear_steering);
    linear_y_ = rear_linear_speed*sin(rear_steering)
                + wheel_base_*angular_/2.0;
    linear_ =  copysign(1.0, rear_linear_speed)*sqrt(pow(linear_x_,2)+pow(linear_y_,2));

    /// Compute x, y and heading using velocity
    const double dt = (time - last_update_timestamp_).toSec();
    last_update_timestamp_ = time;
    /// Integrate odometry:
    integrateXY(linear_x_*dt, linear_y_*dt, angular_*dt);

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

  void Odometry::setWheelParams(double track, double wheel_radius, double wheel_base)
  {
    track_ = track;
    wheel_radius_     = wheel_radius;
    wheel_base_       = wheel_base;
  }

  void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
  {
    velocity_rolling_window_size_ = velocity_rolling_window_size;

    resetAccumulators();
  }

  void Odometry::integrateXY(double linear_x, double linear_y, double angular)
  {
    const double delta_x = linear_x*cos(heading_) - linear_y*sin(heading_);
    const double delta_y = linear_x*sin(heading_) + linear_y*cos(heading_);

    x_ += delta_x;
    y_ += delta_y;
    heading_ += angular;
  }

  void Odometry::integrateRungeKutta2(double linear, double angular)
  {
    const double direction = heading_ + angular * 0.5;

    /// Runge-Kutta 2nd order integration:
    x_       += linear * cos(direction);
    y_       += linear * sin(direction);
    heading_ += angular;
  }

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

} // namespace four_wheel_steering_controller
