
#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <ros/time.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/function.hpp>

namespace four_wheel_steering_controller
{
  namespace bacc = boost::accumulators;

  /**
   * \brief The Odometry class handles odometry readings
   * (2D pose and velocity with related timestamp)
   */
  class Odometry
  {
  public:

    /// Integration function, used to integrate the odometry:
    typedef boost::function<void(double, double)> IntegrationFunction;

    /**
     * \brief Constructor
     * Timestamp will get the current time value
     * Value will be set to zero
     * \param velocity_rolling_window_size Rolling window size used to compute the velocity mean
     */
    Odometry(size_t velocity_rolling_window_size = 10);

    /**
     * \brief Initialize the odometry
     * \param time Current time
     */
    void init(const ros::Time &time);

    /**
     * \brief Updates the odometry class with latest wheels and steerings position
     * \param fl_speed front left wheel vehicle speed [rad/s]
     * \param fr_speed front right wheel vehicle speed [rad/s]
     * \param rl_speed rear left wheel vehicle speed [rad/s]
     * \param rr_speed rear right wheel vehicle speed [rad/s]
     * \param front_steering  steering position [rad]
     * \param rear_steering  steering position [rad]
     * \param time      Current time
     * \return true if the odometry is actually updated
     */
    bool update(const double& fl_speed, const double& fr_speed, const double& rl_speed, const double& rr_speed,
                double front_steering, double rear_steering, const ros::Time &time);

    /**
     * \brief Updates the odometry class with latest velocity command
     * \param linear  Linear velocity [m/s]
     * \param angular Angular velocity [rad/s]
     * \param time    Current time
     */
    void updateOpenLoop(double linear, double angular, const ros::Time &time);

    /**
     * \brief heading getter
     * \return heading [rad]
     */
    double getHeading() const
    {
      return heading_;
    }

    /**
     * \brief x position getter
     * \return x position [m]
     */
    double getX() const
    {
      return x_;
    }

    /**
     * \brief y position getter
     * \return y position [m]
     */
    double getY() const
    {
      return y_;
    }


    /**
     * \brief linear velocity getter norm
     * \return linear velocity [m/s]
     */
    double getLinear() const
    {
      return linear_;
    }

    /**
     * \brief linear velocity getter along X on the robot base link frame
     * \return linear velocity [m/s]
     */
    double getLinearX() const
    {
      return linear_x_;
    }

    /**
     * \brief linear velocity getter along Y on the robot base link frame
     * \return linear velocity [m/s]
     */
    double getLinearY() const
    {
      return linear_y_;
    }

    /**
     * \brief angular velocity getter
     * \return angular velocity [rad/s]
     */
    double getAngular() const
    {
      return angular_;
    }

    /**
     * \brief Sets the wheel parameters: radius and separation
     * \param track Seperation between left and right wheels [m]
     * \param wheel_radius     Wheel radius [m]
     * \param wheel_base       Wheel base [m]
     */
    void setWheelParams(double track, double wheel_radius, double wheel_base);

    /**
     * \brief Velocity rolling window size setter
     * \param velocity_rolling_window_size Velocity rolling window size
     */
    void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

  private:

    /// Rolling mean accumulator and window:
    typedef bacc::accumulator_set<double, bacc::stats<bacc::tag::rolling_mean> > RollingMeanAcc;
    typedef bacc::tag::rolling_window RollingWindow;

    /**
     * \brief Integrates the velocities (linear on x and y and angular)
     * \param linear_x  Linear  velocity along x of the robot frame  [m] (linear  displacement, i.e. m/s * dt) computed by encoders
     * \param linear_y  Linear  velocity along y of the robot frame   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
     * \param angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
     */
    void integrateXY(double linear_x, double linear_y, double angular);

    /**
     * \brief Integrates the velocities (linear and angular) using 2nd order Runge-Kutta
     * \param linear  Linear  velocity   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
     * \param angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
     */
    void integrateRungeKutta2(double linear, double angular);

    /**
     * \brief Integrates the velocities (linear and angular) using exact method
     * \param linear  Linear  velocity   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
     * \param angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
     */
    void integrateExact(double linear, double angular);

    /**
     *  \brief Reset linear and angular accumulators
     */
    void resetAccumulators();

    /// Current timestamp:
    ros::Time timestamp_, last_update_timestamp_;

    /// Current pose:
    double x_;        //   [m]
    double y_;        //   [m]
    double heading_;  // [rad]

    /// Current velocity:
    double linear_, linear_x_, linear_y_;  //   [m/s]
    double angular_; // [rad/s]

    /// Wheel kinematic parameters [m]:
    double track_;
    double wheel_radius_;
    double wheel_base_;

    /// Previous wheel position/state [rad]:
    double wheel_old_pos_;

    /// Rolling mean accumulators for the linar and angular velocities:
    size_t velocity_rolling_window_size_;
    RollingMeanAcc linear_acc_;
    RollingMeanAcc angular_acc_;
  };
}

#endif /* ODOMETRY_H_ */
