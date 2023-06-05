#pragma once

#include <ros/time.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/function.hpp>

namespace dual_wheel_steering_controller
{
  namespace bacc = boost::accumulators;

  class Odometry
  {
  public:

    /// Integration function, used to integrate the odometry:
    typedef boost::function<void(double, double)> IntegrationFunction;

    Odometry(size_t velocity_rolling_window_size = 10);

    void init(const ros::Time &time);

    bool update(double steer_front_pos, double steer_rear_pos, double drive_front_pos, double drive_rear_pos, const ros::Time &time);

    void updateOpenLoop(double linear, double angular, const ros::Time &time);

    double getHeading() const
    {
      return heading_;
    }

    double getX() const
    {
      return x_;
    }

    double getY() const
    {
      return y_;
    }

    double getLinear() const
    {
      return linear_;
    }

    double getLinearX() const
    {
      return linear_x_;
    }

    double getLinearY() const
    {
      return linear_y_;
    }

    double getAngular() const
    {
      return angular_;
    }

    void setWheelParams(double wheel_diameter, double wheel_base);

    void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

  private:

    double computeOdomHVariable(double wheel_base);

    /// Rolling mean accumulator and window:
    typedef bacc::accumulator_set<double, bacc::stats<bacc::tag::rolling_mean> > RollingMeanAcc;
    typedef bacc::tag::rolling_window RollingWindow;

    void integrateRungeKutta2(double linear, double angular);

    void integrateExact(double linear, double angular);

    void integrateXY(double steer_front_diff_pos, double steer_rear_diff_pos, double drive_front_diff_pos, double drive_rear_diff_pos, double dt);

    void resetAccumulators();

    /// Current timestamp:
    ros::Time timestamp_;

    /// Current pose:
    double x_;        //   [m]
    double y_;        //   [m]
    double heading_;  // [rad]

    /// Current velocity:
    double linear_;  //   [m/s]
    double linear_x_;  //   [m/s]
    double linear_y_;  //   [m/s]
    double angular_; // [rad/s]

    /// Wheel kinematic parameters [m]:
    double wheel_diameter_;
    double wheel_base_;

    // Odometry parameter from H matrix
    double odom_h_variable_;

    /// Previou wheel position/state [steer -> rad], [drive -> m]:
    double steer_front_wheel_old_pos_;
    double steer_rear_wheel_old_pos_;
    double drive_front_wheel_old_pos_;
    double drive_rear_wheel_old_pos_;


    /// Rolling mean accumulators for the linar and angular velocities:
    size_t velocity_rolling_window_size_;
    RollingMeanAcc linear_acc_;
    RollingMeanAcc linear_x_acc_;
    RollingMeanAcc linear_y_acc_;
    RollingMeanAcc angular_acc_;

    /// Integration funcion, used to integrate the odometry:
    IntegrationFunction integrate_fun_;
  };
}
