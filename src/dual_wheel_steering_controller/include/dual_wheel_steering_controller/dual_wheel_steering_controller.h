#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <dual_wheel_steering_controller/odometry.h>
#include <dual_wheel_steering_controller/speed_limiter.h>

namespace dual_wheel_steering_controller
{

    class DualWheelSteeringController
        : public controller_interface::MultiInterfaceController<hardware_interface::VelocityJointInterface,
                                                                hardware_interface::PositionJointInterface>
    {
    public:
        DualWheelSteeringController();

        bool init(hardware_interface::RobotHW *robot_hw,
                  ros::NodeHandle &root_nh,
                  ros::NodeHandle &controller_nh);

        void update(const ros::Time &time, const ros::Duration &period);

        void starting(const ros::Time &time);

        void stopping(const ros::Time & /*time*/);

    private:
        std::string name_;

        bool isStopBefore_ = false;

        /// Odometry related:
        ros::Duration publish_period_;
        ros::Time last_state_publish_time_;
        bool open_loop_;

        /// Hardware handles:
        hardware_interface::JointHandle front_wheel_joint_;
        hardware_interface::JointHandle rear_wheel_joint_;
        hardware_interface::JointHandle front_steer_joint_;
        hardware_interface::JointHandle rear_steer_joint_;

        /// Velocity command related:
        struct Command
        {
            ros::Time stamp;

            Command() : stamp(0.0) {}
        };
        struct CommandTwist : Command
        {
            double lin_x;
            double lin_y;
            double ang;

            CommandTwist() : lin_x(0.0), lin_y(0.0), ang(0.0) {}
        };
        realtime_tools::RealtimeBuffer<CommandTwist> command_twist_;
        CommandTwist command_struct_twist_;
        ros::Subscriber sub_command_;

        /// Odometry related:
        std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_;
        std::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage>> tf_odom_pub_;
        Odometry odometry_;

        /// Wheel radius (assuming it's the same for the front and rear wheels):
        double wheel_radius_;

        /// Wheel base, the horizontal distance between the centers of the front and rear wheels
        double wheel_base_;

        /// Wheel base and radius calibration multipliers:
        double wheel_base_multiplier_;
        double wheel_radius_multiplier_;

        // Wheel base and radius after calibration.
        double wb_;
        double wr_;

        /// Timeout to consider cmd_vel commands old:
        double cmd_vel_timeout_;

        /// Whether to allow multiple publishers on cmd_vel topic or not:
        bool allow_multiple_cmd_vel_publishers_;

        /// Frame to use for the robot base:
        std::string base_frame_id_;

        /// Frame to use for odometry and odom tf:
        std::string odom_frame_id_;

        /// Whether to publish odometry to tf or not:
        bool enable_odom_tf_;

        /// Whether the control is make with four_wheel_steering msg or twist msg:
        bool enable_twist_cmd_;

        /// Throttled DEBUG output will print a message at most once per "period" sec:
        double debug_period_time_;

        /// 上一個時刻 前輪的轉向控制令 unit rad 初始值為 0
        double last_front_steer_pos_cmd_;
        /// 上一個時刻 後輪的轉向控制令 unit rad 初始值為 0
        double last_rear_steer_pos_cmd_;

        int controller_version_;

        // last cmds
        double last1_vx_cmd_;
        double last0_vx_cmd_;
        double last1_vy_cmd_;
        double last0_vy_cmd_;
        double last1_w_cmd_;
        double last0_w_cmd_;

        /// Speed limiters:
        SpeedLimiter limiter_vx_;
        SpeedLimiter limiter_vy_;
        SpeedLimiter limiter_w_;

        /**
         * \brief Get the wheel names from a wheel param
         * \param [in]  controller_nh Controller node handler
         * \param [in]  wheel_param   Param name
         * \param [out] wheel_names   Vector with the whel names
         * \return true if the wheel_param is available and the wheel_names are
         *        retrieved successfully from the param server; false otherwise
         *  TODO 更新 joints 初始化方式之後，就不需要getWheelNames()函式的功能了，但是程式碼以後還是可以參考
         */
        bool getWheelNames(ros::NodeHandle &controller_nh,
                           const std::string &wheel_param,
                           std::vector<std::string> &wheel_names);

        void cmdVelCallback(const geometry_msgs::Twist &command);
        // void cmdDualWheelSteeringCallback(const four_wheel_steering_msgs::FourWheelSteering &command);

        void updateOdometry(const ros::Time &time);

        void updateCommand(const ros::Time &time, const ros::Duration &period);

        void brake();

        /**
         * \brief Sets odometry parameters from the URDF, i.e. the wheel radius and wheel base
         * \param root_nh Root node handle
         * \param front_wheel_name Name of the front wheel joint
         * \param rear_wheel_name  Name of the rear wheel joint
         * \param front_steer_name Name of the front steer joint
         * \param rear_steer_name  Name of the rear steer joint
         */
        bool setOdomParamsFromUrdf(ros::NodeHandle &root_nh,
                                   const std::string front_wheel_name,
                                   const std::string rear_wheel_name,
                                   const std::string front_steer_name,
                                   const std::string rear_steer_name,
                                   bool lookup_wheel_radius,
                                   bool lookup_wheel_base);

        /**
         * \brief Sets the odometry publishing fields
         * \param root_nh Root node handle
         * \param controller_nh Node handle inside the controller namespace
         */
        void setOdomPubFields(ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);
    };

} // namespace dual_wheel_steering_controller
