#include <cmath>
// #include <four_wheel_steering_controller/four_wheel_steering_controller.h>
#include <dual_wheel_steering_controller/dual_wheel_steering_controller.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf/transform_datatypes.h>
#include <urdf_geometry_parser/urdf_geometry_parser.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include "glog/logging.h"
// #include "dual_wheel_steering_controller/global_defination.h"
/*
 * \brief Check if the link is modeled as a cylinder
 * \param link Link
 * \return true if the link is modeled as a Cylinder; false otherwise
 */
static bool isCylinder(const urdf::LinkConstSharedPtr &link)
{
    if (!link)
    {
        ROS_ERROR("Link pointer is null.");
        return false;
    }

    if (!link->collision)
    {
        ROS_ERROR_STREAM("Link " << link->name << " does not have collision description. Add collision description for link to urdf.");
        return false;
    }

    if (!link->collision->geometry)
    {
        ROS_ERROR_STREAM("Link " << link->name << " does not have collision geometry description. Add collision geometry description for link to urdf.");
        return false;
    }

    if (link->collision->geometry->type != urdf::Geometry::CYLINDER)
    {
        ROS_ERROR_STREAM("Link " << link->name << " does not have cylinder geometry");
        return false;
    }

    return true;
}

/*
 * \brief Get the wheel radius
 * \param [in]  wheel_link   Wheel link
 * \param [out] wheel_radius Wheel radius [m]
 * \return true if the wheel radius was found; false other wise
 */
static bool getWheelRadius(const urdf::LinkConstSharedPtr &wheel_link, double &wheel_radius)
{
    if (!isCylinder(wheel_link))
    {
        ROS_ERROR_STREAM("Wheel link " << wheel_link->name << " is NOT modeled as a cylinder!");
        return false;
    }

    wheel_radius = (static_cast<urdf::Cylinder *>(wheel_link->collision->geometry.get()))->radius;
    return true;
}

namespace dual_wheel_steering_controller
{

    DualWheelSteeringController::DualWheelSteeringController()
        : open_loop_(false),
          command_struct_twist_(),
          wheel_radius_(0.0),
          wheel_base_(0.0),
          wheel_radius_multiplier_(1.0),
          wheel_base_multiplier_(1.0),
          cmd_vel_timeout_(0.5),
          allow_multiple_cmd_vel_publishers_(true),
          base_frame_id_("base_link"),
          odom_frame_id_("odom"),
          enable_odom_tf_(true),
          enable_twist_cmd_(false),
          debug_period_time_(1.0),
          last_front_steer_pos_cmd_(0.0),
          last_rear_steer_pos_cmd_(0.0)
        //   steer_delay_factor_coefficient_(5.0)
    {
    }

    bool DualWheelSteeringController::init(hardware_interface::RobotHW *robot_hw,
                                           ros::NodeHandle &root_nh,
                                           ros::NodeHandle &controller_nh)
    {
        /* get multiple types of hardware_interface */
        hardware_interface::VelocityJointInterface *const vel_joint_hw = robot_hw->get<hardware_interface::VelocityJointInterface>();
        hardware_interface::PositionJointInterface *const pos_joint_hw = robot_hw->get<hardware_interface::PositionJointInterface>();

        const std::string complete_ns = controller_nh.getNamespace();
        std::size_t id = complete_ns.find_last_of("/");
        name_ = complete_ns.substr(id + 1);

        /* Get joint names from the parameter server */
        std::string front_wheel_name = "front_wheel_joint";
        controller_nh.param("front_wheel_joint", front_wheel_name, front_wheel_name);
        LOG(INFO) << "front_wheel_joint = " << front_wheel_name;

        std::string rear_wheel_name = "rear_wheel_joint";
        controller_nh.param("rear_wheel_joint", rear_wheel_name, rear_wheel_name);
        LOG(INFO) << "rear_wheel_joint = " << rear_wheel_name;

        std::string front_steer_name = "front_steer_joint";
        controller_nh.param("front_steer_joint", front_steer_name, front_steer_name);
        LOG(INFO) << "front_steer_joint = " << front_steer_name;

        std::string rear_steer_name = "rear_steer_joint";
        controller_nh.param("rear_steer_joint", rear_steer_name, rear_steer_name);
        LOG(INFO) << "rear_steer_joint = " << rear_steer_name;

        /* Odometry related */
        double publish_rate;
        controller_nh.param("publish_rate", publish_rate, 50.0);
        ROS_INFO_STREAM_NAMED(name_, "Controller state will be published at " << publish_rate << "Hz.");
        publish_period_ = ros::Duration(1.0 / publish_rate);

        controller_nh.param("open_loop", open_loop_, open_loop_);

        controller_nh.param("wheel_radius_multiplier", wheel_radius_multiplier_, wheel_radius_multiplier_);
        LOG(INFO) << "Wheel radius will be multiplied by " << wheel_radius_multiplier_ << ".";

        controller_nh.param("wheel_base_multiplier", wheel_base_multiplier_, wheel_base_multiplier_);
        LOG(INFO) << "Wheel base will be multiplied by " << wheel_base_multiplier_ << ".";

        int velocity_rolling_window_size = 10;
        controller_nh.param("velocity_rolling_window_size", velocity_rolling_window_size, velocity_rolling_window_size);
        ROS_INFO_STREAM_NAMED(name_, "Velocity rolling window size of " << velocity_rolling_window_size << ".");

        odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size);

        /* Twist command related */ 
        controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, cmd_vel_timeout_);
        ROS_INFO_STREAM_NAMED(name_, "Velocity commands will be considered old if they are older than " << cmd_vel_timeout_ << "s.");

        controller_nh.param("allow_multiple_cmd_vel_publishers", allow_multiple_cmd_vel_publishers_, allow_multiple_cmd_vel_publishers_);
        ROS_INFO_STREAM_NAMED(name_, "Allow mutiple cmd_vel publishers is "
                                         << (allow_multiple_cmd_vel_publishers_ ? "enabled" : "disabled"));

        controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
        LOG(INFO) << "Base frame_id set to " << base_frame_id_;

        controller_nh.param("odom_frame_id", odom_frame_id_, odom_frame_id_);
        LOG(INFO) << "Odometry frame_id set to " << odom_frame_id_;

        controller_nh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);
        ROS_INFO_STREAM_NAMED(name_, "Publishing to tf is " << (enable_odom_tf_ ? "enabled" : "disabled"));

        controller_nh.param("debug_period_time", debug_period_time_, debug_period_time_);
        ROS_INFO_STREAM_NAMED(name_, "Throttled DEBUG output will print a message at most once per " << debug_period_time_ << " sec.");

        /* Speedlimter related */
        controller_nh.param("linear/x/has_velocity_limits", limiter_vx_.has_velocity_limits, limiter_vx_.has_velocity_limits);
        controller_nh.param("linear/x/has_acceleration_limits", limiter_vx_.has_acceleration_limits, limiter_vx_.has_acceleration_limits);
        controller_nh.param("linear/x/max_velocity", limiter_vx_.max_velocity, limiter_vx_.max_velocity);
        controller_nh.param("linear/x/min_velocity", limiter_vx_.min_velocity, -limiter_vx_.max_velocity);
        controller_nh.param("linear/x/max_acceleration", limiter_vx_.max_acceleration, limiter_vx_.max_acceleration);
        controller_nh.param("linear/x/min_acceleration", limiter_vx_.min_acceleration, -limiter_vx_.max_acceleration);

        controller_nh.param("linear/y/has_acceleration_limits", limiter_vy_.has_acceleration_limits, limiter_vy_.has_acceleration_limits);
        controller_nh.param("linear/y/max_velocity_limits", limiter_vy_.has_velocity_limits, limiter_vy_.has_velocity_limits);
        controller_nh.param("linear/y/has_velocity", limiter_vy_.max_velocity, limiter_vy_.max_velocity);
        controller_nh.param("linear/y/min_velocity", limiter_vy_.min_velocity, -limiter_vy_.max_velocity);
        controller_nh.param("linear/y/max_acceleration", limiter_vy_.max_acceleration, limiter_vy_.max_acceleration);
        controller_nh.param("linear/y/min_acceleration", limiter_vy_.min_acceleration, -limiter_vy_.max_acceleration);

        controller_nh.param("angular/z/has_velocity_limits", limiter_w_.has_velocity_limits, limiter_w_.has_velocity_limits);
        controller_nh.param("angular/z/has_acceleration_limits", limiter_w_.has_acceleration_limits, limiter_w_.has_acceleration_limits);
        controller_nh.param("angular/z/max_velocity", limiter_w_.max_velocity, limiter_w_.max_velocity);
        controller_nh.param("angular/z/min_velocity", limiter_w_.min_velocity, -limiter_w_.max_velocity);
        controller_nh.param("angular/z/max_acceleration", limiter_w_.max_acceleration, limiter_w_.max_acceleration);
        controller_nh.param("angular/z/min_acceleration", limiter_w_.min_acceleration, -limiter_w_.max_acceleration);

        // If either parameter is not available, we need to look up the value in the URDF
        bool lookup_wheel_radius = !controller_nh.getParam("wheel_radius", wheel_radius_);
        bool lookup_wheel_base = !controller_nh.getParam("wheel_base", wheel_base_);

        if (!setOdomParamsFromUrdf(root_nh,
                                   front_wheel_name,
                                   rear_wheel_name,
                                   front_steer_name,
                                   rear_steer_name,
                                   lookup_wheel_radius,
                                   lookup_wheel_base))
        {
            return false;
        }


        // Regardless of how we got the separation and radius, use them to set the odometry parameters
        wb_ = wheel_base_multiplier_ * wheel_base_;
        wr_ = wheel_radius_multiplier_ * wheel_radius_;
        odometry_.setWheelParams(wr_, wb_);
        ROS_INFO_STREAM_NAMED(name_,
                              "Odometry params : "
                                  << " wheel radius " << wr_
                                  << ", wheel base " << wb_);

        setOdomPubFields(root_nh, controller_nh);

        // Get the joint object to use in the realtime loop
        ROS_INFO_STREAM_NAMED(name_,
                              "Adding the front wheel with joint name: " << front_wheel_name);
        front_wheel_joint_ = vel_joint_hw->getHandle(front_wheel_name); // throws on failure

        ROS_INFO_STREAM_NAMED(name_,
                              "Adding the rear wheel with joint name: " << rear_wheel_name);
        rear_wheel_joint_ = vel_joint_hw->getHandle(rear_wheel_name); // throws on failure

        ROS_INFO_STREAM_NAMED(name_,
                              "Adding the front steer with joint name: " << front_steer_name);
        front_steer_joint_ = pos_joint_hw->getHandle(front_steer_name); // throws on failure

        ROS_INFO_STREAM_NAMED(name_,
                              "Adding the rear steer with joint name: " << rear_steer_name);
        rear_steer_joint_ = pos_joint_hw->getHandle(rear_steer_name); // throws on failure

        sub_command_ = controller_nh.subscribe("cmd_vel", 1, &DualWheelSteeringController::cmdVelCallback, this);
        ROS_INFO_STREAM_NAMED(name_, "Finished dual wheel Steering controller initialization");

        return true;
    }

    void DualWheelSteeringController::update(const ros::Time &time, const ros::Duration &period)
    {
        if (!isStopBefore_)
        {
            updateOdometry(time);
            updateCommand(time, period);
        }
        isStopBefore_ = false;
    }

    void DualWheelSteeringController::updateOdometry(const ros::Time &time)
    {
        const double steer_front_pos = front_steer_joint_.getPosition();
        const double steer_rear_pos = rear_steer_joint_.getPosition();
        const double drive_front_pos = front_wheel_joint_.getPosition();
        const double drive_rear_pos = rear_wheel_joint_.getPosition();

        if (std::isnan(steer_front_pos) || std::isnan(steer_rear_pos) || std::isnan(drive_front_pos) || std::isnan(drive_rear_pos))
            return;
        
        odometry_.update(steer_front_pos, steer_rear_pos, drive_front_pos, drive_rear_pos, time);

        // realtime pub & tf
        if (last_state_publish_time_ + publish_period_ < time)
        {
            last_state_publish_time_ += publish_period_;
            const geometry_msgs::Quaternion orientation(tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

            if (odom_pub_->trylock())
            {
                odom_pub_->msg_.header.stamp = time;
                odom_pub_->msg_.pose.pose.position.x = odometry_.getX();
                odom_pub_->msg_.pose.pose.position.y = odometry_.getY();
                odom_pub_->msg_.pose.pose.orientation = orientation;
                odom_pub_->msg_.twist.twist.linear.x = odometry_.getLinearX();
                odom_pub_->msg_.twist.twist.linear.y = odometry_.getLinearY();
                odom_pub_->msg_.twist.twist.angular.z = odometry_.getAngular();
                odom_pub_->unlockAndPublish();
            }
            if (enable_odom_tf_ && tf_odom_pub_->trylock())
            {
                geometry_msgs::TransformStamped &odom_frame = tf_odom_pub_->msg_.transforms[0];
                odom_frame.header.stamp = time;
                odom_frame.transform.translation.x = odometry_.getX();
                odom_frame.transform.translation.y = odometry_.getY();
                odom_frame.transform.rotation = orientation;
                tf_odom_pub_->unlockAndPublish();
            }
        }
    }

    void DualWheelSteeringController::updateCommand(const ros::Time &time, const ros::Duration &period)
    {
        // Retreive current velocity command and time step:
        CommandTwist cur_cmd_twist = *(command_twist_.readFromRT());

        // Brake if cmd_vel has timeout:
        const double dt = (time - cur_cmd_twist.stamp).toSec();
        if (dt > cmd_vel_timeout_)
        {
            cur_cmd_twist.lin_x = 0.0;
            cur_cmd_twist.lin_y = 0.0;
            cur_cmd_twist.ang = 0.0;
            LOG(INFO) << "Controller timeout.";
        }

        VLOG(10) << " 原始速度命令   : "
                 << " x: " << std::fixed << std::setprecision(2) << cur_cmd_twist.lin_x << " m/s , "
                 << " y: " << cur_cmd_twist.lin_y << " m/s , "
                 << "Ang: " << cur_cmd_twist.ang << " rad/s \n";

        

        /* Read cmd_vel */
        double v_x = cur_cmd_twist.lin_x;
        double v_y = cur_cmd_twist.lin_y;
        double w = cur_cmd_twist.ang;

        /* Speedlimit to smooth cmd_vel */
        const double cmd_dt(period.toSec());
        limiter_vx_.limit(v_x , last0_vx_cmd_ , last1_vx_cmd_ , cmd_dt);
        limiter_vy_.limit(v_y , last0_vy_cmd_ , last1_vy_cmd_ , cmd_dt);
        limiter_w_.limit(w , last0_w_cmd_ , last1_w_cmd_ , cmd_dt);
        
        /* Calculate joint command with kinematics considered */
        double v1x = v_x;
        double v1y = v_y + w * wb_ / 2;
        double v2x = v_x;
        double v2y = v_y - w * wb_ / 2;

        double front_steer_pos_cmd = 0.0; // unit: rad
        double rear_steer_pos_cmd = 0.0;  // unit: rad

        double front_wheel_vel = 0.0;     // unit: m/s
        double front_wheel_ang_vel = 0.0; // unit: rad/s

        double rear_wheel_vel = 0.0;     // unit: m/s
        double rear_wheel_ang_vel = 0.0; // unit: rad/s

        front_wheel_vel = std::sqrt(v1x * v1x + v1y * v1y);
        rear_wheel_vel = std::sqrt(v2x * v2x + v2y * v2y);
        front_steer_pos_cmd = std::atan2(v1y, v1x);
        rear_steer_pos_cmd = std::atan2(v2y, v2x);

        if (front_steer_pos_cmd > M_PI / 2 + 0.5 /*30 degree*/)
        {
            front_steer_pos_cmd -= M_PI;
            front_wheel_vel *= -1;
        }
        else if (front_steer_pos_cmd < -M_PI / 2 - 0.5 /*30 degree*/)
        {
            front_steer_pos_cmd += M_PI;
            front_wheel_vel *= -1;
        }

        if (rear_steer_pos_cmd > M_PI / 2 + 0.5 /*30 degree*/)
        {
            rear_steer_pos_cmd -= M_PI;
            rear_wheel_vel *= -1;
        }
        else if (rear_steer_pos_cmd < -M_PI / 2 - 0.5 /*30 degree*/)
        {
            rear_steer_pos_cmd += M_PI;
            rear_wheel_vel *= -1;
        }
        front_wheel_ang_vel = front_wheel_vel / wr_; // omega = linear_vel / radius
        rear_wheel_ang_vel = rear_wheel_vel / wr_;   // omega = linear_vel / radius
        

        VLOG(9) << " Front Steer : "
                << std::fixed << std::setprecision(2) << front_steer_pos_cmd / M_PI * 180.0f << " degree , "
                << front_steer_pos_cmd << " rad \n";
        VLOG(9) << " Rear  Steer : "
                << std::fixed << std::setprecision(2) << rear_steer_pos_cmd / M_PI * 180.0f << " degree , "
                << rear_steer_pos_cmd << " rad \n";
        VLOG(8) << " Front Wheel : "
                << std::fixed << std::setprecision(2) << front_wheel_vel << " m/s , "
                << front_wheel_ang_vel << " rad/s \n";
        VLOG(8) << " Rear  Wheel : "
                << std::fixed << std::setprecision(2) << rear_wheel_vel << " m/s , "
                << rear_wheel_ang_vel << " rad/s \n";

        /* Update last command */
        last1_vx_cmd_ = last0_vx_cmd_;
        last0_vx_cmd_ = v_x;
        last1_vy_cmd_ = last0_vy_cmd_;
        last0_vy_cmd_ = v_y;
        last1_w_cmd_ = last0_w_cmd_;
        last0_w_cmd_ = w;
        last_front_steer_pos_cmd_ = front_steer_pos_cmd;
        last_rear_steer_pos_cmd_ = rear_steer_pos_cmd;

        /* Set command to joint */
        front_wheel_joint_.setCommand(front_wheel_ang_vel);
        rear_wheel_joint_.setCommand(rear_wheel_ang_vel);
        front_steer_joint_.setCommand(front_steer_pos_cmd);
        rear_steer_joint_.setCommand(rear_steer_pos_cmd);

        VLOG(10) << " limit 速度命令 : "
                 << " front wheel cmd: " << std::fixed << std::setprecision(2) << front_wheel_ang_vel << " rad/s , "
                 << " rear wheel cmd: " << rear_wheel_ang_vel << " rad/s \n";

    }

    void DualWheelSteeringController::brake()
    {
        std::cout << "  DualWheelSteeringController::brake  " << std::endl;
        const double vel = 0.0;
        front_wheel_joint_.setCommand(vel);
        rear_wheel_joint_.setCommand(vel);
        const double pos = 0.0;
        front_steer_joint_.setCommand(pos);
        rear_steer_joint_.setCommand(pos);
    }

    void DualWheelSteeringController::starting(const ros::Time &time)
    {
        brake();
        last0_vx_cmd_ = 0.0;
        last1_vx_cmd_ = 0.0;
        last0_vy_cmd_ = 0.0;
        last1_vy_cmd_ = 0.0;
        last0_w_cmd_  = 0.0;
        last1_w_cmd_  = 0.0;
        std::cout << "  DualWheelSteeringController::Starting  " << std::endl;
        last_state_publish_time_ = time;
        odometry_.init(time);
    }

    void DualWheelSteeringController::stopping(const ros::Time & /*time*/)
    {
        brake();
        std::cout << "  DualWheelSteeringController::Stopping  " << std::endl;
        isStopBefore_ = true;
    }

    // TODO 更新 joints 初始化方式之後，就不需要getWheelNames()函式的功能了，但是程式碼以後還是可以參考
    bool DualWheelSteeringController::getWheelNames(ros::NodeHandle &controller_nh,
                                                    const std::string &wheel_param,
                                                    std::vector<std::string> &wheel_names)
    {
        // TODO 可以參考 XmlRpc::XmlRpcValue 的用法
        XmlRpc::XmlRpcValue wheel_list;
        if (!controller_nh.getParam(wheel_param, wheel_list))
        {
            LOG(ERROR) << "Couldn't retrieve wheel param '" << wheel_param << "'.";
            return false;
        }
        // TODO 可以參考 XmlRpc::XmlRpcValue::TypeArray 的用法
        if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            if (wheel_list.size() == 0)
            {
                ROS_ERROR_STREAM_NAMED(name_, "Wheel param '" << wheel_param << "' is an empty list");
                return false;
            }

            for (int i = 0; i < wheel_list.size(); ++i)
            {
                if (wheel_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
                {
                    ROS_ERROR_STREAM_NAMED(name_, "Wheel param '" << wheel_param << "' #" << i << " isn't a string.");
                    return false;
                }
            }

            wheel_names.resize(wheel_list.size());
            for (int i = 0; i < wheel_list.size(); ++i)
            {
                wheel_names[i] = static_cast<std::string>(wheel_list[i]);
            }
        }
        else if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeString)
        {
            wheel_names.push_back(wheel_list);
        }
        else
        {
            ROS_ERROR_STREAM_NAMED(name_,
                                   "Wheel param '" << wheel_param << "' is neither a list of strings nor a string.");
            return false;
        }
        return true;
    }

    void DualWheelSteeringController::cmdVelCallback(const geometry_msgs::Twist &command)
    {
        if (isRunning())
        {
            // check that we don't have multiple publishers on the command topic
            if (!allow_multiple_cmd_vel_publishers_ && sub_command_.getNumPublishers() > 1)
            {
                ROS_ERROR_STREAM_THROTTLE_NAMED(1.0, name_, "Detected " << sub_command_.getNumPublishers() << " publishers. Only 1 publisher is allowed. Going to brake.");
                brake();
                return;
            }
            if (std::isnan(command.angular.z) || std::isnan(command.linear.x))
            {
                ROS_WARN("Received NaN in geometry_msgs::Twist. Ignoring command.");
                return;
            }
            command_struct_twist_.ang = command.angular.z;
            command_struct_twist_.lin_x = command.linear.x;
            command_struct_twist_.lin_y = command.linear.y;
            command_struct_twist_.stamp = ros::Time::now();
            command_twist_.writeFromNonRT(command_struct_twist_);
        }
        else
        {
            std::cout << " Can't accept new commands. " << std::endl;

            ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
        }
    }

    bool DualWheelSteeringController::setOdomParamsFromUrdf(ros::NodeHandle &root_nh,
                                                            const std::string front_wheel_name,
                                                            const std::string rear_wheel_name,
                                                            const std::string front_steer_name,
                                                            const std::string rear_steer_name,
                                                            bool lookup_wheel_radius,
                                                            bool lookup_wheel_base)
    {

        if (!(lookup_wheel_radius || lookup_wheel_base))
        {
            // Short-circuit in case we don't need to look up anything, so we don't have to parse the URDF
            return true;
        }

        // Parse robot description
        const std::string model_param_name = "robot_description";
        bool res = root_nh.hasParam(model_param_name);
        std::string robot_model_str = "";
        if (!res || !root_nh.getParam(model_param_name, robot_model_str))
        {
            LOG(ERROR) << "Robot descripion couldn't be retrieved from param server.";
            return false;
        }

        urdf::ModelInterfaceSharedPtr model(urdf::parseURDF(robot_model_str));

        urdf::JointConstSharedPtr front_wheel_joint(model->getJoint(front_wheel_name));
        urdf::JointConstSharedPtr rear_wheel_joint(model->getJoint(rear_wheel_name));

        urdf_geometry_parser::UrdfGeometryParser uvk(root_nh, base_frame_id_);

        if (lookup_wheel_radius)
        {
            // Get wheel radius
            if (!uvk.getJointRadius(front_wheel_name, wheel_radius_))
            {
                LOG(ERROR) << "Couldn't retrieve " << front_wheel_name << " wheel radius";
                return false;
            }
            ROS_INFO_STREAM("Retrieved wheel_radius: " << wheel_radius_);
        }

        if (lookup_wheel_base)
        {
            // Get wheel base
            if (!front_wheel_joint)
            {
                LOG(ERROR) << front_wheel_name << " couldn't be retrieved from model description";
                return false;
            }

            if (!rear_wheel_joint)
            {
                LOG(ERROR) << rear_wheel_name << " couldn't be retrieved from model description";
                return false;
            }

            if (!uvk.getDistanceBetweenJoints(front_wheel_name, rear_wheel_name, wheel_base_))
            {
                LOG(ERROR) << "Couldn't retrieve wheel base";
                return false;
            }
            LOG(INFO) << "Retrieved wheel_base: " << wheel_base_;
        }

        return true;
    }

    void DualWheelSteeringController::setOdomPubFields(ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
    {
        // Get and check params for covariances
        XmlRpc::XmlRpcValue pose_cov_list;
        controller_nh.getParam("pose_covariance_diagonal", pose_cov_list);
        ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(pose_cov_list.size() == 6);
        for (int i = 0; i < pose_cov_list.size(); ++i)
            ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

        XmlRpc::XmlRpcValue twist_cov_list;
        controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
        ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(twist_cov_list.size() == 6);
        for (int i = 0; i < twist_cov_list.size(); ++i)
            ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

        // Setup odometry realtime publisher + odom message constant fields
        odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
        odom_pub_->msg_.header.frame_id = odom_frame_id_;
        odom_pub_->msg_.child_frame_id = base_frame_id_;
        odom_pub_->msg_.pose.pose.position.z = 0;
        odom_pub_->msg_.pose.covariance = {
            static_cast<double>(pose_cov_list[0]), 0., 0., 0., 0., 0.,
            0., static_cast<double>(pose_cov_list[1]), 0., 0., 0., 0.,
            0., 0., static_cast<double>(pose_cov_list[2]), 0., 0., 0.,
            0., 0., 0., static_cast<double>(pose_cov_list[3]), 0., 0.,
            0., 0., 0., 0., static_cast<double>(pose_cov_list[4]), 0.,
            0., 0., 0., 0., 0., static_cast<double>(pose_cov_list[5])};
        odom_pub_->msg_.twist.twist.linear.y = 0;
        odom_pub_->msg_.twist.twist.linear.z = 0;
        odom_pub_->msg_.twist.twist.angular.x = 0;
        odom_pub_->msg_.twist.twist.angular.y = 0;
        odom_pub_->msg_.twist.covariance = {
            static_cast<double>(twist_cov_list[0]), 0., 0., 0., 0., 0.,
            0., static_cast<double>(twist_cov_list[1]), 0., 0., 0., 0.,
            0., 0., static_cast<double>(twist_cov_list[2]), 0., 0., 0.,
            0., 0., 0., static_cast<double>(twist_cov_list[3]), 0., 0.,
            0., 0., 0., 0., static_cast<double>(twist_cov_list[4]), 0.,
            0., 0., 0., 0., 0., static_cast<double>(twist_cov_list[5])};
        tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
        tf_odom_pub_->msg_.transforms.resize(1);
        tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
        tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
        tf_odom_pub_->msg_.transforms[0].header.frame_id = odom_frame_id_;
    }

} // namespace dual_wheel_steering_controller

PLUGINLIB_EXPORT_CLASS(dual_wheel_steering_controller::DualWheelSteeringController, controller_interface::ControllerBase)
