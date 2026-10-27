#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

using namespace std::chrono_literals;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl()
        : Node("offboard_control")
    {
        const auto px4_publisher_qos =
            rclcpp::QoS(rclcpp::KeepLast(10))
                .best_effort()
                .transient_local();
        const auto px4_subscriber_qos =
            rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        offboard_control_mode_pub_ =
            create_publisher<px4_msgs::msg::OffboardControlMode>(
                "/fmu/in/offboard_control_mode", px4_publisher_qos);
        trajectory_setpoint_pub_ =
            create_publisher<px4_msgs::msg::TrajectorySetpoint>(
                "/fmu/in/trajectory_setpoint", px4_publisher_qos);
        vehicle_command_pub_ =
            create_publisher<px4_msgs::msg::VehicleCommand>(
                "/fmu/in/vehicle_command", px4_publisher_qos);

        local_position_sub_ =
            create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                "/fmu/out/vehicle_local_position",
                px4_subscriber_qos,
                [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
                {
                    if (msg->timestamp == 0 ||
                        msg->timestamp <= last_local_position_timestamp_) {
                        return;
                    }

                    last_local_position_timestamp_ = msg->timestamp;

                    if (!msg->xy_valid || !msg->z_valid) {
                        return;
                    }

                    current_x_ = msg->x;
                    current_y_ = msg->y;
                    current_z_ = msg->z;

                    if (!position_received_) {
                        // Start at the current position and circle around a
                        // center one radius to the west of it.
                        circle_center_x_ = current_x_ - kCircleRadius;
                        circle_center_y_ = current_y_;
                        target_x_ = current_x_;
                        target_y_ = current_y_;
                        target_z_ = current_z_ - 5.0F;
                        position_received_ = true;

                        RCLCPP_INFO(
                            get_logger(),
                            "Position setpoint: x=%.2f y=%.2f z=%.2f",
                            target_x_, target_y_, target_z_);
                    }
                });

        attitude_sub_ =
            create_subscription<px4_msgs::msg::VehicleAttitude>(
                "/fmu/out/vehicle_attitude",
                px4_subscriber_qos,
                [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
                {
                    if (msg->timestamp == 0 ||
                        msg->timestamp <= last_attitude_timestamp_) {
                        return;
                    }

                    last_attitude_timestamp_ = msg->timestamp;
                    attitude_received_ = true;
                });

        vehicle_status_sub_ =
            create_subscription<px4_msgs::msg::VehicleStatus>(
                "/fmu/out/vehicle_status_v1",
                px4_subscriber_qos,
                [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg)
                {
                    nav_state_ = msg->nav_state;
                    arming_state_ = msg->arming_state;
                    failsafe_ = msg->failsafe;
                    preflight_checks_pass_ = msg->pre_flight_checks_pass;
                    valid_nav_states_mask_ = msg->valid_nav_states_mask;
                    status_received_ = true;
                });

        vehicle_command_ack_sub_ =
            create_subscription<px4_msgs::msg::VehicleCommandAck>(
                "/fmu/out/vehicle_command_ack",
                px4_subscriber_qos,
                [this](const px4_msgs::msg::VehicleCommandAck::SharedPtr msg)
                {
                    if (msg->command !=
                            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE &&
                        msg->command !=
                            px4_msgs::msg::VehicleCommand::
                                VEHICLE_CMD_COMPONENT_ARM_DISARM) {
                        return;
                    }

                    if (msg->result ==
                        px4_msgs::msg::VehicleCommandAck::
                            VEHICLE_CMD_RESULT_ACCEPTED) {
                        RCLCPP_INFO(
                            get_logger(),
                            "PX4 accepted command %u.",
                            msg->command);
                    } else {
                        RCLCPP_ERROR(
                            get_logger(),
                            "PX4 rejected command %u with result %u.",
                            msg->command, msg->result);
                    }
                });

        timer_ = create_wall_timer(
            20ms,
            std::bind(&OffboardControl::timer_callback, this));

        RCLCPP_INFO(
            get_logger(),
            "Offboard control started. Waiting for valid PX4 local position.");
    }

private:
    void timer_callback()
    {
        publish_offboard_control_mode();

        if (!position_received_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Waiting for valid /fmu/out/vehicle_local_position.");
            return;
        }

        publish_trajectory_setpoint();

        if (setpoint_counter_ < 50) {
            ++setpoint_counter_;
            return;
        }

        if (!status_received_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Waiting for /fmu/out/vehicle_status_v1.");
            return;
        }

        if (!offboard_command_sent_ &&
            !offboard_ready()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Waiting for PX4 readiness: status=%s, attitude=%s, "
                "position=%s, preflight=%s, failsafe=%s.",
                status_received_ ? "yes" : "no",
                attitude_received_ ? "yes" : "no",
                position_received_ ? "yes" : "no",
                preflight_checks_pass_ ? "pass" : "fail",
                failsafe_ ? "active" : "clear");
            return;
        }

        if (nav_state_ !=
                px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD &&
            !offboard_command_sent_) {
            RCLCPP_INFO(get_logger(), "Requesting PX4 Offboard mode.");
            publish_vehicle_command(
                px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                1.0F,
                6.0F);
            offboard_command_sent_ = true;
            return;
        }

        if (nav_state_ ==
                px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD &&
            arming_state_ ==
                px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED &&
            !arm_command_sent_) {
            if (!offboard_ready()) {
            return;
            }

            RCLCPP_INFO(get_logger(), "Requesting PX4 arm.");
            publish_vehicle_command(
                px4_msgs::msg::VehicleCommand::
                    VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0F,
                0.0F);
            arm_command_sent_ = true;
        }
    }

    bool offboard_ready() const
    {
        constexpr uint8_t kMaxNavState =
            px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MAX;
        constexpr uint8_t kDisarmed =
            px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;

        const bool nav_state_valid =
            nav_state_ < kMaxNavState &&
            (valid_nav_states_mask_ & (1u << nav_state_)) != 0;

        return status_received_ &&
               attitude_received_ &&
               position_received_ &&
               last_attitude_timestamp_ > 0 &&
               last_local_position_timestamp_ > 0 &&
               nav_state_valid &&
               arming_state_ == kDisarmed &&
               preflight_checks_pass_ &&
               !failsafe_;
    }

    uint64_t timestamp_us()
    {
        return static_cast<uint64_t>(
            get_clock()->now().nanoseconds() / 1000);
    }

    void publish_offboard_control_mode()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = timestamp_us();
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.thrust_and_torque = false;
        msg.direct_actuator = false;
        offboard_control_mode_pub_->publish(msg);
    }

    void publish_trajectory_setpoint()
    {
        if (!circle_started_) {
            target_x_ = circle_center_x_ + kCircleRadius;
            target_y_ = circle_center_y_;

            const bool altitude_reached =
                std::fabs(current_z_ - target_z_) <= kTakeoffAltitudeTolerance;
            const bool armed =
                arming_state_ ==
                px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;

            if (armed && altitude_reached) {
                circle_start_time_us_ = timestamp_us();
                circle_started_ = true;
                RCLCPP_INFO(
                    get_logger(),
                    "Takeoff altitude reached. Starting circular trajectory.");
            }
        }

        double angle = 0.0;
        if (circle_started_) {
            const double elapsed_seconds =
                static_cast<double>(timestamp_us() - circle_start_time_us_) /
                1e6;
            angle = kCircleAngularSpeed * elapsed_seconds;

            target_x_ =
                circle_center_x_ + kCircleRadius * std::cos(angle);
            target_y_ =
                circle_center_y_ + kCircleRadius * std::sin(angle);
        }

        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = timestamp_us();
        msg.position = {target_x_, target_y_, target_z_};
        msg.velocity = {NAN, NAN, NAN};
        msg.acceleration = {NAN, NAN, NAN};
        // Point the vehicle along the tangent of the circle.
        msg.yaw = circle_started_
                      ? static_cast<float>(angle + kPi / 2.0)
                      : NAN;
        msg.yawspeed = NAN;
        trajectory_setpoint_pub_->publish(msg);
    }

    void publish_vehicle_command(
        uint16_t command,
        float param1,
        float param2)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = timestamp_us();
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        vehicle_command_pub_->publish(msg);
    }

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
        offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
        trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr
        vehicle_command_pub_;

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
        local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr
        attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr
        vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr
        vehicle_command_ack_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    uint32_t setpoint_counter_{0};
    uint8_t nav_state_{
        px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MAX};
    uint8_t arming_state_{
        px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED};
    uint32_t valid_nav_states_mask_{0};

    bool position_received_{false};
    bool attitude_received_{false};
    bool status_received_{false};
    bool failsafe_{false};
    bool preflight_checks_pass_{false};
    bool offboard_command_sent_{false};
    bool arm_command_sent_{false};
    bool circle_started_{false};
    uint64_t last_attitude_timestamp_{0};
    uint64_t last_local_position_timestamp_{0};

    float current_x_{0.0F};
    float current_y_{0.0F};
    float current_z_{0.0F};
    float target_x_{0.0F};
    float target_y_{0.0F};
    float target_z_{0.0F};
    float circle_center_x_{0.0F};
    float circle_center_y_{0.0F};
    uint64_t circle_start_time_us_{0};

    static constexpr float kCircleRadius{5.0F};
    static constexpr float kCircleAngularSpeed{0.2F};
    static constexpr float kTakeoffAltitudeTolerance{0.2F};
    static constexpr double kPi{3.14159265358979323846};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}
