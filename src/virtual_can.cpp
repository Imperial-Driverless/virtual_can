#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <id_msgs/msg/vcu_drive_command.hpp>
#include <id_msgs/msg/vcu_drive_feedback.hpp>

#include "../FS-AI_API/FS-AI_API/can.h"
#include "../FS-AI_API/FS-AI_API/fs-ai_api.h"

using std::placeholders::_1;
using std::placeholders::_2;

typedef union can_data_t {
	volatile uint8_t ubytes[8];
	volatile int8_t sbytes[8];
	volatile uint16_t uwords[4];
	volatile int16_t swords[4];
	volatile uint32_t ulongs[2];
	volatile int32_t slongs[2];
	volatile float floats[2];
} can_data_t;


class SimulateCAN : public rclcpp::Node
{
public:
    SimulateCAN() : Node("virtual_can")
    {
        handshake = fs_ai_api_handshake_send_bit_e::HANDSHAKE_SEND_BIT_OFF;
        
        // declare parameters
        can_debug = declare_parameter<int>("can_debug", can_debug);
        can_simulate = declare_parameter<int>("can_simulate", can_simulate);
        can_interface = declare_parameter<std::string>("can_interface", can_interface);
        loop_rate = declare_parameter<int>("loop_rate", loop_rate);
        if (declare_parameter<bool>("debug_logging", false))
        {
            get_logger().set_level(rclcpp::Logger::Level::Debug);
        }

        if (can_debug) 
        {
            RCLCPP_INFO(get_logger(), "Starting FS-AI API in debug mode");
        }

        // declare subscriptions (simulator info)
        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 1, std::bind(&SimulateCAN::imu_callback, this, _1));
        vcu_drive_feedback_sub = this->create_subscription<id_msgs::msg::VCUDriveFeedback>("/vcu_drive_feedback", 1, std::bind(&SimulateCAN::vcu_drive_feedback_callback, this, _1));

        // declare publishers (simulator commands)
        cmd_pub = this->create_publisher<id_msgs::msg::VCUDriveCommand>("/vcu_drive_cmd", 1);

        // declare services


        // setup interface
        fs_ai_api_init_reverse(const_cast<char *>(can_interface.c_str()), can_debug, can_simulate);

        // setup ROS times
        std::chrono::duration<float> rate(1 / static_cast<double>(loop_rate));
        timer = this->create_wall_timer(rate, std::bind(&SimulateCAN::loop, this));
    }

    void loop()
    {
        // get fresh data from AI
        fs_ai_api_ai2vcu_get_data(&ai2vcu_data);

        // Log new data (in one string so log messages don't get separated)
        std::string msg_recv =
            "--- Read CAN data ---\n"
            "MISSION:     " + std::to_string(ai2vcu_data.AI2VCU_MISSION_STATUS) + "\n" +
            "DIRECTION:   " + std::to_string(ai2vcu_data.AI2VCU_DIRECTION_REQUEST) + "\n" +
            "ESTOP:       " + std::to_string(ai2vcu_data.AI2VCU_ESTOP_REQUEST) + "\n" +
            "HANDSHAKE:   " + std::to_string(ai2vcu_data.AI2VCU_HANDSHAKE_SEND_BIT) + "\n" +
            "STEER:       " + std::to_string(ai2vcu_data.AI2VCU_STEER_ANGLE_REQUEST_deg) + "\n" +
            "AXLE SPEED:  " + std::to_string(ai2vcu_data.AI2VCU_AXLE_SPEED_REQUEST_rpm) + "\n" +
            "AXLE TORQUE: " + std::to_string(ai2vcu_data.AI2VCU_AXLE_TORQUE_REQUEST_Nm) + "\n" +
            "BRAKE REQ:   " + std::to_string(ai2vcu_data.AI2VCU_BRAKE_PRESS_REQUEST_pct) + "\n";
        RCLCPP_DEBUG(get_logger(), "%s", msg_recv.c_str());

        // publish all received data
        id_msgs::msg::VCUDriveCommand cmd_msg = makeCommandMessage(ai2vcu_data);
        cmd_pub->publish(cmd_msg);

        // assign data to be sent only when it's ready not to overcrowd CAN bus
        // VCU2AI
        if (vcu2ai_data_ready)
        {
            vcu2ai_data_ready = false;
            vcu2ai_data.VCU2AI_HANDSHAKE_RECEIVE_BIT = VCU2AI_HANDSHAKE_RECEIVE_BIT;
            vcu2ai_data.VCU2AI_RES_GO_SIGNAL = VCU2AI_RES_GO_SIGNAL;
            vcu2ai_data.VCU2AI_AS_STATE = VCU2AI_AS_STATE;
            vcu2ai_data.VCU2AI_AMI_STATE = VCU2AI_AMI_STATE;
            vcu2ai_data.VCU2AI_STEER_ANGLE_deg = steering_angle_deg;
            vcu2ai_data.VCU2AI_BRAKE_PRESS_F_pct = brake_pressure_f_pct;
            vcu2ai_data.VCU2AI_BRAKE_PRESS_R_pct = brake_pressure_r_pct;
            vcu2ai_data.VCU2AI_FL_WHEEL_SPEED_rpm = fl_wheel_speed_rpm;
            vcu2ai_data.VCU2AI_FR_WHEEL_SPEED_rpm = fr_wheel_speed_rpm;
            vcu2ai_data.VCU2AI_RL_WHEEL_SPEED_rpm = rl_wheel_speed_rpm;
            vcu2ai_data.VCU2AI_RR_WHEEL_SPEED_rpm = rr_wheel_speed_rpm;
            vcu2ai_data.VCU2AI_FL_PULSE_COUNT = fl_pulse_count;
            vcu2ai_data.VCU2AI_FR_PULSE_COUNT = fr_pulse_count;
            vcu2ai_data.VCU2AI_RL_PULSE_COUNT = rl_pulse_count;
            vcu2ai_data.VCU2AI_RR_PULSE_COUNT = rr_pulse_count;
            fs_ai_api_vcu2ai_set_data(&vcu2ai_data);
        }

        // IMU
        if (imu_data_ready)
        {
            imu_data_ready = false;
            imu_data.IMU_Acceleration_X_mG = x_linear_acceleration;
            imu_data.IMU_Acceleration_Y_mG = y_linear_acceleration;
            imu_data.IMU_Acceleration_Z_mG = z_linear_acceleration;
            imu_data.IMU_Temperature_degC = temperature_degC;
            imu_data.IMU_VerticalAxis = vertical_axis;
            imu_data.IMU_Orientation = orientation;
            imu_data.IMU_MagneticField_X_uT = magnetic_field_x_uT;
            imu_data.IMU_MagneticField_Y_uT = magnetic_field_y_uT;
            imu_data.IMU_MagneticField_Z_uT = magnetic_field_z_uT;
            imu_data.IMU_Rotation_X_degps = x_angular_velocity;
            imu_data.IMU_Rotation_Y_degps = y_angular_velocity;
            imu_data.IMU_Rotation_Z_degps = z_angular_velocity;
            fs_ai_api_imu_set_data(&imu_data);
        }

        // Log sent data (in one string so log messages don't get separated)
        std::string msg_send =
            "--- Send CAN data ---\n"
            "AS STATE:    " + std::to_string(vcu2ai_data.VCU2AI_AS_STATE) + "\n" +
            "AMI STATE:   " + std::to_string(vcu2ai_data.VCU2AI_AMI_STATE) + "\n" +
            "FL RPM:      " + std::to_string(vcu2ai_data.VCU2AI_FL_WHEEL_SPEED_rpm) + "\n" +
            "FR RPM:      " + std::to_string(vcu2ai_data.VCU2AI_FR_WHEEL_SPEED_rpm) + "\n" +
            "RL RPM:      " + std::to_string(vcu2ai_data.VCU2AI_RL_WHEEL_SPEED_rpm) + "\n" +
            "RR RPM:      " + std::to_string(vcu2ai_data.VCU2AI_RR_WHEEL_SPEED_rpm) + "\n" +
            "STEER ANGLE: " + std::to_string(vcu2ai_data.VCU2AI_STEER_ANGLE_deg) + "\n";
        RCLCPP_DEBUG(get_logger(), "%s", msg_send.c_str());
    }

private:
    id_msgs::msg::VCUDriveCommand makeCommandMessage(const fs_ai_api_ai2vcu_struct &data)
    {
        id_msgs::msg::VCUDriveCommand cmd_msg;

        cmd_msg.motor_torque_nm = data.AI2VCU_AXLE_TORQUE_REQUEST_Nm;
        cmd_msg.steering_angle_rad = data.AI2VCU_STEER_ANGLE_REQUEST_deg * M_PI / 180.0f;
        cmd_msg.brake_pct = data.AI2VCU_BRAKE_PRESS_REQUEST_pct / 100.0f;
        cmd_msg.rpm_limit = data.AI2VCU_AXLE_SPEED_REQUEST_rpm;

        return cmd_msg;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {   
        imu_data_ready = true;

        // acceleration data is in mG
        // Actual acceleremoter requires multiplication by 3.91 so we emulate that here
        x_linear_acceleration = (int16_t) (msg->linear_acceleration.x * 1000.0f / 9.8f);
        y_linear_acceleration = (int16_t) (msg->linear_acceleration.y * 1000.0f / 9.8f);
        z_linear_acceleration = (int16_t) (msg->linear_acceleration.z * 1000.0f / 9.8f);
        
        // ros_can expects degrees
        x_angular_velocity = (msg->angular_velocity.x / M_PI) * 180.0;
        y_angular_velocity = (msg->angular_velocity.y / M_PI) * 180.0;
        z_angular_velocity = (msg->angular_velocity.z / M_PI) * 180.0;
    }

    void vcu_drive_feedback_callback(const id_msgs::msg::VCUDriveFeedback::SharedPtr msg)
    {
        vcu2ai_data_ready = true;
        
        // prepare message
        fl_wheel_speed_rpm = msg->fl_wheel_speed_rpm;
        fr_wheel_speed_rpm = msg->fr_wheel_speed_rpm;
        rl_wheel_speed_rpm = msg->rl_wheel_speed_rpm;
        rr_wheel_speed_rpm = msg->rr_wheel_speed_rpm;

        steering_angle_deg = ((-msg->steering_angle_rad) * 180.0f) / M_PI;
    }

    fs_ai_api_handshake_send_bit_e get_handshake()
    {
        // switch handshake bit and return it
        if (handshake == fs_ai_api_handshake_send_bit_e::HANDSHAKE_SEND_BIT_OFF)
        {
            handshake = fs_ai_api_handshake_send_bit_e::HANDSHAKE_SEND_BIT_ON;
            return fs_ai_api_handshake_send_bit_e::HANDSHAKE_SEND_BIT_ON;
        }
        else
        {
            handshake = fs_ai_api_handshake_send_bit_e::HANDSHAKE_SEND_BIT_OFF;
            return fs_ai_api_handshake_send_bit_e::HANDSHAKE_SEND_BIT_OFF;
        }
    }

    bool check_hanshake(fs_ai_api_handshake_send_bit_e other)
    {
        return handshake == other;
    }
    
    fs_ai_api_handshake_send_bit_e handshake;
    // parameters and their defaults
    int can_debug = 0;
    int can_simulate = 0;
    std::string can_interface = "can0";
    int loop_rate = 100;
    
    // subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<id_msgs::msg::VCUDriveFeedback>::SharedPtr vcu_drive_feedback_sub;
    
    // publishers
    rclcpp::Publisher<id_msgs::msg::VCUDriveCommand>::SharedPtr cmd_pub;
    
    // declare services

    // ROS timer
    rclcpp::TimerBase::SharedPtr timer;

    // FS-AI API structs to store data
    struct fs_ai_api_vcu2ai_struct vcu2ai_data; // wheel speed data to send to ros_can
    struct fs_ai_api_imu_struct imu_data; // IMU data to send to ros_can
    struct fs_ai_api_ai2vcu_struct ai2vcu_data; // AI data sent to car

    // Flags determining if data is ready to send
    bool vcu2ai_data_ready = false;
    bool imu_data_ready = false;

    // Variables 
    double fl_wheel_speed_rpm = 0.0;
    double fr_wheel_speed_rpm = 0.0;
    double rl_wheel_speed_rpm = 0.0;
    double rr_wheel_speed_rpm = 0.0;
    double steering_angle_deg = 0.0;

    int16_t x_linear_acceleration = 0;
    int16_t y_linear_acceleration = 0;
    int16_t z_linear_acceleration = 0;

    double x_angular_velocity = 0.0;
    double y_angular_velocity = 0.0;
    double z_angular_velocity = 0.0;

    fs_ai_api_handshake_receive_bit_e   VCU2AI_HANDSHAKE_RECEIVE_BIT = HANDSHAKE_RECEIVE_BIT_OFF;
    fs_ai_api_res_go_signal_bit_e		VCU2AI_RES_GO_SIGNAL = RES_GO_SIGNAL_NO_GO;
    fs_ai_api_as_state_e				VCU2AI_AS_STATE = AS_OFF;
    fs_ai_api_ami_state_e				VCU2AI_AMI_STATE = AMI_NOT_SELECTED;

    // variables not implemented in simulator
    float brake_pressure_f_pct = 0.0f;
    float brake_pressure_r_pct = 0.0f;
    float fl_pulse_count = 0.0f;
    float fr_pulse_count = 0.0f;
    float rl_pulse_count = 0.0f;
    float rr_pulse_count = 0.0f;
    float temperature_degC = 0.0f;
    uint8_t vertical_axis = 0;
    uint8_t orientation = 0;
    int16_t magnetic_field_x_uT = 0;
    int16_t magnetic_field_y_uT = 0;
    int16_t magnetic_field_z_uT = 0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulateCAN>());
  rclcpp::shutdown();
  return 0;
}
