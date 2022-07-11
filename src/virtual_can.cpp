#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <imperial_driverless_interfaces/msg/vcu_drive_feedback.hpp>

#include "../FS-AI_API/FS-AI_API/can.h"
#include "../FS-AI_API/FS-AI_API/fs-ai_api.h"

using std::placeholders::_1;

typedef union can_data_t {
	volatile uint8_t ubytes[8];
	volatile int8_t sbytes[8];
	volatile uint16_t uwords[4];
	volatile int16_t swords[4];
	volatile uint32_t ulongs[2];
	volatile int32_t slongs[2];
	volatile float floats[2];
} can_data_t;

#define VCU2AI_WHEEL_SPEEDS_ID	            0x525
#define PCAN_GPS_BMC_ACCELERATION_ID		0X600
#define PCAN_GPS_L3GD20_ROTATION_A_ID		0X610
#define PCAN_GPS_L3GD20_ROTATION_B_ID		0X611

static struct can_frame VCU2AI_Wheel_speeds = {VCU2AI_WHEEL_SPEEDS_ID, 8};
static struct can_frame PCAN_GPS_BMC_Acceleration = {PCAN_GPS_BMC_ACCELERATION_ID, 8};
static struct can_frame PCAN_GPS_L3GD20_Rotation_A = {PCAN_GPS_L3GD20_ROTATION_A_ID, 8};
static struct can_frame PCAN_GPS_L3GD20_Rotation_B = {PCAN_GPS_L3GD20_ROTATION_B_ID, 8};


class SimulateCAN : public rclcpp::Node
{
public:
    SimulateCAN() : Node("virtual_can")
    {
        handshake = fs_ai_api_handshake_send_bit_e::HANDSHAKE_SEND_BIT_OFF;
        // declare parameters

        // declare subscriptions (simulator info)
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 1, std::bind(&SimulateCAN::imu_callback, this, _1));
        vcu_drive_feedback_sub_ = this->create_subscription<imperial_driverless_interfaces::msg::VCUDriveFeedback>("/vcu_drive_feedback", 1, std::bind(&SimulateCAN::vcu_drive_feedback_callback, this, _1));

        // declare services
    }

private:

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // IMU data is transmitted over 3 GPS frames
        // PCAN_GPS_BMC_Acceleration, PCAN_GPS_L3GD20_Rotation_A and PCAN_GPS_L3GD20_Rotation_B
        can_data_t* temp;

        temp = (can_data_t*)&PCAN_GPS_BMC_Acceleration.data[0];
        
        // acceleration data is in mG
        temp->swords[0] = (int16_t) (msg->linear_acceleration.x * 1000.0f / 9.81f);
        temp->swords[1] = (int16_t) (msg->linear_acceleration.y * 1000.0f / 9.81f);
        temp->swords[2] = (int16_t) (msg->linear_acceleration.z * 1000.0f / 9.81f);
        
        // ros_can expects degrees
        temp = (can_data_t*)&PCAN_GPS_L3GD20_Rotation_A.data[0];
        temp->floats[0] = (msg->angular_velocity.x / M_PI) * 180.0;
        temp->floats[0] = (msg->angular_velocity.y / M_PI) * 180.0;

        temp = (can_data_t*)&PCAN_GPS_L3GD20_Rotation_B.data[0];
        temp->floats[0] = (msg->angular_velocity.z / M_PI) * 180.0;

        can_send(&PCAN_GPS_BMC_Acceleration);
        can_send(&PCAN_GPS_L3GD20_Rotation_A);
        can_send(&PCAN_GPS_L3GD20_Rotation_B);
    }

    void vcu_drive_feedback_callback(const imperial_driverless_interfaces::msg::VCUDriveFeedback::SharedPtr msg)
    {
        // prepare can frame

        VCU2AI_Wheel_speeds.data[0] = (uint8_t)((int)(msg->fl_wheel_speed_rpm) & 0x00FF);
        VCU2AI_Wheel_speeds.data[1] = (uint8_t)(((int)(msg->fl_wheel_speed_rpm) & 0xFF00) >> 8);

        VCU2AI_Wheel_speeds.data[2] = (uint8_t)((int)(msg->fr_wheel_speed_rpm) & 0x00FF);
        VCU2AI_Wheel_speeds.data[3] = (uint8_t)(((int)(msg->fr_wheel_speed_rpm) & 0xFF00) >> 8);

        VCU2AI_Wheel_speeds.data[4] = (uint8_t)((int)(msg->rl_wheel_speed_rpm) & 0x00FF);
        VCU2AI_Wheel_speeds.data[5] = (uint8_t)(((int)(msg->rl_wheel_speed_rpm) & 0xFF00) >> 8);

        VCU2AI_Wheel_speeds.data[6] = (uint8_t)((int)(msg->rr_wheel_speed_rpm) & 0x00FF);
        VCU2AI_Wheel_speeds.data[7] = (uint8_t)(((int)(msg->rr_wheel_speed_rpm) & 0xFF00) >> 8);

        can_send(&VCU2AI_Wheel_speeds);
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
    // subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<imperial_driverless_interfaces::msg::VCUDriveFeedback>::SharedPtr vcu_drive_feedback_sub_;
    // declare services
};