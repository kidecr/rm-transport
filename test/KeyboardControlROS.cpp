#include <iostream>
#include <chrono>

#include <unistd.h>
#include <termio.h>
#include <sys/unistd.h>
#include <sys/time.h>
#include <sys/statfs.h>

// #define __USE_ROS2__ 
#include "PackageManager.hpp"
#include "BaseROSInterface.hpp"

#include "base_interfaces/msg/gimbal_pose.hpp"
#include "base_interfaces/msg/shooter.hpp"

#ifdef __USE_ROS2__

void monitorKeyboard(char *key)
{
    while (true)
    {
        termios new_settings;
        termios stored_settings;
        tcgetattr(0, &stored_settings);
        new_settings = stored_settings;
        new_settings.c_lflag &= (~ICANON);
        new_settings.c_cc[VTIME] = 0;
        tcgetattr(0, &stored_settings);
        new_settings.c_cc[VMIN] = 1;
        tcsetattr(0, TCSANOW, &new_settings);

        *key = getchar();
        tcsetattr(0, TCSANOW, &stored_settings);
    }
}

char c = 0;

void callback(){};

class KeyBoardControl : public transport::BaseROSInterface
{
public:
    KeyBoardControl(const rclcpp::Node::SharedPtr& node) : BaseROSInterface(node, transport::PackageManager::SharedPtr(nullptr))
    {
        // gimbal 控制
        addPublisher<base_interfaces::msg::GimbalPose>("SetGimbalAngle", 1s, 10, std::bind(&callback), this);
        addPublisher<base_interfaces::msg::GimbalPose>("SetGimbal_YawSpeed_PitchAngle", 1s, 10, std::bind(&callback), this);
        // shoot 控制
        addPublisher<base_interfaces::msg::GimbalPose>("ShootSome", 1s, 10, std::bind(&callback), this);
        // gimbal 接收
        addSubscription<base_interfaces::msg::GimbalPose>("GetGimbalAngle", 10, std::bind(&KeyBoardControl::getGimbalAngleCallback, this, _1), this);
        addSubscription<base_interfaces::msg::GimbalPose>("GetGimbalSpeed", 10, std::bind(&KeyBoardControl::getGimbalSpeedCallback, this, _1), this);
        std::thread t(&KeyBoardControl::keyProcess, this);
        t.detach();
    }

    void keyProcess()
    {
        base_interfaces::msg::GimbalPose gimbal;
        base_interfaces::msg::Shooter shoot;
        bool gimbal_control_mode = true;
        bool quit = false;
        while (!quit && rclcpp::ok())
        {
            switch (c)
            {
            //云台
            case 'w':
                gimbal.pitch -= 0.1;
                break;
            case 's':
                gimbal.pitch += 0.1;
                break;
            case 'a':
                gimbal.yaw += 0.1;
                break;
            case 'd':
                gimbal.yaw -= 0.1;
                break;
            case 'v':
                gimbal_control_mode = !gimbal_control_mode;
            //发射
            case '1':
                shoot.bulletnum = 1;    // 打一发
                break;
            case '2':
                shoot.bulletnum = 3;    // 打3发
                break;
            case '3':
                shoot.bulletnum = -1;   // 连发
                break;
            case '4':
                shoot.bulletnum = -2;   // 停止
                break;
            //解使能
            case 'q':
                quit = true;
            case 'c':
                gimbal.pitch = 0;
                gimbal.yaw = 0;
                shoot.bulletnum = -2;
                break;
            default:
                break;
            }
            
            c = 0;

            if(gimbal_control_mode) // SetGimbalAngle
                publisher<base_interfaces::msg::GimbalPose>(0)->publish(gimbal);
            else                    // SetGimbal_YawSpeed_PitchAngle
                publisher<base_interfaces::msg::GimbalPose>(1)->publish(gimbal);  

            // ShootSome
            publisher<base_interfaces::msg::Shooter>(2)->publish(shoot);     
            shoot.bulletnum = 0;

            printf("******************************************************\n");
            printf("send: pitch: %5f yaw: %5f\n", gimbal.pitch, gimbal.yaw);
            printf("recv: pitch: %5f yaw: %5f\n", m_gimbal_angle.pitch, m_gimbal_angle.yaw);
            printf("current gimbal control mode: %s\n", gimbal_control_mode ? "angle" : "yawSpeed_pitchAngle");
        
            usleep(1e5);
        }
        exit(0);
    }

    base_interfaces::msg::GimbalPose m_gimbal_angle;
    base_interfaces::msg::GimbalPose m_gimbal_speed;

    void getGimbalAngleCallback(const base_interfaces::msg::GimbalPose::SharedPtr msg)
    {
        m_gimbal_angle.pitch = msg->pitch;
        m_gimbal_angle.yaw = msg->yaw;
    }
    void getGimbalSpeedCallback(const base_interfaces::msg::GimbalPose::SharedPtr msg)
    {
        m_gimbal_speed.pitch = msg->pitch;
        m_gimbal_speed.yaw = msg->yaw;
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    std::thread t(&monitorKeyboard, &c);
    auto node = std::make_shared<rclcpp::Node>("KeyBoardControl");
    auto key_board_control = std::make_shared<KeyBoardControl>(node);
    t.join();
    return 0;
}

#endif // __USE_ROS2__