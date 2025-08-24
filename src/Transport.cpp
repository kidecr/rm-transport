#include "PackageManager.hpp"
#include "PortManager.hpp"
#include "PortScheduler.hpp"

#include "impls/logger.hpp"
#include "impls/Config.hpp"

#include <chrono>

#ifdef __USE_ROS2__

#include "rclcpp/rclcpp.hpp"
#include "external-interface/Shoot.hpp"
#include "external-interface/Gimbal.hpp"

using namespace transport;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<rclcpp::Node>("transport");
#if defined __USE_ROS2__ && defined __USE_ROS_LOG__
        LOGINIT(node);
#else
        LOGINIT();
#endif // defined __USE_ROS2__ && defined __USE_ROS_LOG__
        // auto packageManager = std::make_shared<PackageManager>(TRANSPORT_CONFIG_FILE_PATH);
        // auto portManager = std::make_shared<PortManager>(TRANSPORT_CONFIG_FILE_PATH, packageManager);
        // auto portScheduler = std::make_shared<PortScheduler>(TRANSPORT_CONFIG_FILE_PATH, portManager);

        auto config = std::make_shared<config::Config>();
        auto packageManager = std::make_shared<PackageManager>(config);
        auto portManager = std::make_shared<PortManager>(config, packageManager);
        auto portScheduler = std::make_shared<PortScheduler>(config, portManager);
        portScheduler->run();

        auto shoot_node = std::make_shared<Shoot>(node, packageManager);
        auto gimbal_node = std::make_shared<Gimbal>(node, packageManager);

        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    }
    catch (PortException e)
    {
        std::cout << "[" << __FILE__ << ":" << __LINE__ << " catch PortExpection]: ";
        std::cout << e.what() << std::endl;
    }
    transport::shutdown();
    rclcpp::shutdown();
    // node.spin(gimbal);
    return 0;
}

#else

#include "external-interface/cxxInterface.hpp"
#include "external-interface/BluetoothInterface.hpp"

using namespace transport;

int main(int argc, char* argv[]) 
{
    // LOGINIT("transport", "./log");

    try{
#ifdef ENABLE_WIN_BLUETOOTH
        auto blue = std::make_shared<BluetoothInterface>(TRANSPORT_CONFIG_XML_FILE_PATH);


        int cnt = 0;
        while (transport::ok())
        {
            auto imu = blue->recvWTIMU();
            std::cout << imu.toString() << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            std::cout << ++cnt << std::endl;
        } 
#else
        auto config = std::make_shared<config::Config>();
        auto packageManager = std::make_shared<PackageManager>(config);
        auto portManager = std::make_shared<PortManager>(config, packageManager);
        auto portScheduler = std::make_shared<PortScheduler>(config, portManager);

        auto control = std::make_shared<WMJRobotControl>(packageManager);
        portScheduler->run();

        while (transport::ok())
        {
            control->setTime();
            std::this_thread::sleep_for(std::chrono::seconds(1));
            auto time = control->getTime();
            std::cout << time.toString() << std::endl;
        } 
#endif // ENABLE_WIN_BLUETOOTH
    }
    catch (PortException e)
    {
        std::cout << "[" << __FILE__ << ":" << __LINE__ << " catch PortExpection]: ";
        std::cout << e.what() << std::endl;
    }

}
#endif // __USE_ROS2__