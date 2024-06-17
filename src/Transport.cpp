#include "PackageManager.hpp"
#include "PortManager.hpp"
#include "PortScheduler.hpp"

#include "impls/logger.hpp"
#include "impls/BackGround.hpp"

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

        auto background = std::make_shared<background::BackGround>();
        auto packageManager = std::make_shared<PackageManager>(background);
        auto portManager = std::make_shared<PortManager>(background, packageManager);
        auto portScheduler = std::make_shared<PortScheduler>(background, portManager);
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

using namespace transport;

int main(int argc, char* argv[]) 
{
    LOGINIT("transport", "./log");

    try{
        auto packageManager = std::make_shared<PackageManager>(TRANSPORT_CONFIG_FILE_PATH);
        auto portManager = std::make_shared<PortManager>(TRANSPORT_CONFIG_FILE_PATH, packageManager);
        auto portScheduler = std::make_shared<PortScheduler>(TRANSPORT_CONFIG_FILE_PATH, portManager);

        auto control = std::make_shared<WMJRobotControl>(packageManager);
        portScheduler->run();

        int cnt = 0;
        while (transport::ok())
        {
            control->setTime();
            usleep(1e6);
            std::cout << ++cnt << std::endl;
        }
    }
    catch (PortException e)
    {
        std::cout << "[" << __FILE__ << ":" << __LINE__ << " catch PortExpection]: ";
        std::cout << e.what() << std::endl;
    }

}
#endif // __USE_ROS2__