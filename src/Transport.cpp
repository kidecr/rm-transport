#include "PackageManager.hpp"
#include "PortManager.hpp"
#include "PortSheduler.hpp"

#include "GimbalPose.hpp"
#include "Shoot.hpp"

#ifdef __USE_ROS__
#include "rclcpp/rclcpp.hpp"
#include "external-interface/Gimbal.hpp"
#include "external-interface/Gimbal2.hpp"


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        auto packageManager = std::make_shared<PackageManager>(TRANSPORT_CONFIG_FILE_PATH);
        auto portManager = std::make_shared<PortManager>(TRANSPORT_CONFIG_FILE_PATH, packageManager);
        auto portSheduler = std::make_shared<PortSheduler>(TRANSPORT_CONFIG_FILE_PATH, portManager);
        portSheduler->run();

        auto node = std::make_shared<rclcpp::Node>("transport");
        auto gimbal_node = std::make_shared<Gimbal>(node, packageManager);
        auto gimbal2_node = std::make_shared<Gimbal2>(node, packageManager);

        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    }
    catch (PortException e)
    {
        std::cout << e.what() << std::endl;
    }

    // node.spin(gimbal);
    return 0;
}

#else

#include "external-interface/cxxInterface.hpp"

int main(int argc, char* argv[]) 
{
    try{
        auto packageManager = std::make_shared<PackageManager>(TRANSPORT_CONFIG_FILE_PATH);
        auto portManager = std::make_shared<PortManager>(TRANSPORT_CONFIG_FILE_PATH, packageManager);
        auto portSheduler = std::make_shared<PortSheduler>(TRANSPORT_CONFIG_FILE_PATH, portManager);

        auto control = std::make_shared<WMJRobotControl>(packageManager);
        portSheduler->run();

        int cnt = 0;
        while (1)
        {
            control->setTime();
            usleep(1e6);
            std::cout << ++cnt << std::endl;
        }
    }
    catch (PortException e)
    {
        std::cout << e.what() << std::endl;
    }

}
#endif // __USE_ROS__