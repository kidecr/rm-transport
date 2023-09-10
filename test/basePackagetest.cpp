#include "BasePackage.hpp"
#include "PackageManager.hpp"
#include "pkg/Gimbal.hpp"
#include "pkg/Shoot.hpp"

#include "PortManager.hpp"
#include "PortScheduler.hpp"

#ifndef __USE_ROS__

#include "external-interface/cxxInterface.hpp"

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    try
    {
        auto packageManager = std::make_shared<PackageManager>(TRANSPORT_CONFIG_FILE_PATH);
        auto portManager = std::make_shared<PortManager>(TRANSPORT_CONFIG_FILE_PATH, packageManager);
        auto portScheduler = std::make_shared<PortScheduler>(TRANSPORT_CONFIG_FILE_PATH, portManager);

        auto control = std::make_shared<WMJRobotControl>(packageManager);
        portScheduler->run();

        while (1)
        {
            wmj::GimbalPose pose;
            timeval tv;
            gettimeofday(&tv, nullptr);
            pose.yaw = tv.tv_sec % 10, pose.pitch = tv.tv_usec % 10;
            int i = 0;
            std::thread send([&](){
                // std::cout << "---------------先发包-------------------" << std::endl;
                control->setGimbalPose(pose);
                control->switchCoor(true);
                control->setGimbalPose(pose);
                control->ShootSome(++i);
                control->setTime();
                usleep(5);
                control->setGimbalPose(pose);
                control->switchCoor(true);
                control->setGimbalPose(pose);
                control->ShootSome(++i);
                control->setTime();
                usleep(5e3);
            });
            //usleep(1e6);
            std::thread recv([&]() {
                // std::cout << "---------------后收包-------------------" << std::endl;
                control->getGimbalPose();
                // std::cout << control->getGimbalPose().toString() << std::endl;
                control->switchCoor(false);
                control->getGimbalPose();
                // std::cout << control->getGimbalPose().toString() << std::endl;
                // std::cout << control->getShootPackage().toString() << std::endl;
                control->getShootPackage();
                auto time1 = control->getTime();
                TimeTest time2;
                // std::cout << "收包时间：" << time2.getTimeByMicroSec() - time1.getTimeByMicroSec() << "ms " << time1.index << std::endl;
                usleep(5e2);
            });
            send.join();
            recv.join();
        }
    }
    catch (PortException &e)
    {
        std::cout << e.what() << std::endl;
    }

    // node.spin(gimbal);
    return 0;
}

#else

#include "rclcpp/rclcpp.hpp"
#include "external-interface/Shoot.hpp"
#include "external-interface/Gimbal.hpp"
#include "external-interface/Chassis.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        auto packageManager = std::make_shared<PackageManager>("../config/PackageList.yaml");
        auto portManager = std::make_shared<PortManager>("../config/PackageList.yaml", packageManager);
        auto portScheduler = std::make_shared<PortScheduler>("../config/PackageList.yaml", portManager);
        portScheduler->run();

        auto node = std::make_shared<rclcpp::Node>("transport");
        auto shoot_node = std::make_shared<Shoot>(node, packageManager);
        auto gimbal_node = std::make_shared<Gimbal>(node, packageManager);
        auto chassis_node = std::make_shared<Chassis>(node, packageManager);

        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    }
    catch (PortException *e)
    {
        std::cout << e->what() << std::endl;
        delete e;
    }
    catch (...)
    {
        std::cout << "what error?" << std::endl;
    }

    // node.spin(gimbal);
    return 0;
}

#endif // __USE_ROS__
