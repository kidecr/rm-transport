#include "BasePackage.hpp"
#include "PackageManager.hpp"
#include "CanPort.hpp"
#include "GimbalPose.hpp"
#include "Shoot.hpp"

#include "PortManager.hpp"
#include "PortSheduler.hpp"

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
        auto portSheduler = std::make_shared<PortSheduler>(TRANSPORT_CONFIG_FILE_PATH, portManager);

        auto control = std::make_shared<WMJRobotControl>(packageManager);
        portSheduler->run();

        while (1)
        {
            GimbalPose pose;
            timeval tv;
            gettimeofday(&tv, nullptr);
            pose.yaw = tv.tv_sec % 10, pose.pitch = tv.tv_usec % 10;
            int i = 0;
            std::thread send([&](){
                // std::cout << "---------------先发包-------------------" << std::endl;
                control->setGimbalPose(pose);
                control->switchCoor(true);
                control->setGimbalPose(pose);
                control->shootSome(++i);
                control->setTime();
                usleep(5);
                control->setGimbalPose(pose);
                control->switchCoor(true);
                control->setGimbalPose(pose);
                control->shootSome(++i);
                control->setTime();
                usleep(1e3);
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
                usleep(1e3);
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
#include "external-interface/Gimbal.hpp"
#include "external-interface/Gimbal2.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        auto packageManager = std::make_shared<PackageManager>("../config/PackageList.yaml");
        auto portManager = std::make_shared<PortManager>("../config/PackageList.yaml", packageManager);
        auto portSheduler = std::make_shared<PortSheduler>("../config/PackageList.yaml", portManager);
        portSheduler->run();

        auto node = std::make_shared<rclcpp::Node>("transport");
        auto gimbal_node = std::make_shared<Gimbal>(node, packageManager);
        auto gimbal2_node = std::make_shared<Gimbal2>(node, packageManager);

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
