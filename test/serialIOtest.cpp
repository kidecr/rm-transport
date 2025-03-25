#ifdef ENABLE_SERIAL_PORT
#include "PackageManager.hpp"
#include "PortManager.hpp"
#include "PortScheduler.hpp"

#include "impls/logger.hpp"
#include "impls/Config.hpp"

#include "pkg/Gimbal.hpp"
#include "pkg/Shoot.hpp"

using namespace transport;

int main(int argc, char* argv[])
{
    LOGINIT("serialIOtest")
    auto config = std::make_shared<config::Config>();
    auto packageManager = std::make_shared<PackageManager>(config);
    auto portManager = std::make_shared<PortManager>(config, packageManager);
    auto portScheduler = std::make_shared<PortScheduler>(config, portManager);
    portScheduler->run();

    double angle = 0.3;
    uint8_t shoot_cnt = 0;
    while(angle < 1e4 && transport::ok()){
        GimbalPackage gimbal_package;
        gimbal_package.SetGimbal_YawSpeed_PitchAngle(1.2, angle);
        packageManager->send(JUDGE_ID, gimbal_package);
        ShootPackage shoot_package;
        shoot_package.shootSome(shoot_cnt);
        packageManager->send(SHOOT_ID, shoot_package);
        std::this_thread::sleep_for(std::chrono::microseconds(5e5));
        
        auto gimbal_recv = packageManager->recv<GimbalPackage>(JUDGE_ID);
        auto shoot_recv = packageManager->recv<ShootPackage>(SHOOT_ID);
        std::cout << "send: " << gimbal_package.toString() << std::endl;
        std::cout << "recv: " << gimbal_recv.toString() << std::endl;
        std::cout << "send: " << shoot_package.toString() << std::endl;
        std::cout << "recv: " << shoot_recv.toString() << std::endl; 
        angle = angle + 1.0;
        shoot_cnt = shoot_cnt + 1;
        // LOGINFO("recv: " + gimbal_recv.toString());
    }
}
#endif // ENABLE_SERIAL_PORT