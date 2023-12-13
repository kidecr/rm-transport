#include <iostream>
#include <pkg/Gimbal.hpp>

using namespace transport;

Buffer setGimbal(uint8_t mode, float pitch_angle, float yaw_angle, float pitch_speed, float yaw_speed)
{
    Buffer data_to_send(8);

    // angle
    pitch_angle = pitch_angle < 0 ? pitch_angle + 2 * PI : pitch_angle;
    yaw_angle = yaw_angle < 0 ? yaw_angle + 2 * PI : yaw_angle;

    pitch_angle = pitch_angle > 2 *PI ? pitch_angle -= 2 * PI : pitch_angle;
    yaw_angle = yaw_angle > 2 *PI ? yaw_angle -= 2 * PI : yaw_angle;

    // speed
    pitch_speed = std::fabs(pitch_speed) > PI ? (pitch_speed > 0 ? PI : -PI) : pitch_speed;
    yaw_speed = std::fabs(yaw_speed) > PI ? (yaw_speed > 0 ? PI : -PI) : yaw_speed;

    int16_t s_pitch = (2047 * pitch_speed) / (2 * PI);
    int16_t s_yaw = (2047 * yaw_speed) / (2 * PI);

    s_pitch = s_pitch > 2047 ? 2047 : (s_pitch < -2048 ? -2048 : s_pitch);
    s_yaw = s_yaw > 2047 ? 2047 : (s_yaw < -2048 ? -2048 : s_yaw);

    data_to_send[0] = mode;
    data_to_send[1] = (uint8_t)s_yaw;
    data_to_send[2] = (uint8_t)((((uint16_t)s_yaw >> 8) & 0x0f) | (uint8_t)(s_pitch << 4));
    data_to_send[3] = (uint8_t)((uint16_t)s_pitch >> 4);
    data_to_send[4] = (uint8_t)((uint16_t)(yaw_angle / PI * 180 / 360 * 65535));
    data_to_send[5] = (uint8_t)((uint16_t)(yaw_angle / PI * 180 / 360 * 65535) >> 8);
    data_to_send[6] = (uint8_t)((uint16_t)(pitch_angle / PI * 180 / 360 * 65535));
    data_to_send[7] = (uint8_t)((uint16_t)(pitch_angle / PI * 180 / 360 * 65535) >> 8);

    return data_to_send;
    // canBus->sendFrame(data_to_send);
}

wmj::GimbalPose lastpose;
int circlecnt = 0;
auto getGimbal(Buffer data_from_read)
{
    uint16_t timestamp = 0, roll_angle = 0, yaw_angle = 0, pitch_angle = 0;
    static bool poseInit = false;
    timestamp = (uint16_t)data_from_read[0] | (uint16_t)data_from_read[1] << 8;
    yaw_angle = (uint16_t)data_from_read[2] | (uint16_t)data_from_read[3] << 8;
    pitch_angle = (uint16_t)data_from_read[4] | (uint16_t)data_from_read[5] << 8;
    roll_angle = (uint16_t)data_from_read[6] | (uint16_t)data_from_read[7] << 8;

    float fpitch = (float)pitch_angle / 65535.f * 2 * PI;
    float fyaw = (float)yaw_angle / 65535.f * 2 * PI - 2 * PI;
    float froll = (float)roll_angle / 65535.f * 2 * PI;

    wmj::GimbalPose cur_pose{fpitch, fyaw, froll};
    cur_pose.timestamp = timestamp;
    if (!poseInit)
    {
        lastpose = cur_pose;
        poseInit = true;
    }
    if (std::fabs(lastpose.yaw - cur_pose.yaw) > PI)
    {
        if (cur_pose.yaw - lastpose.yaw > 0)
            circlecnt++;
        else
            circlecnt--;
    }
    lastpose = cur_pose;
    cur_pose.yaw = cur_pose.yaw - circlecnt * 2.f * PI;
    cur_pose.pitch = cur_pose.pitch > PI ? cur_pose.pitch - 2 * PI : cur_pose.pitch;

    float data[4] = {cur_pose.pitch, cur_pose.yaw, 0, 0};
    return std::vector<float>(data, data + 4);
}
int main(int argc, char* argv[])
{
    GimbalPackage gimbal_package;
    Buffer buffer;
    gimbal_package.SetGimbalAngle(-1.35, 2.24);
    gimbal_package.SetGimbalSpeed(-2.35, -2.24);
    buffer << gimbal_package;
    std::cout << buffer.toString() << std::endl;
    buffer = setGimbal(0x77, -1.35, 2.24, -2.35, -2.24);
    std::cout << buffer.toString() << std::endl;

    gimbal_package << buffer;
    auto gimbal = gimbal_package.GetGimbalAngle();
    std::cout << "pitch=" << gimbal.pitch << " yaw=" << gimbal.yaw << std::endl;
    auto gimbal_vec = getGimbal(buffer);
    std::cout << "pitch=" << gimbal_vec[0] << " yaw=" << gimbal_vec[1] << std::endl;
    return 0;
}