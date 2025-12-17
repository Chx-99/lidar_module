#include <chrono>
#include <iostream>
#include <vector>
#include <iomanip>

#include "base/data_struct.h"

int main()
{
    Frame<HandShake> handshake("192.168.1.103", 45000, 55000, 65000);
    std::cout << handshake << std::endl;

    Frame<HeartBeat> heartbeat;
    std::cout << heartbeat << std::endl;

    Frame<SetLaserStatus> set_laser_status(1);
    std::cout << set_laser_status << std::endl;

    Frame<SetIMUFrequency> set_imu_frequency(1);
    std::cout << set_imu_frequency << std::endl;

    return 0;
}