
#include "base/dataStruct.h"

int main()
{
  old_version::Frame<old_version::setIMUFre> set_imu_frequency(1);
  for (const auto &byte : set_imu_frequency.serialize())
  {
    std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
  }
  std::cout << std::dec << std::endl;

  return 0;
}