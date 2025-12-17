//
// Created by mwt on 25-1-20.
//

#ifndef DATASTRUCT_H
#define DATASTRUCT_H
#include <bitset> // 二进制支持
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <unordered_map>
#include <vector>

#include "crcCalculator.h"

#define OPEN 0x01
#define CLOSE 0x00
#define CMD_TYPE_INDEX 4
#define CMD_SET_INDEX 9
#define CMD_ID_INDEX 10
// 切分数据域的宏
#define GET_DATA_FIELD(data, size) \
  ([&]() -> std::vector<uint8_t> {                              \
    if ((size) < 13)                                            \
      throw std::invalid_argument(                              \
          "Size must be at least 13 to perform slicing.");      \
    return std::vector<uint8_t>((data) + 9, (data) + (size)-4); }())
namespace old_version
{
  struct TupleHash
  {
    template <typename T1, typename T2, typename T3>
    std::size_t operator()(const std::tuple<T1, T2, T3> &tuple) const
    {
      auto hash1 = std::hash<T1>{}(std::get<0>(tuple));
      auto hash2 = std::hash<T2>{}(std::get<1>(tuple));
      auto hash3 = std::hash<T3>{}(std::get<2>(tuple));
      return hash1 ^ (hash2 << 1) ^ (hash3 << 2); // 混合哈希值
    }
  };

  // cmd-type:cmd_set:cmd_id:type
  inline std::unordered_map<std::tuple<unsigned int, unsigned int, unsigned int>,
                            std::string, TupleHash>
      TAG_MAP{{{0x02, 0x00, 0x00}, "boardcast"},
              {{0x01, 0x00, 0x01}, "handShakeACK"},
              {{0x01, 0x00, 0x03}, "heartBeatACK"},
              {{0x01, 0x00, 0x04}, "switchACK"},
              {{0x01, 0x00, 0x07}, "errorMsg"},
              {{0x01, 0x00, 0x0A}, "restartACK"},
              {{0x01, 0x01, 0x08}, "setIMUFreACK"}};

#pragma pack(push, 1) // 确保结构体的字节对齐为1字节
  template <typename frameDataType>
  class Frame
  {
  public:
    uint8_t sof = 0xaa;        // 起始字节，固定为 0xAA
    uint8_t version = 0x01;    // 协议版本
    uint16_t length;           // 数据帧长度
    uint8_t cmd_type = 0x00;   // 命令类型
    uint16_t seq_num = 0x00;   // 数据帧序列号
    uint16_t crc_16;           // 包头校验码
    std::vector<uint8_t> data; // 数据域（动态长度）
    frameDataType frame_data_bean;
    uint32_t crc_32; // 整个数据帧校验码
  public:
    // 可变参构造函数
    template <typename... Args>
    explicit Frame(Args &&...args) : frame_data_bean((args)...)
    {
      data = frame_data_bean();
    }

    // 无参的时候使用
    explicit Frame() { data = frame_data_bean(); }

    // 序列化为字节数组
    std::vector<uint8_t> serialize()
    {
      std::vector<uint8_t> frame;
      // 固定部分
      frame.push_back(sof);     // sof
      frame.push_back(version); // version

      // 预留 length，占位
      frame.insert(frame.end(), sizeof(length), 0);

      frame.push_back(cmd_type); // cmd_type

      // 添加 seq_num
      frame.insert(frame.end(), reinterpret_cast<const uint8_t *>(&seq_num),
                   reinterpret_cast<const uint8_t *>(&seq_num) + sizeof(seq_num));

      // 预留 crc_16，占位
      frame.insert(frame.end(), sizeof(crc_16), 0);

      // 添加 data 实际内容，而不是 vector 的元信息
      frame.insert(frame.end(), data.begin(), data.end());

      // 计算 length 并填充
      length = static_cast<uint16_t>(frame.size() + sizeof(crc_32));
      std::memcpy(&frame[2], &length, sizeof(length));

      // 计算并填充 crc_16
      crc_16 = crc16(reinterpret_cast<const uint8_t *>(frame.data()), 7);
      std::memcpy(&frame[7], &crc_16, sizeof(crc_16));

      // 计算并填充 crc_32
      crc_32 =
          crc32(reinterpret_cast<const uint8_t *>(frame.data()), frame.size());
      frame.insert(frame.end(), reinterpret_cast<const uint8_t *>(&crc_32),
                   reinterpret_cast<const uint8_t *>(&crc_32) + sizeof(crc_32));
      return frame;
    }

    std::vector<uint8_t> operator()() { return serialize(); }

    template <typename T>
    inline std::ostream &operator<<(const Frame<T> &frame)
    {
      auto serialized = frame();

      // 原始数据输出
      //  for (auto &byte : serialized) {
      //      os << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
      //      << static_cast<int>(byte) << " 原始数据:" <<
      //      static_cast<int>(byte)<<std::endl;;
      //  }

      size_t offset = 0;

      // sof
      std::cout << "sof: 0x" << std::hex << std::uppercase << std::setw(2)
                << std::setfill('0') << static_cast<int>(serialized[offset++])
                << "\n";

      // version
      std::cout << "version: 0x" << std::hex << std::uppercase << std::setw(2)
                << std::setfill('0') << static_cast<int>(serialized[offset++])
                << "\n";

      // length
      uint16_t length;
      std::memcpy(&length, &serialized[offset], sizeof(length));
      offset += sizeof(length);
      std::cout << "length: 0x" << std::hex << std::uppercase << length << "\n";

      // cmd_type
      std::cout << "cmd_type: 0x" << std::hex << std::uppercase << std::setw(2)
                << std::setfill('0') << static_cast<int>(serialized[offset++])
                << "\n";

      // seq_num
      uint16_t seq_num;
      std::memcpy(&seq_num, &serialized[offset], sizeof(seq_num));
      offset += sizeof(seq_num);
      std::cout << "seq_num: 0x" << std::hex << std::uppercase << seq_num << "\n";

      // crc_16
      uint16_t crc_16;
      std::memcpy(&crc_16, &serialized[offset], sizeof(crc_16));
      offset += sizeof(crc_16);
      std::cout << "crc_16: 0x" << std::hex << std::uppercase << crc_16 << "\n";

      // data
      std::cout << "data: ";
      for (size_t i = offset; i < serialized.size() - sizeof(uint32_t); ++i)
      {
        std::cout << std::hex << std::uppercase << std::setw(2)
                  << std::setfill('0') << static_cast<int>(serialized[i]) << " ";
      }
      std::cout << "\n";

      // crc_32
      uint32_t crc_32;
      std::memcpy(&crc_32, &serialized[serialized.size() - sizeof(uint32_t)],
                  sizeof(crc_32));
      std::cout << "crc_32: 0x" << std::hex << std::uppercase << crc_32 << "\n";
    }
  };

  //======================通用指令===========================
  /**
   * 广播指令 (被动接受，不需要使用该结构体)
   */
  struct boardcast
  {
    uint8_t cmd_set = 0x00;
    uint8_t cmd_id = 0x00;
    uint8_t broadcast_code[16];
    uint8_t dev_type = 0x00;
    uint16_t reserved;

    friend std::ostream &operator<<(std::ostream &os, boardcast &boardcast)
    {
      std::string broadcastCodeStr(
          reinterpret_cast<const char *>(boardcast.broadcast_code), 15);
      os << "cmd_set: 0x" << std::hex << std::uppercase
         << static_cast<int>(boardcast.cmd_set) << "\n";
      os << "cmd_id: 0x" << std::hex << std::uppercase
         << static_cast<int>(boardcast.cmd_id) << "\n";
      // if (DEBUG) {
      os << "broadcast_code: " << broadcastCodeStr << "\n";
      os << "加密后SN: " << std::dec << boardcast::encryptSN(broadcastCodeStr) << "\n";
      // }
      os << "dev_type: 0x" << std::hex << std::uppercase
         << static_cast<int>(boardcast.dev_type) << "\n";
      os << "reserved: 0x" << std::hex << std::uppercase << boardcast.reserved
         << "\n";
      return os;
    }

    std::string getSN()
    {
      return boardcast::encryptSN(
          std::string(reinterpret_cast<const char *>(broadcast_code), 15));
    }

    static std::string encryptSN(const std::string &input)
    {
      if (input.size() != 15)
      {
        throw std::invalid_argument("Input size must be exactly 15 characters.");
      }
      // 加密键
      std::string key = "zwkjLidar";
      std::string output;
      // 对输入字符串进行基本的XOR加密
      for (int i = 0; i < input.size(); ++i)
      {
        char encrypted_char = input[i] ^ key[i % key.size()];
        output.push_back(encrypted_char);
      }
      // 哈希函数，将加密后的数据压缩到8个字符
      std::string final_output(8, '\0');
      for (int i = 0; i < output.size(); ++i)
      {
        final_output[i % 8] ^= output[i];
      }
      // 将最终的输出字符限制在字母、数字和点号的范围内
      for (char &c : final_output)
      {
        int base = c % (10 + 26);
        if (base < 10)
        {
          c = '0' + base; // 0-9
        }
        else
        {
          c = 'A' + (base - 10); // A-Z
        }
      }
      return final_output;
    }
  };

  /**
   * 握手指令
   */
  struct handShake
  {
    uint8_t cmd_set = 0x00;
    uint8_t cmd_id = 0x01;
    std::vector<uint8_t> user_ip;
    uint16_t data_port; // 主机点云数据UDP目的端口
    uint16_t cmd_port;  // 主机控制指令UDP目的端口
    uint16_t imu_port;  // 主机控制IMU UDP目的端口

    explicit handShake(std::vector<uint8_t> user_ip, uint16_t data_port,
                       uint16_t cmd_port, uint16_t imu_port)
        : user_ip(user_ip),
          data_port(data_port),
          cmd_port(cmd_port),
          imu_port(imu_port) {}

    std::vector<uint8_t> operator()()
    {
      std::vector<uint8_t> data;
      data.push_back(cmd_set);
      data.push_back(cmd_id);
      data.insert(data.end(), user_ip.begin(), user_ip.end());
      data.insert(data.end(), reinterpret_cast<uint8_t *>(&data_port),
                  reinterpret_cast<uint8_t *>(&data_port) + sizeof(data_port));
      data.insert(data.end(), reinterpret_cast<uint8_t *>(&cmd_port),
                  reinterpret_cast<uint8_t *>(&cmd_port) + sizeof(cmd_port));
      data.insert(data.end(), reinterpret_cast<uint8_t *>(&imu_port),
                  reinterpret_cast<uint8_t *>(&imu_port) + sizeof(imu_port));
      return data;
    }
  };

  /**
   * 握手应答
   */
  struct handShakeACK
  {
    uint8_t cmd_set = 0x00;
    uint8_t cmd_id = 0x01;
    uint8_t ret_code;

    friend std::ostream &operator<<(std::ostream &os, handShakeACK &ack)
    {
      os << "cmd_set: 0x" << std::hex << std::uppercase
         << static_cast<int>(ack.cmd_set) << "\n";
      os << "cmd_id: 0x" << std::hex << std::uppercase
         << static_cast<int>(ack.cmd_id) << "\n";
      os << "ret_code: 0x" << std::hex << std::uppercase
         << static_cast<int>(ack.ret_code) << "\n";
      return os;
    }
  };

  /**
   * 查询设备指令
   */
  struct queryDevice
  {
    uint8_t cmd_set = 0x00;
    uint8_t cmd_id = 0x02;
  };

  /**
   * 查询设备应答
   */
  struct queryDeviceACK
  {
    uint8_t cmd_set = 0x00;
    uint8_t cmd_id = 0x02;
    uint8_t ret_code;
    uint8_t version[4];
  };

  /**
   *心跳指令
   */
  struct heartBeat
  {
    uint8_t cmd_set = 0x00;
    uint8_t cmd_id = 0x03;

    heartBeat() = default;

    std::vector<uint8_t> operator()()
    {
      std::vector<uint8_t> data;
      data.push_back(cmd_set);
      data.push_back(cmd_id);
      return data;
    }
  };

  /**
   * 心跳应答
   */
  struct heartBeatACK
  {
    uint8_t cmd_set = 0x00;
    uint8_t cmd_id = 0x03;
    uint8_t ret_code;
    uint8_t work_state;
    uint8_t feature_msg;
    uint32_t ack_msg;

    friend std::ostream &operator<<(std::ostream &os, heartBeatACK &ack)
    {
      os << "cmd_set: 0x" << std::hex << std::uppercase
         << static_cast<int>(ack.cmd_set) << "\n";
      os << "cmd_id: 0x" << std::hex << std::uppercase
         << static_cast<int>(ack.cmd_id) << "\n";
      os << "ret_code: 0x" << std::hex << std::uppercase
         << static_cast<int>(ack.ret_code) << "\n";
      os << "work_state: 0x" << std::hex << std::uppercase
         << static_cast<int>(ack.work_state) << "\n";
      os << "feature_msg: 0x" << std::hex << std::uppercase
         << static_cast<int>(ack.feature_msg) << "\n";
      os << "ack_msg: 0x" << std::hex << std::uppercase << ack.ack_msg << "\n";
      return os;
    }
  };

  /**
   * 雷达开关指令
   */
  struct Switch
  {
    uint8_t cmd_set = 0x00;
    uint8_t cmd_id = 0x04;
    uint8_t switch_status;

    explicit Switch(uint8_t switch_status) : switch_status(switch_status) {}

    std::vector<uint8_t> operator()()
    {
      std::vector<uint8_t> data;
      data.push_back(cmd_set);
      data.push_back(cmd_id);
      data.push_back(switch_status);
      return data;
    }
  };

  /**
   * 雷达开关应答
   */
  struct SwitchACK
  {
    uint8_t cmd_set = 0x00;
    uint8_t cmd_id = 0x04;
    uint8_t ret_code = 0x00;

    explicit SwitchACK() {}

    std::vector<uint8_t> operator()()
    {
      std::vector<uint8_t> data;
      data.push_back(cmd_set);
      data.push_back(cmd_id);
      data.push_back(ret_code);
      return data;
    }
  };

  /**
   *更改点云坐标系指令
   *该指令只会在RAM中起效，重启后系统坐标系会复位为直角坐标系.
   */
  struct ChangeCoordinate
  {
    uint8_t cmd_set = 0x00;
    uint8_t cmd_id = 0x05;
    uint8_t coordinate_system;
  };

  /**
   * 更改点云坐标系应答
   */
  struct ChangeCoordinateACK
  {
    uint8_t cmd_set = 0x00;
    uint8_t cmd_id = 0x05;
    uint8_t ret_code;
  };

  /**
   *断开链接指令
   */
  struct Disconnect
  {
    uint8_t cmd_set = 0x00;
    uint8_t cmd_id = 0x06;
  };

  /**
   * 断开链接应答
   */
  struct DisconnectACK
  {
    uint8_t cmd_set = 0x00;
    uint8_t cmd_id = 0x06;
    uint8_t ret_code;
  };

  /**
   *异常信息
   *一种特殊的应答，当雷达发生异常时，雷达会主动发送该指令
   */
  struct ExceptionMsg
  {
    uint8_t cmd_set = 0x00;
    uint8_t cmd_id = 0x07;
    uint32_t status_code; // 需要转换成二进制
    friend std::ostream &operator<<(std::ostream &os, ExceptionMsg &msg)
    {
      os << "cmd_set: 0x" << std::hex << std::uppercase
         << static_cast<int>(msg.cmd_set) << "\n";
      os << "cmd_id: 0x" << std::hex << std::uppercase
         << static_cast<int>(msg.cmd_id) << "\n";
      os << "status_code: " << std::bitset<32>(msg.status_code) << "\n";
      return os;
    }
  };

  /**
   *配置雷达IP 指令
   */
  struct ConfigIP
  {
    uint8_t cmd_set = 0x00;
    uint8_t cmd_id = 0x08;
    uint8_t ip_mode;                 // 0x00 动态ip 0x01 静态ip
    std::vector<uint8_t> lidar_ip;   // 雷达IP
    std::vector<uint8_t> lidar_mask; // 子网掩码
    std::vector<uint8_t> lidar_gw;   // 网关
  };

  /**
   * 配置雷达IP 应答
   */
  struct ConfigIPACK
  {
    uint8_t cmd_set = 0x00;
    uint8_t cmd_id = 0x08;
    uint8_t ret_code;
  };

  /**
   *重启雷达指令
   */
  struct Restart
  {
    uint8_t cmd_set = 0x00;
    uint8_t cmd_id = 0x0A;
    uint16_t timeout;

    explicit Restart(uint16_t timeout) : timeout(timeout) {}

    std::vector<uint8_t> operator()()
    {
      std::vector<uint8_t> data;
      data.push_back(cmd_set);
      data.push_back(cmd_id);
      data.insert(data.end(), reinterpret_cast<uint8_t *>(&timeout),
                  reinterpret_cast<uint8_t *>(&timeout) + sizeof(timeout));
      return data;
    }
  };

  /**
   * 重启雷达应答
   */
  struct RestartACK
  {
    uint8_t cmd_set = 0x00;
    uint8_t cmd_id = 0x0A;
    uint8_t ret_code;

    friend std::ostream &operator<<(std::ostream &os, RestartACK &ack)
    {
      os << "cmd_set: 0x" << std::hex << std::uppercase
         << static_cast<int>(ack.cmd_set) << "\n";
      os << "cmd_id: 0x" << std::hex << std::uppercase
         << static_cast<int>(ack.cmd_id) << "\n";
      os << "ret_code: 0x" << std::hex << std::uppercase
         << static_cast<int>(ack.ret_code) << "\n";
      return os;
    }
  };

  //====================雷达指令集========================
  struct setLidarMode
  {
    uint8_t cmd_set = 0x01;
    uint8_t cmd_id = 0x00;
    uint8_t lidar_mode; // 0x01 正常模式 0x02低功耗 0x03待机
    std::vector<uint8_t> operator()()
    {
      std::vector<uint8_t> data;
      data.push_back(cmd_set);
      data.push_back(cmd_id);
      data.push_back(lidar_mode);
      return data;
    }
  };

  struct setLidarModeACK
  {
    uint8_t cmd_set = 0x01;
    uint8_t cmd_id = 0x00;
    uint8_t ret_code;
  };

  // 抗雨雾
  struct switchRrf
  {
    uint8_t cmd_set = 0x01;
    uint8_t cmd_id = 0x03;
    uint8_t rrf_mode; // 0x00 关闭 0x01 开启

    explicit switchRrf(uint8_t rrf_mode) : rrf_mode(rrf_mode) {}

    std::vector<uint8_t> operator()()
    {
      std::vector<uint8_t> data;
      data.push_back(cmd_set);
      data.push_back(cmd_id);
      data.push_back(rrf_mode);
      return data;
    }
  };

  struct switchRrfACK
  {
    uint8_t cmd_set = 0x01;
    uint8_t cmd_id = 0x03;
    uint8_t ret_code;

    friend std::ostream &operator<<(std::ostream &os, switchRrfACK &ack)
    {
      os << "cmd_set: 0x" << std::hex << std::uppercase
         << static_cast<int>(ack.cmd_set) << "\n";
      os << "cmd_id: 0x" << std::hex << std::uppercase
         << static_cast<int>(ack.cmd_id) << "\n";
    }
  };

  struct switchFan
  {
    uint8_t cmd_set = 0x01;
    uint8_t cmd_id = 0x04;
    uint8_t fan_mode; // 0x00 关闭 0x01 开启

    explicit switchFan(uint8_t fan_mode) : fan_mode(fan_mode) {}

    std::vector<uint8_t> operator()()
    {
      std::vector<uint8_t> data;
      data.push_back(cmd_set);
      data.push_back(cmd_id);
      data.push_back(fan_mode);
      return data;
    }
  };

  struct switchFanACK
  {
    uint8_t cmd_set = 0x01;
    uint8_t cmd_id = 0x04;
    uint8_t ret_code;

    friend std::ostream &operator<<(std::ostream &os, switchFanACK &ack)
    {
      os << "cmd_set: 0x" << std::hex << std::uppercase
         << static_cast<int>(ack.cmd_set) << "\n";
      os << "cmd_id: 0x" << std::hex << std::uppercase
         << static_cast<int>(ack.cmd_id) << "\n";
      os << "ret_code:0x" << std::hex << std::uppercase
         << static_cast<int>(ack.ret_code) << "\n";
      return os;
    }
  };
  // aiva并不支持该指令
  struct readFan
  {
    uint8_t cmd_set = 0x01;
    uint8_t cmd_id = 0x05;

    explicit readFan() = default;

    std::vector<uint8_t> operator()()
    {
      std::vector<uint8_t> data;
      data.push_back(cmd_set);
      data.push_back(cmd_id);
      return data;
    }
  };

  struct readFanACK
  {
    uint8_t cmd_set = 0x01;
    uint8_t cmd_id = 0x05;
    uint8_t ret_code;
    uint8_t fan_status;

    friend std::ostream &operator<<(std::ostream &os, readFanACK &ack)
    {
      os << "cmd_set: 0x" << std::hex << std::uppercase
         << static_cast<int>(ack.cmd_set) << "\n";
      os << "cmd_id: 0x" << std::hex << std::uppercase
         << static_cast<int>(ack.cmd_id) << "\n";
      os << "ret_code:0x" << std::hex << std::uppercase
         << static_cast<int>(ack.ret_code) << "\n";
      os << "fan_status:0x" << std::hex << std::uppercase
         << static_cast<int>(ack.fan_status) << "\n";
      return os;
    }
  };

  struct setIMUFre
  {
    uint8_t cmd_set = 0x01;
    uint8_t cmd_id = 0x08;
    uint8_t imu_fre; // 0x00 close 0x01 200HZ

    setIMUFre(uint8_t imu_fre) : imu_fre(imu_fre) {}

    std::vector<uint8_t> operator()()
    {
      std::vector<uint8_t> data;
      data.push_back(cmd_set);
      data.push_back(cmd_id);
      data.push_back(imu_fre);
      return data;
    }
  };

  struct setIMUFreACK
  {
    uint8_t cmd_set = 0x01;
    uint8_t cmd_id = 0x08;
    uint8_t ret_code; // 0x00成功 0x01失败
  };
};
// //===================点云数据部分=======================
// template <typename dataType, std::size_t N>  // dataType
//                                              // 数据类型(单回波，双回波，三回波，直角坐标系，球坐标系），N
//                                              // 数据个数
//                                              struct dataFrame {
//   uint8_t version;
//   uint8_t slot_id;
//   uint8_t lidar_id;
//   uint8_t reserved;
//   uint32_t status_code;  // 需要转换成二进制
//   uint8_t timestamp_type;
//   uint8_t data_type;
//   uint64_t timestamp;
//   std::array<dataType, N> data;
//   // 序列化输出 重载<<
//   friend std::ostream &operator<<(std::ostream &os, dataFrame &frame) {
//     os << "version: " << static_cast<int>(frame.version) << "\n";
//     os << "slot_id: " << static_cast<int>(frame.slot_id) << "\n";
//     os << "lidar_id: " << static_cast<int>(frame.lidar_id) << "\n";
//     os << "reserved: " << static_cast<int>(frame.reserved) << "\n";
//     os << "status_code: " << std::bitset<32>(frame.status_code) << "\n";
//     os << "timestamp_type: " << std::dec
//        << static_cast<long>(frame.timestamp_type) << "\n";
//     os << "data_type: " << static_cast<int>(frame.data_type) << "\n";
//     os << "timestamp: " << frame.timestamp << "\n";
//     os << "data:\n";
//     for (auto &d : frame.data) {
//       os << d << "\n";
//     }
//     os << std::endl;
//     return os;
//   }
// };

// // 单回波直角坐标系
// struct singleEchoRectangularData {
//   int32_t x;
//   int32_t y;
//   int32_t z;
//   uint8_t intensity;  //反射率
//   uint8_t lable;      //点云标签 需要转换成二进制读取
//   friend std::ostream &operator<<(std::ostream &os,
//                                   singleEchoRectangularData &data) {
//     os << std::dec << "[x: " << data.x / 1000.f << " y: " << data.y / 1000.f
//        << " z: " << data.z / 1000.f
//        << " intensity: " << static_cast<float>(data.intensity)
//        << " lable: " << std::bitset<8>(data.lable) << "]";
//     return os;
//   }
// };

// // IMU数据
// struct imuData {
//   float gyro_x;
//   float gyro_y;
//   float gyro_z;
//   float acc_x;
//   float acc_y;
//   float acc_z;

//   friend std::ostream &operator<<(std::ostream &os, imuData &data) {
//     os << "[gyro_x: " << data.gyro_x << " gyro_y: " << data.gyro_y
//        << " gyro_z: " << data.gyro_z << " acc_x: " << data.acc_x
//        << " acc_y: " << data.acc_y << " acc_z: " << data.acc_z << "]";
//     return os;
//   }
// };

#pragma pack(pop)

#endif // DATASTRUCT_H
