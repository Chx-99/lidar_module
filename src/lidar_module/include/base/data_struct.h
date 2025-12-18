/**
 * @brief 帧格式定义
 *
 * @file data_struct.h
 * @author cao haoxuan
 * @date 2025-12-16
 */

#pragma once
#include "crc_calculator.h"

#include <bitset>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <span>
#include <unordered_map>
#include <vector>

/*
    追加指令:
        每个指令必须包含cmd_set与cmd_id字段
        结构体必须为紧凑格式
        必须实现<<操作符重载

*/

// 最小长度
#define FRAME_SIZE 13
#define DATA_MIN_SIZE 2
#define FRAME_DATA_MIN_SIZE 15

// cmd_set与cmd_id偏移量
#define CMD_SET_INDEX 9
#define CMD_ID_INDEX 10
#define RET_CODE 11

// 获取ack ret code字段
#define GET_ACK_RET_CODE(data) (data)[RET_CODE]
#define GET_ACK_SET(data) (data)[CMD_SET_INDEX]
#define GET_ACK_ID(data) (data)[CMD_ID_INDEX]

#define FRAME_TO_SPAN(data) frame_tools::frameToSpan(reinterpret_cast<const uint8_t*>(&data), data.length)

namespace frame_tools {
// 获取ack中data字段
inline std::span<const uint8_t> getDataFieldSpan(const uint8_t* data, size_t size) {
    if (size < FRAME_DATA_MIN_SIZE) throw std::invalid_argument("帧协议长度最小长度是15字节");
    return std::span<const uint8_t>(data + 9, size - FRAME_SIZE);
}

// 通过set与id解析对应指令类型
constexpr std::string_view getAckNameCompileTime(uint8_t cmd_set, uint8_t cmd_id) {
    const uint16_t key = (static_cast<uint16_t>(cmd_set) << 8) | cmd_id;
    switch (key) {
        case 0x0000: return "Broadcast MSG";
        case 0x0001: return "HandShake ACK";
        case 0x0003: return "HeartBeat ACK";
        case 0x0004: return "SetLaserStatus ACK";
        case 0x0007: return "ErrorMessage";
        case 0x0108: return "Set IMU Frequency ACK";
        default: return "UnKnown ACK/MSG";
    }
}

// 将消息帧转化为span<const uint8_t>
inline std::span<const uint8_t> frameToSpan(const uint8_t* data, size_t size) {
    return std::span<const uint8_t>(data, size);
}


};  // namespace frame_tools

namespace base_frame {
#pragma pack(push, 1)  // 确保结构体的字节对齐为1字节

/**
 * @brief 帧结构体:基础数据格式，通过可变模板参数为data字段构造，在构造函数中通过update函数更新crc字段与length字段
 *
 * @tparam DataType 数据类型
 */
template <typename DataType>
class Frame {
public:
    uint8_t sof = 0xaa;       // 起始字节，固定为 0xAA
    uint8_t version = 0x01;   // 协议版本
    uint16_t length;          // 数据帧长度
    uint8_t cmd_type = 0x00;  // 命令类型
    uint16_t seq_num = 0x00;  // 数据帧序列号
    uint16_t crc_16;          // 包头校验码
    DataType data;            // 数据
    uint32_t crc_32;          // 整个数据帧校验码
public:
    // 可变参构造函数
    template <typename... Args>
    explicit Frame(Args&&... args) : data((args)...) {
        update();
    }
    // 无参的时候使用
    explicit Frame() { update(); }

    void update() {
        length = static_cast<uint16_t>(sizeof(data) + FRAME_SIZE);
        crc_16 = crc16(reinterpret_cast<const uint8_t*>(this), 7);
        crc_32 = crc32(reinterpret_cast<const uint8_t*>(this), length - 4);
    }

    template <typename T>
    friend inline std::ostream& operator<<(std::ostream& os, const Frame<T>& frame);
};
template <typename T>
inline std::ostream& operator<<(std::ostream& os, const Frame<T>& frame) {
    // 使用stringstream预构建，减少流操作次数
    std::ostringstream oss;
    oss << std::hex << std::uppercase << std::setfill('0');
    // 构建sof
    oss << "sof: 0x" << std::setw(2) << static_cast<int>(frame.sof) << "\n";
    // 构建version
    oss << "version: 0x" << std::setw(2) << static_cast<int>(frame.version) << "\n";
    // 构建length
    oss << "length: 0x" << std::setw(4) << frame.length << "\n";
    // 构建cmd_type
    oss << "cmd_type: 0x" << std::setw(2) << static_cast<int>(frame.cmd_type) << "\n";
    // 构建seq_num
    oss << "seq_num: 0x" << std::setw(4) << frame.seq_num << "\n";
    // 构建crc_16
    oss << "crc_16: 0x" << std::setw(4) << frame.crc_16 << "\n";
    // 构建data
    oss << "data: " << frame.data << "\n";
    // 构建crc_32
    oss << "crc_32: 0x" << std::setw(8) << frame.crc_32 << "\n";

    os << oss.str();
    return os;
}

//======================通用指令===========================
/**
 * 广播指令 (被动接受，不需要使用该结构体)
 */
struct BoardCast {
    uint8_t cmd_set = 0x00;
    uint8_t cmd_id = 0x00;
    uint8_t broadcast_code[16];
    uint8_t dev_type = 0x00;
    uint16_t reserved;

    inline friend std::ostream& operator<<(std::ostream& os, const BoardCast& boardcast) {
        std::ostringstream oss;
        std::string broadcast_code_str(reinterpret_cast<const char*>(boardcast.broadcast_code), 15);
        oss << std::hex << std::uppercase << std::setfill('0');
        oss << "cmd_set: 0x" << static_cast<int>(boardcast.cmd_set) << "\n";
        oss << "cmd_id: 0x" << static_cast<int>(boardcast.cmd_id) << "\n";
        oss << "broadcast_code: " << broadcast_code_str << "\n";
        oss << "zwkj-SN: " << std::dec << BoardCast::encryptSN(broadcast_code_str) << "\n";
        oss << "dev_type: 0x" << static_cast<int>(boardcast.dev_type) << "\n";
        oss << "reserved: 0x" << boardcast.reserved << "\n";
        oss << oss.str();
        return os;
    }

    //  zwkj-SN加密算法 当前版本未使用该功能，广播码由程序统一管理，不对外提供
    inline std::string getzwkjSN() {
        return BoardCast::encryptSN(std::string(reinterpret_cast<const char*>(broadcast_code), 15));
    }

    inline std::string getBroadCastCode() { return std::string(reinterpret_cast<const char*>(broadcast_code), 15); }

    inline static std::string encryptSN(const std::string& input) {
        if (input.size() != 15) { throw std::invalid_argument("Input size must be exactly 15 characters."); }
        // 预分配结果空间
        std::string output(8, '\0');
        // 使用固定大小的缓存数组
        alignas(16) uint8_t encrypted[15];  // 对齐到16字节，便于可能的SIMD优化
        // 展开循环，避免取模运算
        const char* key = "zwkjLidar";
        const size_t key_len = 9;  // 硬编码密钥长度，避免在循环中计算
        // 优化1: 展开XOR循环，减少条件判断
        encrypted[0] = input[0] ^ key[0 % key_len];
        encrypted[1] = input[1] ^ key[1 % key_len];
        encrypted[2] = input[2] ^ key[2 % key_len];
        encrypted[3] = input[3] ^ key[3 % key_len];
        encrypted[4] = input[4] ^ key[4 % key_len];
        encrypted[5] = input[5] ^ key[5 % key_len];
        encrypted[6] = input[6] ^ key[6 % key_len];
        encrypted[7] = input[7] ^ key[7 % key_len];
        encrypted[8] = input[8] ^ key[8 % key_len];  // 8 % 9 = 8
        encrypted[9] = input[9] ^ key[0];            // 9 % 9 = 0
        encrypted[10] = input[10] ^ key[1];
        encrypted[11] = input[11] ^ key[2];
        encrypted[12] = input[12] ^ key[3];
        encrypted[13] = input[13] ^ key[4];
        encrypted[14] = input[14] ^ key[5];

        // 优化2: 使用整数数组进行XOR运算，避免字符串操作
        uint8_t temp[8] = {0, 0, 0, 0, 0, 0, 0, 0};

        // 展开压缩循环
        temp[0] ^= encrypted[0] ^ encrypted[8];
        temp[1] ^= encrypted[1] ^ encrypted[9];
        temp[2] ^= encrypted[2] ^ encrypted[10];
        temp[3] ^= encrypted[3] ^ encrypted[11];
        temp[4] ^= encrypted[4] ^ encrypted[12];
        temp[5] ^= encrypted[5] ^ encrypted[13];
        temp[6] ^= encrypted[6] ^ encrypted[14];
        temp[7] ^= encrypted[7];

        // 优化3: 使用查找表进行字符映射
        // 创建静态查找表，避免重复计算
        static const uint8_t char_map[36] = {
            '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',  // 0-9
            'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J',  // 10-19
            'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T',  // 20-29
            'U', 'V', 'W', 'X', 'Y', 'Z'                       // 30-35
        };

        // 展开映射循环
        output[0] = char_map[temp[0] % 36];
        output[1] = char_map[temp[1] % 36];
        output[2] = char_map[temp[2] % 36];
        output[3] = char_map[temp[3] % 36];
        output[4] = char_map[temp[4] % 36];
        output[5] = char_map[temp[5] % 36];
        output[6] = char_map[temp[6] % 36];
        output[7] = char_map[temp[7] % 36];

        return output;
    }
};

/**
 * 握手指令
 */
struct HandShake {
    uint8_t cmd_set = 0x00;
    uint8_t cmd_id = 0x01;
    uint8_t user_ip[4];
    uint16_t data_port;  // 主机点云数据UDP目的端口
    uint16_t cmd_port;   // 主机控制指令UDP目的端口
    uint16_t imu_port;   // 主机控制IMU UDP目的端口

    explicit HandShake(std::string local_ip, uint16_t data_port, uint16_t cmd_port, uint16_t imu_port)
        : data_port(data_port), cmd_port(cmd_port), imu_port(imu_port) {
        parseIpString(local_ip, user_ip);
    }

    inline friend std::ostream& operator<<(std::ostream& os, const HandShake& handshake) {
        std::ostringstream oss;
        oss << std::hex << std::uppercase << std::setfill('0');
        oss << "cmd_set: 0x" << static_cast<int>(handshake.cmd_set) << "\n";
        oss << "cmd_id: 0x" << static_cast<int>(handshake.cmd_id) << "\n";
        oss << "user_ip: " << std::dec << static_cast<int>(handshake.user_ip[0]) << "."
            << static_cast<int>(handshake.user_ip[1]) << "." << static_cast<int>(handshake.user_ip[2]) << "."
            << static_cast<int>(handshake.user_ip[3]) << "\n";
        oss << "data_port: " << std::dec << handshake.data_port << "\n";
        oss << "cmd_port: " << std::dec << handshake.cmd_port << "\n";
        oss << "imu_port: " << std::dec << handshake.imu_port << "\n";
        os << oss.str();
        return os;
    }

    // 修改为静态方法，直接解析到数组
    static void parseIpString(const std::string& ip_str, uint8_t ip[4]) {
        unsigned int a, b, c, d;
        if (std::sscanf(ip_str.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) != 4) {
            throw std::runtime_error("Invalid IP format: " + ip_str);
        }

        if (a > 255 || b > 255 || c > 255 || d > 255) {
            throw std::runtime_error("Invalid IP octet value in: " + ip_str);
        }

        ip[0] = static_cast<uint8_t>(a);
        ip[1] = static_cast<uint8_t>(b);
        ip[2] = static_cast<uint8_t>(c);
        ip[3] = static_cast<uint8_t>(d);
    }
};

/**
 * 握手应答
 */
struct HandShakeACK {
    uint8_t cmd_set = 0x00;
    uint8_t cmd_id = 0x01;
    uint8_t ret_code;

    explicit HandShakeACK() {}

    inline friend std::ostream& operator<<(std::ostream& os, const HandShakeACK& ack) {
        std::ostringstream oss;
        oss << std::hex << std::uppercase << std::setfill('0');
        oss << "cmd_set: 0x" << static_cast<int>(ack.cmd_set) << "\n";
        oss << "cmd_id: 0x" << static_cast<int>(ack.cmd_id) << "\n";
        oss << "ret_code: 0x" << static_cast<int>(ack.ret_code) << "\n";
        os << oss.str();
        return os;
    }
};

/**
 *心跳指令
 */
struct HeartBeat {
    uint8_t cmd_set = 0x00;
    uint8_t cmd_id = 0x03;

    HeartBeat() = default;

    inline friend std::ostream& operator<<(std::ostream& os, const HeartBeat& ack) {
        std::ostringstream oss;
        oss << std::hex << std::uppercase << std::setfill('0');
        oss << "cmd_set: 0x" << static_cast<int>(ack.cmd_set) << "\n";
        oss << "cmd_id: 0x" << static_cast<int>(ack.cmd_id) << "\n";
        os << oss.str();
        return os;
    }
};

/**
 * 心跳应答
 */
struct HeartBeatACK {
    uint8_t cmd_set = 0x00;
    uint8_t cmd_id = 0x03;
    uint8_t ret_code;
    uint8_t work_state;
    uint8_t feature_msg;
    uint32_t status_info;

    inline friend std::ostream& operator<<(std::ostream& os, const HeartBeatACK& ack) {
        std::ostringstream oss;
        oss << std::hex << std::uppercase << std::setfill('0');
        oss << "cmd_set: 0x" << static_cast<int>(ack.cmd_set) << "\n";
        oss << "cmd_id: 0x" << static_cast<int>(ack.cmd_id) << "\n";
        oss << "ret_code: 0x" << static_cast<int>(ack.ret_code) << "\n";
        oss << "work_state: 0x" << static_cast<int>(ack.work_state) << "\n";
        oss << "feature_msg: 0x" << static_cast<int>(ack.feature_msg) << "\n";
        oss << "status_info: 0x" << ack.status_info << "\n";
        os << oss.str();
        return os;
    }
};

/**
 * 雷达开关指令
 */
struct SetLaserStatus {
    uint8_t cmd_set = 0x00;
    uint8_t cmd_id = 0x04;
    uint8_t switch_status;

    explicit SetLaserStatus(uint8_t switch_status) : switch_status(switch_status) {}

    inline friend std::ostream& operator<<(std::ostream& os, const SetLaserStatus& ack) {
        std::ostringstream oss;
        oss << std::hex << std::uppercase << std::setfill('0');
        oss << "cmd_set: 0x" << static_cast<int>(ack.cmd_set) << "\n";
        oss << "cmd_id: 0x" << static_cast<int>(ack.cmd_id) << "\n";
        oss << "switch_status: 0x" << static_cast<int>(ack.switch_status) << "\n";
        os << oss.str();
        return os;
    }
};

/**
 * 雷达开关应答
 */
struct SetLaserStatusACK {
    uint8_t cmd_set = 0x00;
    uint8_t cmd_id = 0x04;
    uint8_t ret_code = 0x00;

    explicit SetLaserStatusACK() {}

    inline friend std::ostream& operator<<(std::ostream& os, const SetLaserStatusACK& ack) {
        std::ostringstream oss;
        oss << std::hex << std::uppercase << std::setfill('0');
        oss << "cmd_set: 0x" << static_cast<int>(ack.cmd_set) << "\n";
        oss << "cmd_id: 0x" << static_cast<int>(ack.cmd_id) << "\n";
        oss << "ret_code: 0x" << static_cast<int>(ack.ret_code) << "\n";
        os << oss.str();
        return os;
    }
};

/**
 * 异常信息
 * 一种特殊的应答，当雷达发生异常时，雷达会主动发送该消息
 */
struct ErrorMessage {
    uint8_t cmd_set = 0x00;
    uint8_t cmd_id = 0x07;
    uint32_t status_code;  // 需要转换成二进制
    inline friend std::ostream& operator<<(std::ostream& os, ErrorMessage& msg) {
        std::ostringstream oss;
        oss << std::hex << std::uppercase << std::setfill('0');
        oss << "cmd_set: 0x" << static_cast<int>(msg.cmd_set) << "\n";
        oss << "cmd_id: 0x" << static_cast<int>(msg.cmd_id) << "\n";
        oss << "status_code: " << std::bitset<32>(msg.status_code) << "\n";
        oss << "status_code: " << msg.parseLidarStatusToString(msg.status_code) << "\n";
        os << oss.str();
        return os;
    }

    inline std::string parseLidarStatusToString(uint32_t status_code) {
        thread_local static std::string result;
        thread_local static const char* temp_desc[3] = {"正常", "偏高/偏低", "极高/极低"};
        thread_local static const char* volt_desc[3] = {"正常", "偏高", "极高"};
        thread_local static const char* motor_desc[3] = {"正常", "警告", "错误"};
        thread_local static const char* dirty_desc[2] = {"无遮挡", "有遮挡"};
        thread_local static const char* firmware_desc[2] = {"正常", "出错，需要升级"};
        thread_local static const char* pps_desc[2] = {"无PPS", "正常"};
        thread_local static const char* device_desc[2] = {"正常", "寿命警告"};
        thread_local static const char* fan_desc[2] = {"正常", "风扇警告"};
        thread_local static const char* heat_desc[2] = {"关闭", "开启"};
        thread_local static const char* ptp_desc[2] = {"无1588信号", "正常"};
        thread_local static const char* time_sync_desc[5] = {"未同步", "PTP 1588", "GPS", "PPS", "系统时间异常"};
        thread_local static const char* system_desc[4] = {"正常", "警告", "错误", "未知"};

        result.clear();
        result.reserve(1024);  // 预分配足够空间

        // 使用bitset获取原始位字符串
        std::bitset<32> bits(status_code);

        // 直接位运算提取，避免函数调用开销
        uint32_t temp_status = (status_code) & 0x3;
        uint32_t volt_status = (status_code >> 2) & 0x3;
        uint32_t motor_status = (status_code >> 4) & 0x3;
        uint32_t dirty_warn = (status_code >> 6) & 0x3;
        uint32_t firmware_status = (status_code >> 8) & 0x1;
        uint32_t pps_status = (status_code >> 9) & 0x1;
        uint32_t device_status = (status_code >> 10) & 0x1;
        uint32_t fan_status = (status_code >> 11) & 0x1;
        uint32_t self_heating = (status_code >> 12) & 0x1;
        uint32_t ptp_status = (status_code >> 13) & 0x1;
        uint32_t time_sync_status = (status_code >> 14) & 0x7;
        uint32_t system_status = (status_code >> 30) & 0x3;

        // 直接构建字符串，避免中间对象
        result += "{\"raw_bits\":\"";
        result += bits.to_string();
        result += "\",";

        result += "\"temp_status\":{\"value\":";
        result += std::to_string(temp_status);
        result += ",\"desc\":\"";
        result += temp_desc[temp_status < 3 ? temp_status : 2];
        result += "\"},";

        result += "\"volt_status\":{\"value\":";
        result += std::to_string(volt_status);
        result += ",\"desc\":\"";
        result += volt_desc[volt_status < 3 ? volt_status : 2];
        result += "\"},";

        result += "\"motor_status\":{\"value\":";
        result += std::to_string(motor_status);
        result += ",\"desc\":\"";
        result += motor_desc[motor_status < 3 ? motor_status : 2];
        result += "\"},";

        result += "\"dirty_warn\":{\"value\":";
        result += std::to_string(dirty_warn);
        result += ",\"desc\":\"";
        result += dirty_desc[dirty_warn < 2 ? dirty_warn : 1];
        result += "\"},";

        result += "\"firmware_status\":{\"value\":";
        result += std::to_string(firmware_status);
        result += ",\"desc\":\"";
        result += firmware_desc[firmware_status < 2 ? firmware_status : 1];
        result += "\"},";

        result += "\"pps_status\":{\"value\":";
        result += std::to_string(pps_status);
        result += ",\"desc\":\"";
        result += pps_desc[pps_status < 2 ? pps_status : 1];
        result += "\"},";

        result += "\"device_status\":{\"value\":";
        result += std::to_string(device_status);
        result += ",\"desc\":\"";
        result += device_desc[device_status < 2 ? device_status : 1];
        result += "\"},";

        result += "\"fan_status\":{\"value\":";
        result += std::to_string(fan_status);
        result += ",\"desc\":\"";
        result += fan_desc[fan_status < 2 ? fan_status : 1];
        result += "\"},";

        result += "\"self_heating\":{\"value\":";
        result += std::to_string(self_heating);
        result += ",\"desc\":\"";
        result += heat_desc[self_heating < 2 ? self_heating : 1];
        result += "\"},";

        result += "\"ptp_status\":{\"value\":";
        result += std::to_string(ptp_status);
        result += ",\"desc\":\"";
        result += ptp_desc[ptp_status < 2 ? ptp_status : 1];
        result += "\"},";

        result += "\"time_sync_status\":{\"value\":";
        result += std::to_string(time_sync_status);
        result += ",\"desc\":\"";
        result += time_sync_status < 5 ? time_sync_desc[time_sync_status] : "未知";
        result += "\"},";

        result += "\"system_status\":{\"value\":";
        result += std::to_string(system_status);
        result += ",\"desc\":\"";
        result += system_desc[system_status < 4 ? system_status : 3];
        result += "\"}}";

        return result;
    }
};

/*
    设置IMU频率
*/
struct SetIMUFrequency {
    uint8_t cmd_set = 0x01;
    uint8_t cmd_id = 0x08;
    uint8_t imu_frequency;  // IMU频率: 0x00关闭, 0x01 200Hz

    explicit SetIMUFrequency(uint8_t frequency) : imu_frequency(frequency) {}

    inline friend std::ostream& operator<<(std::ostream& os, const SetIMUFrequency& msg) {
        std::ostringstream oss;
        oss << std::hex << std::uppercase << std::setfill('0');
        oss << "cmd_set: 0x" << static_cast<int>(msg.cmd_set) << "\n";
        oss << "cmd_id: 0x" << static_cast<int>(msg.cmd_id) << "\n";
        oss << "imu_frequency: 0x" << static_cast<int>(msg.imu_frequency) << "\n";
        os << oss.str();
        return os;
    }
};

/*
    设置IMU频率ACK
*/
struct set_imu_frequency_ack {
    uint8_t cmd_set = 0x01;
    uint8_t cmd_id = 0x08;
    uint8_t ret_code;  // 0x00成功 0x01失败

    inline friend std::ostream& operator<<(std::ostream& os, const set_imu_frequency_ack& msg) {
        std::ostringstream oss;
        oss << std::hex << std::uppercase << std::setfill('0');
        oss << "cmd_set: 0x" << static_cast<int>(msg.cmd_set) << "\n";
        oss << "cmd_id: 0x" << static_cast<int>(msg.cmd_id) << "\n";
        oss << "ret_code: 0x" << static_cast<int>(msg.ret_code) << "\n";
        os << oss.str();
        return os;
    }
};

/*
    传感器数据基础帧协议
*/
template <typename dataType, std::size_t N = 96>
struct dataFrame {
    uint8_t version;                // 版本号
    uint8_t slot_id;                // 端口号
    uint8_t lidar_id;               // 雷达编号
    uint8_t reserved;               // 保留
    uint32_t status_code;           // 状态码
    uint8_t timestamp_type;         // 时间戳类型
    uint8_t data_type;              // 数据类型
    uint64_t timestamp;             // 时间戳
    std::array<dataType, N> datas;  // 数据

    inline friend std::ostream& operator<<(std::ostream& os, const dataFrame& frame) {
        std::ostringstream oss;
        oss << "version: " << static_cast<int>(frame.version) << "\n";
        oss << "slot_id: " << static_cast<int>(frame.slot_id) << "\n";
        oss << "lidar_id: " << static_cast<int>(frame.lidar_id) << "\n";
        oss << "reserved: " << static_cast<int>(frame.reserved) << "\n";
        oss << "status_code: " << std::bitset<32>(frame.status_code) << "\n";
        oss << "timestamp_type: " << std::dec << static_cast<long>(frame.timestamp_type) << "\n";
        oss << "data_type: " << static_cast<int>(frame.data_type) << "\n";
        oss << "timestamp: " << frame.timestamp << "\n";
        oss << "data:\n";
        for (const auto& data : frame.datas) { oss << data << "\n"; }
        os << oss.str();
        return os;
    }
};

/*
    单回波直角坐标系
*/
struct SingleEchoRectangularData {
    int32_t x;
    int32_t y;
    int32_t z;
    uint8_t intensity;  // 反射率
    uint8_t lable;      // 点云标签 需要转换成二进制读取
    friend std::ostream& operator<<(std::ostream& os, SingleEchoRectangularData& data) {
        std::ostringstream oss;
        oss << std::dec << "[x: " << data.x / 1000.f << " y: " << data.y / 1000.f << " z: " << data.z / 1000.f
            << " intensity: " << static_cast<float>(data.intensity) << " lable: " << std::bitset<8>(data.lable) << "]"
            << std::endl;
        os << oss.str();
        return os;
    }
};
/*
    IMU数据
*/
struct ImuData {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;

    friend std::ostream& operator<<(std::ostream& os, const ImuData& data) {
        std::ostringstream oss;
        oss << "[gyro_x: " << data.gyro_x << " gyro_y: " << data.gyro_y << " gyro_z: " << data.gyro_z
            << " acc_x: " << data.acc_x << " acc_y: " << data.acc_y << " acc_z: " << data.acc_z << "]" << std::endl;
        os << oss.str();
        return os;
    }
};

#pragma pack(pop)
};  // namespace base_frame