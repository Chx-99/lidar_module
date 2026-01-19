#ifndef PLC_DEVICE_BASE_HPP
#define PLC_DEVICE_BASE_HPP

#include <string>
#include <variant>
#include <vector>
#include <cstdint>

enum class ByteOrder
{
    BigEndian,
    LittleEndian
};

using PlcValue = std::variant<bool, int16_t, int32_t, float, double>;

class PlcDevice
{
public:
    explicit PlcDevice(ByteOrder order = ByteOrder::BigEndian) : byte_order_(order) {}
    virtual ~PlcDevice() = default;

    virtual void connect() = 0;
    virtual void disconnect() = 0;
    virtual void sendRequest() = 0;
    virtual void handleResponse() = 0;
    virtual std::string getDeviceInfo() const = 0;

protected:
    ByteOrder byte_order_;

    // 工具函数：编码任意 PLC 数据类型为 modbus 16-bit 寄存器数组
    std::vector<uint16_t> encodeValue(const PlcValue &value) const
    {
        std::vector<uint16_t> regs;
        if (std::holds_alternative<bool>(value))
        {
            regs.push_back(std::get<bool>(value) ? 1 : 0);
        }
        else if (std::holds_alternative<int16_t>(value))
        {
            int16_t v = std::get<int16_t>(value);
            regs.push_back(static_cast<uint16_t>(v));
        }
        else if (std::holds_alternative<int32_t>(value))
        {
            int32_t v = std::get<int32_t>(value);
            uint16_t high = static_cast<uint16_t>((v >> 16) & 0xFFFF);
            uint16_t low = static_cast<uint16_t>(v & 0xFFFF);
            if (byte_order_ == ByteOrder::BigEndian)
            {
                regs = {high, low};
            }
            else
            {
                regs = {low, high};
            }
        }
        else if (std::holds_alternative<float>(value))
        {
            float v = std::get<float>(value);
            uint32_t bin;
            std::memcpy(&bin, &v, sizeof(float));
            uint16_t high = (bin >> 16) & 0xFFFF;
            uint16_t low = bin & 0xFFFF;
            regs = (byte_order_ == ByteOrder::BigEndian)
                       ? std::vector<uint16_t>{high, low}
                       : std::vector<uint16_t>{low, high};
        }
        else if (std::holds_alternative<double>(value))
        {
            double v = std::get<double>(value);
            uint64_t bin;
            std::memcpy(&bin, &v, sizeof(double));
            for (int i = 0; i < 4; ++i)
            {
                uint16_t part = (bin >> (48 - i * 16)) & 0xFFFF;
                regs.push_back(part);
            }
            if (byte_order_ == ByteOrder::LittleEndian)
            {
                std::reverse(regs.begin(), regs.end());
            }
        }

        return regs;
    }

    // 解码为 PlcValue，可用于控制类/轮询类设备
    PlcValue decodeValue(const std::vector<uint16_t> &regs, const std::string &type_hint) const
    {
        if (type_hint == "bool" && !regs.empty())
        {
            return regs[0] != 0;
        }
        else if (type_hint == "int16" && regs.size() >= 1)
        {
            return static_cast<int16_t>(regs[0]);
        }
        else if (type_hint == "int32" && regs.size() >= 2)
        {
            uint32_t value;
            if (byte_order_ == ByteOrder::BigEndian)
            {
                value = (static_cast<uint32_t>(regs[0]) << 16) | regs[1];
            }
            else
            {
                value = (static_cast<uint32_t>(regs[1]) << 16) | regs[0];
            }
            return static_cast<int32_t>(value);
        }
        else if (type_hint == "float" && regs.size() >= 2)
        {
            uint32_t value;
            if (byte_order_ == ByteOrder::BigEndian)
            {
                value = (static_cast<uint32_t>(regs[0]) << 16) | regs[1];
            }
            else
            {
                value = (static_cast<uint32_t>(regs[1]) << 16) | regs[0];
            }
            float result;
            std::memcpy(&result, &value, sizeof(result));
            return result;
        }
        else if (type_hint == "double" && regs.size() >= 4)
        {
            uint64_t value = 0;
            for (int i = 0; i < 4; ++i)
            {
                int idx = (byte_order_ == ByteOrder::BigEndian) ? i : (3 - i);
                value |= (static_cast<uint64_t>(regs[idx]) << ((3 - i) * 16));
            }
            double result;
            std::memcpy(&result, &value, sizeof(result));
            return result;
        }

        throw std::invalid_argument("无法解析类型：" + type_hint);
    }
};

#endif // PLC_DEVICE_BASE_HPP
