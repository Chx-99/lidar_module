/*******************************************************************************
****FilePath: /Photonix/src/plc_module/include/plc_module/modbus_rtu.h
****Author: mwt 911608720@qq.com
****Date: 2025-04-15 09:00:58
****Description:
****Copyright: 2025 by mwt, All Rights Reserved.
********************************************************************************/
#ifndef MODBUS_RTU_HPP
#define MODBUS_RTU_HPP

#include <modbus/modbus.h>
#include <string>
#include "sensor/motor/base/comm_interface.hpp"

class ModbusRTU : public CommInterface
{
public:
    ModbusRTU(const std::string &device, int baud = 115200, char parity = 'N', int data_bit = 8, int stop_bit = 1)
        : device_(device), baud_(baud), data_bit_(data_bit), stop_bit_(stop_bit), parity_(parity), ctx_(nullptr) {}

    ~ModbusRTU() override { disconnect(); }
    void connect(int slave_id) override
    {
        if (ctx_)
        {
            modbus_close(ctx_);
            modbus_free(ctx_);
            ctx_ = nullptr;
        }

        ctx_ = modbus_new_rtu(device_.c_str(), baud_, parity_, data_bit_, stop_bit_);
        if (!ctx_)
        {
            throw std::runtime_error("创建 Modbus RTU 失败");
        }

        if (modbus_set_slave(ctx_, slave_id) == -1)
        {
            std::string err = modbus_strerror(errno);
            throw std::runtime_error("设置从站 ID 失败: " + err);
        }

        if (modbus_connect(ctx_) == -1)
        {
            std::string err = modbus_strerror(errno);
            throw std::runtime_error("串口连接失败: " + err);
        }
        modbus_set_debug(ctx_, false);
    }

    void disconnect() override
    {
        if (ctx_)
        {
            modbus_close(ctx_);
            modbus_free(ctx_);
            ctx_ = nullptr;
        }
    }

    bool writeRegisters(int address, const std::vector<uint16_t> &data) override
    {
        if (!ctx_)
            return false;
        if (data.empty())
            return false;

        if (data.size() == 1)
        {
            int rc = modbus_write_register(ctx_, address, data[0]); // 功能码 0x06
            return rc == 1;
        }
        else
        {
            int rc = modbus_write_registers(ctx_, address, data.size(),
                                            data.data()); // 功能码 0x10
            return rc == static_cast<int>(data.size());
        }
    }

    bool readHoldingRegisters(int address, int count, std::vector<uint16_t> &out) override
    {
        if (!ctx_)
            return false;
        out.resize(count);
        int rc = modbus_read_registers(ctx_, address, count, out.data());
        if (rc == -1)
        {
            std::string err = modbus_strerror(errno);
            std::cout << "错误: " << err << std::endl;
        }
        return rc == count;
    }

    bool readInputRegisters(int address, int count, std::vector<uint16_t> &out) override
    {
        if (!ctx_)
            return false;
        out.resize(count);
        int rc = modbus_read_input_registers(ctx_, address, count, out.data());
        return rc == count;
    }

    bool writeCoil(int address, bool value) override
    {
        if (!ctx_)
            return false;
        return modbus_write_bit(ctx_, address, value ? 1 : 0) == 1;
    }

    void setSlaveId(int id) const override
    {
        if (!ctx_)
            return;
        modbus_set_slave(ctx_, id);
    }

    std::string info() const override
    {
        return "ModbusRTU(" + device_ + ", " + std::to_string(baud_) + ", " +
               parity_ + ", " + std::to_string(data_bit_) + ", " +
               std::to_string(stop_bit_) + ")";
    }

private:
    std::string device_;
    int baud_, data_bit_, stop_bit_, slave_id_;
    char parity_;
    modbus_t *ctx_;
};

#endif // MODBUS_RTU_HPP
