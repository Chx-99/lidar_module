/*******************************************************************************
****FilePath: /Photonix/src/plc_module/include/plc_module/modbus_tcp.h
****Author: mwt 911608720@qq.com
****Date: 2025-04-15 09:00:54
****Description:
****Copyright: 2025 by mwt, All Rights Reserved.
********************************************************************************/
#ifndef MODBUS_TCP_HPP
#define MODBUS_TCP_HPP

#include <modbus/modbus.h>
#include <string>
#include "sensor/motor/base/comm_interface.hpp"

class ModbusTCP : public CommInterface
{
public:
    ModbusTCP(const std::string &ip, int port) : ip_(ip), port_(port), ctx_(nullptr) {}
    ~ModbusTCP() override { disconnect(); }
    void connect(int slave_id) override
    {
        if (ctx_)
        {
            modbus_close(ctx_);
            modbus_free(ctx_);
            ctx_ = nullptr;
        }

        ctx_ = modbus_new_tcp(ip_.c_str(), port_);
        if (!ctx_)
        {
            throw std::runtime_error(
                "modbus tcp 连接失败: modbus_new_tcp 返回空指针");
        }

        if (modbus_connect(ctx_) == -1)
        {
            std::string err = modbus_strerror(errno);
            modbus_free(ctx_);
            ctx_ = nullptr;
            throw std::runtime_error("modbus tcp 连接失败: " + err);
        }

        if (modbus_set_slave(ctx_, slave_id) == -1)
        {
            std::string err = modbus_strerror(errno);
            modbus_close(ctx_);
            modbus_free(ctx_);
            ctx_ = nullptr;
            throw std::runtime_error("设置从站 ID 失败: " + err);
        }
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
        int rc = modbus_write_registers(ctx_, address, data.size(), data.data());
        return rc != -1;
    }

    bool readHoldingRegisters(int address, int count, std::vector<uint16_t> &out) override
    {
        if (!ctx_)
            return false;
        out.resize(count);
        int rc = modbus_read_registers(ctx_, address, count, out.data());
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
        if (ctx_)
            modbus_set_slave(ctx_, id);
    }

    std::string info() const override
    {
        return "ModbusTCP(" + ip_ + ":" + std::to_string(port_) + ")";
    }

private:
    std::string ip_;
    int port_;
    modbus_t *ctx_;
};

#endif // MODBUS_TCP_HPP
