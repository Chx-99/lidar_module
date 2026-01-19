/*******************************************************************************
****FilePath: /Photonix/src/plc_module/include/plc_module/transparent_transport_wrapper.h
****Author: mwt 911608720@qq.com
****Date: 2025-07-31 12:57:59
****Description:
****Copyright: 2025 by mwt, All Rights Reserved.
********************************************************************************/
#ifndef TRANSPARENT_TRANSPORT_WRAPPER_H
#define TRANSPARENT_TRANSPORT_WRAPPER_H

#include <string>
#include <vector>
#include <mutex>
#include <cstring>
#include <iostream>
#include <sstream>

#include <boost/asio.hpp>
#include <modbus.h>

#include "sensor/motor/base/comm_interface.hpp"

static uint16_t compute_modbus_crc16(const uint8_t *data, int length)
{
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < length; pos++)
    {
        crc ^= data[pos];
        for (int i = 0; i < 8; i++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

class TransparentTransportWrapper : public CommInterface
{
public:
    TransparentTransportWrapper(const std::string &ip, int port)
        : ip_(ip), port_(port), slave_id_(1), socket_(io_) {}

    ~TransparentTransportWrapper() { disconnect(); }

    void connect(int slave_id) override
    {
        slave_id_ = slave_id;
        boost::asio::ip::tcp::resolver resolver(io_);
        auto endpoints = resolver.resolve(ip_, std::to_string(port_));
        boost::asio::connect(socket_, endpoints);
    }

    void disconnect() override
    {
        if (socket_.is_open())
            socket_.close();
    }

    bool readHoldingRegisters(int address, int count, std::vector<uint16_t> &out) override
    {
        return readRegisters(0x03, address, count, out);
    }
    bool readInputRegisters(int address, int count, std::vector<uint16_t> &out) override
    {
        return readRegisters(0x04, address, count, out);
    }

    bool writeRegisters(int address, const std::vector<uint16_t> &data) override
    {
        std::lock_guard<std::mutex> lock(mutex_);

        try
        {
            if (data.empty())
            {
                std::cerr << "[Modbus] data 为空" << std::endl;
                return false;
            }

            std::vector<uint8_t> request;
            uint8_t function = (data.size() == 1) ? 0x06 : 0x10;

            if (function == 0x06)
            {
                // 写单寄存器：id 06 addrHi addrLo valHi valLo CRClo CRChi
                uint16_t val = data[0];
                request = {
                    (uint8_t)slave_id_, 0x06,
                    (uint8_t)(address >> 8), (uint8_t)(address & 0xFF),
                    (uint8_t)(val >> 8), (uint8_t)(val & 0xFF)};
            }
            else
            {
                // 写多个寄存器：id 10 addrHi addrLo qtyHi qtyLo byteCount data... CRC
                if (data.size() > 123)
                {
                    std::cerr << "[Modbus] 写入寄存器数量超过 123" << std::endl;
                    return false;
                }
                size_t byte_count = data.size() * 2;
                if (byte_count > 255)
                {
                    std::cerr << "[Modbus] byte_count 超过 255" << std::endl;
                    return false;
                }

                request = {
                    (uint8_t)slave_id_, 0x10,
                    (uint8_t)(address >> 8), (uint8_t)(address & 0xFF),
                    (uint8_t)(data.size() >> 8), (uint8_t)(data.size() & 0xFF),
                    (uint8_t)byte_count};
                for (const auto &val : data)
                {
                    request.push_back((uint8_t)(val >> 8));   // 高字节
                    request.push_back((uint8_t)(val & 0xFF)); // 低字节
                }
            }

            // CRC16 (RTU)
            uint16_t crc = compute_modbus_crc16(request.data(), request.size());
            request.push_back(crc & 0xFF);
            request.push_back((crc >> 8) & 0xFF);

            // 发送
            write(socket_, boost::asio::buffer(request));

            // 仍按固定 8 字节读取（正常响应 0x06/0x10 都是 8B）
            std::vector<uint8_t> response(8);
            read(socket_, boost::asio::buffer(response));

            // CRC 校验（对前 6 字节算 CRC）
            uint16_t crc_recv = response[6] | (response[7] << 8);
            uint16_t crc_calc = compute_modbus_crc16(response.data(), 6);
            if (crc_recv != crc_calc)
            {
                std::cerr << "[Modbus] 写寄存器 CRC 错误" << std::endl;
                return false;
            }

            // 功能码校验
            if (response[0] != (uint8_t)slave_id_)
            {
                std::cerr << "[Modbus] 从站ID不匹配" << std::endl;
                return false;
            }
            if (response[1] != function)
            {
                std::cerr << "[Modbus] 功能码不匹配，期望 0x"
                          << std::hex << int(function)
                          << " 实际 0x" << int(response[1]) << std::dec << std::endl;
                return false;
            }

            // 可选：校验回显内容
            if (function == 0x06)
            {
                // 回显地址与值
                uint16_t addr_echo = (response[2] << 8) | response[3];
                if (addr_echo != (uint16_t)address)
                {
                    std::cerr << "[Modbus] 地址回显不一致" << std::endl;
                    // 不强制失败，按需处理
                }
            }
            else
            { // 0x10
                // 回显地址与数量
                uint16_t addr_echo = (response[2] << 8) | response[3];
                uint16_t qty_echo = (response[4] << 8) | response[5];
                if (addr_echo != (uint16_t)address || qty_echo != (uint16_t)data.size())
                {
                    std::cerr << "[Modbus] 地址/数量回显不一致" << std::endl;
                    // 不强制失败，按需处理
                }
            }

            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << "[Exception] 写寄存器失败: " << e.what() << std::endl;
            return false;
        }
    }

    bool writeCoil(int address, bool value) override
    {
        std::lock_guard<std::mutex> lock(mutex_);

        try
        {
            unsigned char v = value ? 0xFF : 0X00;
            std::vector<uint8_t> request = {
                (uint8_t)slave_id_, 0x05, (uint8_t)(address >> 8),
                (uint8_t)(address & 0xFF), v, 0x00};

            uint16_t crc = compute_modbus_crc16(request.data(), request.size());
            request.push_back(crc & 0xFF);
            request.push_back((crc >> 8) & 0xFF);

            write(socket_, boost::asio::buffer(request));

            std::vector<uint8_t> response(8);
            read(socket_, boost::asio::buffer(response));

            uint16_t crc_recv = response[6] | (response[7] << 8);
            uint16_t crc_calc = compute_modbus_crc16(response.data(), 6);
            if (crc_recv != crc_calc)
            {
                std::cerr << "[Modbus] 写单个线圈 CRC 错误" << std::endl;
                return false;
            }

            return response[0] == slave_id_ && response[1] == 0x05;
        }
        catch (const std::exception &e)
        {
            std::cerr << "[Exception] 写单个线圈失败: " << e.what() << std::endl;
            return false;
        }
    }

    std::string info() const override
    {
        std::ostringstream oss;
        oss << "TransparentTransportWrapper(RTU-over-TCP@" << ip_ << ":" << port_
            << ", slave_id=" << slave_id_ << ")";
        return oss.str();
    }
    void setSlaveId(int id) const override { slave_id_ = id; }

private:
    bool readRegisters(uint8_t function_code, int address, int count, std::vector<uint16_t> &out)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        try
        {
            // 构造请求帧
            std::vector<uint8_t> request = {
                (uint8_t)slave_id_, function_code,
                (uint8_t)(address >> 8), (uint8_t)(address & 0xFF),
                (uint8_t)(count >> 8), (uint8_t)(count & 0xFF)};

            uint16_t crc = compute_modbus_crc16(request.data(), request.size());
            request.push_back(crc & 0xFF);        // CRC低位
            request.push_back((crc >> 8) & 0xFF); // CRC高位

            // 发送
            write(socket_, boost::asio::buffer(request));

            // 接收响应头（3字节）
            std::vector<uint8_t> header(3);
            read(socket_, boost::asio::buffer(header));

            if (header[0] != slave_id_ || header[1] != function_code)
            {
                std::cerr << "[Error] 地址或功能码不匹配" << std::endl;
                return false;
            }

            int byte_count = header[2];
            if (byte_count != count * 2)
            {
                std::cerr << "[Error] 响应长度异常: byte_count=" << byte_count
                          << std::endl;
                return false;
            }

            // 接收数据 + CRC
            std::vector<uint8_t> data_crc(byte_count + 2);
            read(socket_, boost::asio::buffer(data_crc));

            // CRC 校验
            std::vector<uint8_t> full;
            full.insert(full.end(), header.begin(), header.end());
            full.insert(full.end(), data_crc.begin(), data_crc.end());

            uint16_t crc_recv = data_crc[byte_count] | (data_crc[byte_count + 1] << 8);
            uint16_t crc_calc = compute_modbus_crc16(full.data(), full.size() - 2);
            if (crc_recv != crc_calc)
            {
                std::cerr << "[Error] CRC校验失败: recv=" << std::hex << crc_recv
                          << ", calc=" << crc_calc << std::dec << std::endl;
                return false;
            }

            // 解析寄存器数据
            out.resize(count);
            for (int i = 0; i < count; ++i)
            {
                out[i] = (data_crc[2 * i] << 8) | data_crc[2 * i + 1];
            }

            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << "[Exception] Modbus读取失败: " << e.what() << std::endl;
            return false;
        }
    }

private:
    std::string ip_;
    int port_;
    mutable int slave_id_;
    mutable std::mutex mutex_;

    boost::asio::io_service io_;
    boost::asio::ip::tcp::socket socket_;
};

#endif // TRANSPARENT_TRANSPORT_WRAPPER_HPP
