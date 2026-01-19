/*******************************************************************************
****FilePath: /Photonix/src/motor_module/include/motor_module/r1_motor.h
****Author: mwt 911608720@qq.com
****Date: 2025-08-21 14:59:08
****Description:
****Copyright: 2025 by mwt, All Rights Reserved.
********************************************************************************/
#include <atomic>
#include <thread>
#include <chrono>
#include <iostream>
#include <iomanip>

#include "sensor/motor/base/comm_interface.hpp"
#include "sensor/motor/base/plc_device_base.hpp"
#include "sensor/motor/base/data_struct.hpp"

namespace motor_base
{
    /**
     * @brief 电机控制类
     * 根据寄存器映射表实现的完整功能：
     * - 使能/失能
     * - 清除故障
     * - 设置零位/回零
     * - 绝对位置控制
     * - 速度控制
     * - 读取实时数据
     */
    class MotorBase : public PlcDevice
    {
    public:
        /**
         * @brief 构造函数
         * @param comm 通信接口
         * @param slave_id 从站ID
         * @param order 字节序
         */
        MotorBase(std::shared_ptr<CommInterface> comm, int slave_id, ByteOrder order)
            : PlcDevice(order), comm_(comm), slave_id_(slave_id)
        {
            connect();
        }

        ~MotorBase()
        {
            disconnect();
        }

        /**
         * @brief 执行绝对位置控制并等待到达目标位置
         * @param target_degrees 目标角度（度）
         * @param speed_rpm 运行速度（RPM），默认100
         * @param acceleration_rpmps 加速度（RPM/s），默认50
         * @param position_tolerance 位置容差（度），默认0.5度
         * @param timeout_seconds 超时时间（秒），默认30秒
         * @return 成功返回true，失败或超时返回false
         */
        bool moveAbsoluteAndWait(double target_degrees, double &deviation, double speed_rpm = 5.0, double acceleration_rpmps = 1.0,
                                 double position_tolerance = 0.5, double timeout_seconds = 30.0)
        {
            // 1. 读取当前位置
            motor_base_frame::RealtimeData current_data;
            if (!readRealtimeData(current_data))
            {
                throw std::runtime_error("读取当前电机位置失败");
                return false;
            }

            double current_pos = current_data.multi_turn_deg;
            double distance = std::abs(target_degrees - current_pos);

            // 如果已经在目标位置附近，直接返回
            if (distance < position_tolerance)
            {
                deviation = target_degrees - current_pos;
                return true;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            // 2. 设置速度和加速度参数
            if (!setSpeed(speed_rpm))
            {
                throw std::runtime_error("设置电机速度失败");
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            if (!setAcceleration(acceleration_rpmps))
            {
                throw std::runtime_error("设置电机加速度失败");
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            // 3. 发送绝对位置控制命令
            if (!moveAbsolute(target_degrees))
            {
                throw std::runtime_error("发送绝对位置控制命令失败");
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            // 4. 电机使能
            if (enableMotor() == false)
            {
                throw std::runtime_error("使能电机失败");
                return false;
            }

            // 5. 计算预估时间
            double speed_dps = speed_rpm * 6.0;           // RPM转为度/秒 (1 RPM = 6 度/秒)
            double accel_dps2 = acceleration_rpmps * 6.0; // RPM/s转为度/秒² (1 RPM/s = 6 度/秒²)

            double accel_time = speed_dps / accel_dps2;                         // 加速到最大速度的时间（秒）
            double accel_distance = 0.5 * accel_dps2 * accel_time * accel_time; // 加速段距离（度）

            double estimated_time;
            if (distance < 2 * accel_distance)
            {
                // 距离太短，无法达到最大速度
                estimated_time = 2.0 * std::sqrt(distance / accel_dps2);
            }
            else
            {
                // 正常三段运动
                double uniform_distance = distance - 2 * accel_distance;
                double uniform_time = uniform_distance / speed_dps;
                estimated_time = 2 * accel_time + uniform_time;
            }

            // std::cout << "[moveAbsoluteAndWait] 预估时间: " << estimated_time << " 秒" << std::endl;

            // 6. 先等待预估时间的一半，然后开始轮询
            auto start_time = std::chrono::steady_clock::now();
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(estimated_time * 500)));

            // 7. 轮询检查是否到达
            const int poll_interval_ms = 100; // 100ms轮询一次
            while (true)
            {
                // 检查超时
                auto current_time = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration<double>(current_time - start_time).count();

                if (elapsed > timeout_seconds)
                {
                    throw std::runtime_error("电机位置控制超时");
                    return false;
                }

                // 读取当前位置
                if (!readRealtimeData(current_data))
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(poll_interval_ms));
                    continue;
                }

                double current_error = std::abs(target_degrees - current_data.multi_turn_deg);

                // 检查是否到达
                if (current_error < position_tolerance)
                {
                    deviation = target_degrees - current_data.multi_turn_deg;
                    return true;
                }

                // 继续等待
                std::this_thread::sleep_for(std::chrono::milliseconds(poll_interval_ms));
            }
        }

        // ============ 基础控制功能 ============

        /**
         * @brief 使能电机
         * @return 成功返回true
         */
        bool enableMotor()
        {
            PlcValue value = static_cast<int16_t>(255);
            std::vector<uint16_t> regs = encodeValue(value); // 使用基类编码
            return comm_->writeRegisters(motor_base_frame::REG_CONTROL_WORD, regs);
        }

        /**
         * @brief 失能电机
         * @return 成功返回true
         */
        bool disableMotor()
        {
            PlcValue value = static_cast<int16_t>(0);
            std::vector<uint16_t> regs = encodeValue(value); // 使用基类编码
            return comm_->writeRegisters(motor_base_frame::REG_CONTROL_WORD, regs);
        }

        /**
         * @brief 清除故障
         * @return 成功返回true
         */
        bool clearFault()
        {
            PlcValue value = static_cast<int16_t>(1); // 写1清除故障
            std::vector<uint16_t> regs = encodeValue(value);
            return comm_->writeRegisters(motor_base_frame::RESET_FAULT, regs);
        }

        // ============ 模式控制 ============

        /**
         * @brief 设置控制模式
         * @param mode 控制模式
         * @return 成功返回true
         */
        bool setControlMode(motor_base_frame::ControlMode mode)
        {
            PlcValue value = static_cast<int16_t>(mode);
            std::vector<uint16_t> regs = encodeValue(value);
            return comm_->writeRegisters(motor_base_frame::REG_CONTROL_MODE, regs);
        }
        // ============ 位置控制功能 ============
        /**
         * @brief 设置当前位置为零位
         * @return 成功返回true
         */
        bool setOrigin()
        {
            PlcValue value = static_cast<int16_t>(1); // 写1设置零位
            std::vector<uint16_t> regs = encodeValue(value);
            return comm_->writeRegisters(motor_base_frame::SET_ZERO_BY_CURRENT_POSITION, regs);
        }

        /**
         * @brief 回零（查找原点）
         * @return 成功返回true
         */
        bool returnToOrigin()
        {
            setControlMode(motor_base_frame::ControlMode::RunShortestHomePosition);
            PlcValue value = static_cast<int16_t>(1); // 写1开始回零
            std::vector<uint16_t> regs = encodeValue(value);
            return comm_->writeRegisters(motor_base_frame::RUN_SHORTES_HOME_POSITION, regs);
        }

        /**
         * @brief 绝对位置运动
         * @param degrees 目标角度（度）
         * @return 成功返回true
         */
        bool moveAbsolute(double degrees)
        {
            setControlMode(motor_base_frame::ControlMode::AbsolutePosition);
            int32_t scaled = static_cast<int32_t>(std::llround(degrees * 16384.0 / 360.0)); // 度转编码值
            PlcValue value = scaled;
            std::vector<uint16_t> regs = encodeValue(value);
            return comm_->writeRegisters(motor_base_frame::REG_ABSOLUTE_POSITION_VALUE, regs);
        }

        /**
         * @brief 相对位置运动
         * @param degrees 相对角度（度）
         * @return 成功返回true
         */
        bool moveRelative(double degrees)
        {
            setControlMode(motor_base_frame::ControlMode::RelativePosition);
            int32_t scaled = static_cast<int32_t>(std::llround(degrees * 16384.0 / 360.0)); // 度转编码值
            PlcValue value = scaled;
            std::vector<uint16_t> regs = encodeValue(value);
            return comm_->writeRegisters(motor_base_frame::REG_RELATIVE_POSITION_VALUE, regs);
        }

        // ============ 速度控制功能 ============
        /**
         * @brief 速度控制
         * @param rpm 目标速度（RPM）
         * @return 成功返回true
         */
        bool setSpeed(double rpm)
        {
            setControlMode(motor_base_frame::ControlMode::Velocity);
            int32_t scaled = static_cast<int32_t>(std::llround(rpm * 100.0)); // 100 RPM -> 10000
            PlcValue value = scaled;
            std::vector<uint16_t> regs = encodeValue(value);
            return comm_->writeRegisters(motor_base_frame::REG_VELOCITY_CTRL_PARAM, regs);
        }

        /**
         * @brief 设置加速度
         * @param acceleration 加速度（RPM/s）
         * @return 成功返回true
         */
        bool setAcceleration(double acceleration)
        {
            setControlMode(motor_base_frame::ControlMode::Velocity);
            int32_t scaled = static_cast<int32_t>(std::llround(acceleration * 100.0)); // 1 RPM/s -> 100
            PlcValue value = scaled;
            std::vector<uint16_t> regs = encodeValue(value);
            return comm_->writeRegisters(motor_base_frame::REG_ACCELERATION, regs);
        }

        // ============ 数据读取功能 ============
        /**
         * @brief 读取实时数据
         * @param data 输出实时数据
         * @return 成功返回true
         */
        bool readRealtimeData(motor_base_frame::RealtimeData &data) const
        {
            std::vector<uint16_t> regs;

            // 读取寄存器 128-139 (共12个寄存器)
            if (!comm_->readHoldingRegisters(motor_base_frame::REG_SINGLE_TURN_ANGLE, 12, regs))
            {
                return false;
            }

            // 辅助函数
            auto U16 = [&](int offset) -> uint16_t
            {
                return regs.at(offset);
            };
            auto S16 = [&](int offset) -> int16_t
            {
                return static_cast<int16_t>(U16(offset));
            };
            auto S32 = [&](int offset) -> int32_t
            {
                // 高16位在前
                uint16_t hi = U16(offset);
                uint16_t lo = U16(offset + 1);
                uint32_t u = (static_cast<uint32_t>(hi) << 16) | lo;
                return static_cast<int32_t>(u);
            };

            // 解析数据
            data.single_turn_deg = U16(0) * (360.0 / 16384.0);        // 单圈角度
            data.multi_turn_deg = S32(1) * (360.0 / 16384.0);         // 多圈角度
            data.velocity_rpm = S32(3) * 0.01;                        // 速度
            data.q_current_a = S16(5) * 0.001;                        // Q轴电流
            data.bus_voltage_v = U16(6) * 0.01;                       // 母线电压
            data.bus_current_a = U16(7) * 0.01;                       // 母线电流
            data.temperature_c = static_cast<uint8_t>(U16(8) & 0xFF); // 温度
            data.run_state = static_cast<uint8_t>(U16(9) & 0xFF);     // 运行状态
            data.motor_enabled = (U16(10) & 0xFF) != 0;               // 电机状态
            data.fault_code = static_cast<uint8_t>(U16(11) & 0xFF);   // 故障码

            return true;
        }

        /**
         * @brief 打印实时数据
         * @return 格式化的字符串
         */
        std::string getDeviceInfo() const override
        {
            motor_base_frame::RealtimeData data;
            if (!readRealtimeData(data))
            {
                return "读取实时数据失败";
            }

            char buffer[1024];
            snprintf(buffer, sizeof(buffer),
                     "=== 电机实时数据 ===\n"
                     "单圈角度: %.2f °\n"
                     "多圈角度: %.2f °\n"
                     "机械速度: %.2f RPM\n"
                     "Q轴电流: %.3f A\n"
                     "母线电压: %.2f V\n"
                     "母线电流: %.2f A\n"
                     "工作温度: %.0f ℃\n"
                     "运行状态: %d\n"
                     "电机状态: %s\n"
                     "故障码: 0x%02X\n",
                     data.single_turn_deg,
                     data.multi_turn_deg,
                     data.velocity_rpm,
                     data.q_current_a,
                     data.bus_voltage_v,
                     data.bus_current_a,
                     data.temperature_c,
                     data.run_state,
                     data.motor_enabled ? "使能" : "失能",
                     data.fault_code);

            return std::string(buffer);
        }

    private:
        void connect() override
        {
            if (comm_)
            {
                comm_->connect(slave_id_);
            }
        }

        void disconnect() override
        {
            if (comm_)
            {
                comm_->disconnect();
            }
        }

        void sendRequest() override {}
        void handleResponse() override {}

    private:
        std::shared_ptr<CommInterface> comm_;
        int slave_id_;
    };
};