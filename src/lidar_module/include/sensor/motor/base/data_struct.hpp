#pragma once

#include <cstdint>

namespace motor_base_frame
{

    // 实时数据结构
    struct RealtimeData
    {
        double single_turn_deg{0.0}; // 单圈角度（度）
        double multi_turn_deg{0.0};  // 多圈角度（度）
        double velocity_rpm{0.0};    // 速度（RPM）
        double q_current_a{0.0};     // Q轴电流（A）
        double bus_voltage_v{0.0};   // 母线电压（V）
        double bus_current_a{0.0};   // 母线电流（A）
        double temperature_c{0.0};   // 温度（℃）
        uint8_t run_state{0};        // 运行状态
        bool motor_enabled{false};   // 电机使能状态
        uint8_t fault_code{0};       // 故障码
    };

    enum class ControlMode : int16_t
    {
        Current = 1,              // 电流控制
        Velocity = 2,             // 速度控制
        AbsolutePosition = 3,     // 绝对位置控制
        RelativePosition = 4,     // 相对位置控制
        RunShortestHomePosition = 5 // 最短距离回零点
    };

    // ============ 寄存器地址定义 ============
    static constexpr int DEVICE_RESTART = 320;               // 0x0140 驱动复位
    static constexpr int RESET_FAULT = 321;                  // 0x0141 故障清除
    static constexpr int SET_ZERO_BY_CURRENT_POSITION = 322; // 0x0142 设置当前位置为原点
    static constexpr int RUN_SHORTES_HOME_POSITION = 325;    // 最短距离回零点
    static constexpr int REG_CONTROL_WORD = 384;             // 0x0180 使能/失能
    static constexpr int REG_CONTROL_MODE = 385;             // 0x0181 控制模式
    static constexpr int REG_VELOCITY_CTRL_PARAM = 390;      // 0x0186 速度控制参数
    static constexpr int REG_ACCELERATION = 392;             // 0x0188 加速度（2字）
    static constexpr int REG_ABSOLUTE_POSITION_VALUE = 394;  // 0x018A 绝对位置值（2字）
    static constexpr int REG_RELATIVE_POSITION_VALUE = 396;  // 0x018C 相对位置值（2字）

    // 实时数据寄存器
    static constexpr int REG_SINGLE_TURN_ANGLE = 128; // 0x0080 单圈角度
    static constexpr int REG_MULTI_TURN_ANGLE = 129;  // 0x0081 多圈角度（2字）
    static constexpr int REG_VELOCITY = 131;          // 0x0083 速度（2字）
    static constexpr int REG_Q_CURRENT = 133;         // 0x0085 Q轴电流
    static constexpr int REG_BUS_VOLTAGE = 134;       // 0x0086 母线电压
    static constexpr int REG_BUS_CURRENT = 135;       // 0x0087 母线电流
    static constexpr int REG_TEMPERATURE = 136;       // 0x0088 工作温度
    static constexpr int REG_RUN_STATE = 137;         // 0x0089 运行状态
    static constexpr int REG_MOTOR_STATE = 138;       // 0x008A 电机状态
    static constexpr int REG_FAULT_CODE = 139;        // 0x008B 故障码
}