#pragma once
#include "Motor.hpp"
#include "PCANBasic.h"
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -12.0f
#define T_MAX 12.0f

enum class com_type {
  Loc = 1, // 运控模式
  MotorCallBack = 2,
  Enable = 3,          // 使能
  Stop = 4,            // 停止
  SetParameter = 18,   // 掉电消失
  MotorID = 7,         // 设置电机ID
  SetFixParameter = 8, // 掉电不消失
}; // 通讯类型

// 通讯类型 SetParameter
enum class motor_indexs {
  /*-----------读写-----------*/
  run_mode = 0X7005,      // 0: 运控模式1: 位置模式 2: 速度模式 3: 电流模式
  iq_ref = 0X7006,        // 电流模式Iq指令
  spd_ref = 0X700A,       // 转速模式转速指令
  imit_torque = 0X700B,   // 转矩限制
  cur_kp = 0X7010,        // 电流的 Kp
  cur_ki = 0X7011,        // 电流的 Ki
  cur_filt_gain = 0X7014, // 电流滤波系数filt_gain
  loc_ref = 0X7016,       // 位置模式角度指令
  limit_spd = 0X7017,     // 位置模式速度设置
  limit_cur = 0X7018,     // 速度位置模式电流设置
  mechPos = 0x7019,       // 负载端计圈机械角度
  iqf = 0x701A,           // iq 滤波值
  mechVel = 0x701B,       // 负载端转速
  VBUS = 0x701C,          // 母线电压
  rotation = 0x701D,      // 圈数

}; // 电机功能码

// 通讯类型 SetFixParameter
enum class fix_parameter_indexs {
  /*-----------读写-----------*/
  Name = 0X0000,
  BarCode = 0X0001,
  MechPos_init = 0X2006,  // 初始多圈时的参考角度
  limit_torque = 0X2007,  // 转矩限制
  I_FW_MAX = 0X2008,      // 弱磁电流值，默认 0
  CAN_TIMEOUT = 0X200c,   // can 超时阈值，默认 0
  motorOverTemp = 0X200d, // 电机保护温度值，temp（度）*10
  overTempTime = 0X200e,  // 过温保护时间，time（s）
  GearRatio = 0X200f,     // 传动比 原装7.75
  cur_filt_gain = 0X2011, // 电流滤波系数filt_gain
  cur_kp = 0X2012,        // 电流 kp 0.025
  cur_ki = 0X2013,        // 电流 ki 0.0258
  spd_kp = 0X2014,        // 速度 kp 2
  spd_ki = 0X2015,        // 速度 ki 0.021
  loc_kp = 0X2016,        // 位置 kp 30
  spd_filt_gain = 0X2017, // 速度滤波参数 0.1
  limit_spd = 0X2018,     // 位置模式速度限制 2
  limit_cur = 0X2019,     // 位置、速度模式电流限制 23
}; // 电机功能码

struct MI_EXT_ID {
  uint32_t motor_id : 8; // 只占8位
  uint32_t data : 16;
  uint32_t com_type : 5;
  uint32_t res : 3;
  void print() {
    std::cout << "  Device ID: " << motor_id << std::endl;
    std::cout << "  Data: " << data << std::endl;
    std::cout << "  ComType: " << com_type << std::endl;
  }
  DWORD toEXTID() {
    return (com_type << 24) | (data << 8) | motor_id;
    ;
  }
};

class MiCANMsg : public TPCANMsg {
public:
  MiCANMsg() = default;
  MiCANMsg(const TPCANMsg &base_msg) : TPCANMsg(base_msg) {}
  void print() {
    struct MI_EXT_ID *id_info = (struct MI_EXT_ID *)&this->ID;
    id_info->print();
    std::cout << "Data: ";
    for (size_t i = 0; i < 8; i++) {
      std::cout << " 0x" << std::hex << std::uppercase << std::setw(2)
                << std::setfill('0') << static_cast<int>(this->DATA[i]);
    }
    std::cout << std::dec << std::endl;
  }
};

class MiMotor {
public:
  MiMotor();
  ~MiMotor();

  MiCANMsg enableMotor(uint32_t motor_id, bool enable,
                       bool clear_fault = false);
  Motor decode(TPCANMsg msg);

  MiCANMsg set_parameter(uint32_t motor_id, motor_indexs index,
                         float parameter);
  // 运动控制
  MiCANMsg locomotion(uint32_t motor_id, float torque, float MechPosition,
                      float speed, float kp, float kd);
  // 设置固定参数之后要确认参数，全部设置完之后确定就可以了
  MiCANMsg set_fix_parameter(uint32_t motor_id, fix_parameter_indexs index,
                             float parameter);
  MiCANMsg ok_fix_parameter(uint32_t motor_id);

private:
  int float_to_uint(float x, float x_min, float x_max, int bits);
};
