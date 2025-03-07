#pragma once
#include "magic_enum/magic_enum.hpp"
#include <iostream>
#include <string>

#define RESET "\033[0m"
#define RED "\033[31m"
#define GREEN "\033[32m"
#define YELLOW "\033[33m"
#define BLUE "\033[34m"
#define BOLD "\033[1m"

enum class MotorWarning {
  Normal = 0,
  Unenable,            // 未使能
  NoneCom,             // 通讯异常
  OverTemperature,     // 过温
  Overcurrent,         // 过流
  Overvoltage,         // 超压
  Undervoltage,        // 欠压
  MagneticEncoding,    // 磁编码异常
  HALLEncoding,        // HALL编码异常
  MosOverTemperature,  // MOS管过温
  CoilOverTemperature, // 线圈过温
  OverLoad,            // 过载
};

class Motor {
public:
  int id;
  float angle;
  float number_laps; // 圈数
  float ang_vel;
  float torque;
  float current;
  float temperature; // 摄氏度
  
  MotorWarning warning = MotorWarning::Normal;
  void print() {
    std::cout << "Motor ID: " << id << std::endl;
    std::cout << "Angle: " << angle << std::endl;
    std::cout << "Angular Velocity: " << ang_vel << std::endl;
    std::cout << "Torque: " << torque << std::endl;
    std::cout << "Current: " << current << std::endl;
    std::cout << "Temperature: " << temperature << std::endl;
    if (warning == MotorWarning::Normal) {
      std::cout << GREEN;
    } else {
      std::cout << YELLOW;
    }
    std::cout << "Motor: " << std::string(magic_enum::enum_name(warning))
              << RESET << std::endl;
  }
  Motor *invertMotor() {
    angle = -angle;
    number_laps = -number_laps;
    ang_vel = -ang_vel;
    torque = -torque;
    current = -current;
    return this;
  }
};
