#pragma once
#include "PCAN.hpp"
#include "gamepad.h"
#include "rclcpp/rclcpp.hpp"
#include "serial.hpp"
#include <atomic>
#include "nz_msg/msg/motor_ctrl.hpp"
#include "nz_msg/msg/motor.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std;

class CommunicationCenter : public rclcpp::Node {
public:
  CommunicationCenter(const std::string &node_name);
  ~CommunicationCenter();

  PCAN *pcan;
  USBHubPort<TPCANHandle> hub_port;
  PChannel pc;
  bool can1_flag = false, can2_flag = false, can3_flag = false,
       can4_flag = false;
  
  rclcpp::Publisher<nz_msg::msg::Motor>::SharedPtr motor_publisher;
  void sendMotor(std::shared_ptr<nz_msg::msg::MotorCtrl> msg);

  thread th;
  atomic<bool> flag{true};

  GamePad *gamepad;
  void initGamePad();
  Serial *serial;

private:
  
  rclcpp::Subscription<nz_msg::msg::MotorCtrl>::SharedPtr sub_motor;

  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer;
};
