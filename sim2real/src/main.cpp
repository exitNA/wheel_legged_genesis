#include "PCAN.hpp"
#include "MiMotor.h"
int main(int argc, char **argv) {
  PCAN pcan;
  if (!pcan.initPCAN(CAN1, BAUD_1MBPS))
    std::cout << "init pcan false" << std::endl;

  MiMotor mi;

  TPCANMsg motor_enable = mi.enableMotor(127, true);
  pcan.send(CAN1, motor_enable);

  auto loc = mi.locomotion(127, 0.0, 0.0, 2.0, 0.0, 0.3);
  pcan.send(CAN1, loc);

  //   TPCANMsg send_msg = mi.set_parameter(127, motor_indexs::spd_ref, 5.2);
  //   pcan.send(CAN1, send_msg);

  //   send_msg = mi.set_parameter(127, motor_indexs::limit_cur, 23);
  //   pcan.send(CAN1, send_msg);
  //   std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  while (true) {
    auto [is_read, can_msg] = pcan.read(CAN1);
    if (is_read) {
        auto decode = mi.decode(can_msg);
        decode.print();
    }
    pcan.send(CAN1, loc);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  
  std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  pcan.send(CAN1, mi.enableMotor(127, false));
  return 0;
}