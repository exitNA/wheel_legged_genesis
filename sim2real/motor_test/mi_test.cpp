#include "PCAN.hpp"
#include "MiMotor.h"
int main(int argc, char **argv) {
  PCAN pcan;
  auto channel = CAN1;
  if (!pcan.initPCAN(channel, BAUD_1MBPS))
    std::cout << "init pcan false" << std::endl;

  MiMotor mi;
  int motor_id = 10;
  // pcan.send(channel, mi.set_can_id(15, motor_id));

  TPCANMsg motor_enable = mi.enableMotor(motor_id, true);
  pcan.send(channel, motor_enable);

  auto loc = mi.locomotion(motor_id, 0.0, 0.0, 1.0, 0.0, 0.1);
  pcan.send(channel, loc);

  //   TPCANMsg send_msg = mi.set_parameter(motor_id, motor_indexs::spd_ref, 5.2);
  //   pcan.send(channel, send_msg);

  //   send_msg = mi.set_parameter(motor_id, motor_indexs::limit_cur, 23);
  //   pcan.send(channel, send_msg);
  //   std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  for(int i=0;i<100;i++){
    auto [is_read, can_msg] = pcan.read(channel);
    if (is_read) {
        // MiCANMsg(can_msg).print();
        auto decode = mi.decode(can_msg);
        decode.print();
        // decode.invertMotor()->print();
    }
    pcan.send(channel, loc);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  
  pcan.send(channel, mi.enableMotor(motor_id, false));
  return 0;
}