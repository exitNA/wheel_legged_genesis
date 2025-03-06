#include "DMMotor.h"
#include "PCAN.hpp"
int main(int argc, char **argv) {
  PCAN pcan;
  if (!pcan.initPCAN(CAN1, BAUD_1MBPS))
    std::cout << "init pcan false" << std::endl;
  DMMotor dm;
  int motor_id = 0x05;
  pcan.send(CAN1, dm.enableMotor(motor_id, true));

  auto loc = dm.locomotion(motor_id, 0.0, 0, 2.0, 0.0, 0.5);
  for (int i = 0; i < 50; i++) {
    auto [is_read, can_msg] = pcan.read(CAN1);
    if (is_read) {
      DMCANMsg(can_msg).print();
      // auto decode = dm.decode(can_msg);
      // decode.print();
    }
    pcan.send(CAN1, loc);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  pcan.send(CAN1, dm.enableMotor(motor_id, false));

  return 0;
}