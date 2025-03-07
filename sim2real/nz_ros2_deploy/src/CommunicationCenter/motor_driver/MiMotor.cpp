#include "MiMotor.h"
#include <cstdint>
#include <math.h>
MiMotor::MiMotor()
{}
MiMotor::~MiMotor()
{}

MiCANMsg MiMotor::enableMotor(uint8_t motor_id, bool enable, bool clear_fault)
{
    MiCANMsg msg;
    MI_EXT_ID ext_id;
    ext_id.device_id = motor_id;
    ext_id.data=0;
    if(enable)
        ext_id.com_type=static_cast<uint32_t>(com_type::Enable);
    else
        ext_id.com_type=static_cast<uint32_t>(com_type::Stop);
    msg.ID = ext_id.toEXTID();
    msg.MSGTYPE = PCAN_MESSAGE_EXTENDED;
    msg.LEN = 8;
    if(clear_fault)
        msg.DATA[0] = 1;
    return msg;
}

Motor MiMotor::decode(TPCANMsg msg)
{
    MiCANMsg mi_msg(msg);
    Motor motor;
    auto ext_id = mi_msg.get_ext_id();
    auto mi_back = ext_id->toMI_BACK();
    if(ext_id->com_type == static_cast<uint32_t>(com_type::MotorCallBack))
    {
    motor.id = mi_back->motor_id;
    motor.angle = ((float)(mi_msg.DATA[0] << 8 | mi_msg.DATA[1])-32767.5)/32767.5*4*M_PI;
    motor.ang_vel = ((float)(mi_msg.DATA[2] << 8 | mi_msg.DATA[3])-32767.5)/32767.5*30.0;
    motor.torque = ((float)(mi_msg.DATA[4] << 8 | mi_msg.DATA[5])-32767.5)/32767.5*12.0;
    motor.temperature = (float)(mi_msg.DATA[4] << 8 | mi_msg.DATA[5])*10;
    if(mi_back->undervoltage)
        motor.warning = MotorWarning::Undervoltage;
    else if(mi_back->overcurrent)
        motor.warning = MotorWarning::Overcurrent;
    else if(mi_back->overtemperature)
        motor.warning = MotorWarning::OverTemperature;
    else if(mi_back->magnetic_encoding)
        motor.warning = MotorWarning::MagneticEncoding;
    else if(mi_back->HALL_encoding)
        motor.warning = MotorWarning::HALLEncoding;
    else
        motor.warning = MotorWarning::Normal;
    
    }
    else if(ext_id->com_type == static_cast<uint32_t>(com_type::RequestParameter)){
        
    }
    return motor;
}

MiCANMsg MiMotor::locomotion(uint8_t motor_id,  float torque, float pos, float ang_vel, float kp, float kd)
{
    MiCANMsg msg;
    MI_EXT_ID ext_id;
    ext_id.device_id = motor_id;
    ext_id.data=float_to_uint(torque,T_MIN,T_MAX,16);
    ext_id.com_type=static_cast<uint32_t>(com_type::Loc);
    msg.ID = ext_id.toEXTID();
    msg.MSGTYPE = PCAN_MESSAGE_EXTENDED;
    msg.LEN = 8;
    msg.DATA[0] = float_to_uint(pos,P_MIN,P_MAX,16)>>8;
    msg.DATA[1] = float_to_uint(pos,P_MIN,P_MAX,16);
    msg.DATA[2] = float_to_uint(ang_vel,V_MIN,V_MAX,16)>>8;
    msg.DATA[3] = float_to_uint(ang_vel,V_MIN,V_MAX,16);
    msg.DATA[4] = float_to_uint(kp,KP_MIN,KP_MAX,16)>>8;
    msg.DATA[5] = float_to_uint(kp,KP_MIN,KP_MAX,16);
    msg.DATA[6] = float_to_uint(kd,KD_MIN,KD_MAX,16)>>8;
    msg.DATA[7] = float_to_uint(kd,KD_MIN,KD_MAX,16);
    return msg;
}

MiCANMsg MiMotor::set_parameter(uint8_t motor_id, motor_indexs index,float parameter)
{
    MiCANMsg msg;
    MI_EXT_ID ext_id;
    ext_id.device_id = motor_id;
    ext_id.data=0;
    ext_id.com_type=static_cast<uint32_t>(com_type::SetParameter);
    msg.ID = ext_id.toEXTID();
    msg.MSGTYPE = PCAN_MESSAGE_EXTENDED;
    msg.LEN = 8;
    uint16_t motor_index = static_cast<uint16_t>(index);
    memcpy(msg.DATA, &motor_index, sizeof(uint16_t));
    msg.DATA[2] = 0x00;
    msg.DATA[3] = 0x00;
    memcpy(msg.DATA+4, &parameter, sizeof(float));
    return msg;
}

MiCANMsg MiMotor::set_fix_parameter(uint8_t motor_id, fix_parameter_indexs index,float parameter)
{
    MiCANMsg msg;
    MI_EXT_ID ext_id;
    ext_id.device_id = motor_id;
    ext_id.data=253;
    ext_id.com_type=static_cast<uint32_t>(com_type::SetFixParameter);
    msg.ID = ext_id.toEXTID();
    msg.MSGTYPE = PCAN_MESSAGE_EXTENDED;
    msg.LEN = 8;
    uint16_t motor_index = static_cast<uint16_t>(index);
    memcpy(msg.DATA, &motor_index, sizeof(uint16_t));
    msg.DATA[2] = 0x06;
    msg.DATA[3] = 0x00;
    memcpy(msg.DATA+4, &parameter, sizeof(float));
    return msg;
}

MiCANMsg MiMotor::ok_fix_parameter(uint8_t motor_id)
{
    MiCANMsg msg;
    MI_EXT_ID ext_id;
    ext_id.device_id = motor_id;
    ext_id.data=765;
    ext_id.com_type=static_cast<uint32_t>(com_type::SetFixParameter);
    msg.ID = ext_id.toEXTID();
    msg.MSGTYPE = PCAN_MESSAGE_EXTENDED;
    msg.LEN = 8;
    return msg;
}

MiCANMsg MiMotor::request_parameter(uint8_t motor_id, motor_indexs index)
{
    MiCANMsg msg;
    MI_EXT_ID ext_id;
    ext_id.device_id = motor_id;
    ext_id.data=0;
    ext_id.com_type=static_cast<uint32_t>(com_type::RequestParameter);
    msg.ID = ext_id.toEXTID();
    msg.MSGTYPE = PCAN_MESSAGE_EXTENDED;
    msg.LEN = 8;
    uint16_t motor_index = static_cast<uint16_t>(index);
    memcpy(msg.DATA, &motor_index, sizeof(uint16_t));
    return msg;
}

MiCANMsg MiMotor::set_can_id(uint8_t motor_id, uint8_t id)
{
    MiCANMsg msg;
    MI_EXT_ID ext_id;
    ext_id.device_id = motor_id;
    ext_id.data=id<< 8;
    ext_id.com_type=static_cast<uint32_t>(com_type::SetCANID);
    msg.ID = ext_id.toEXTID();
    msg.MSGTYPE = PCAN_MESSAGE_EXTENDED;
    msg.LEN = 8;
    return msg;
}

MiCANMsg MiMotor::set_zero_point(uint8_t motor_id)
{
    MiCANMsg msg;
    MI_EXT_ID ext_id;
    ext_id.device_id = motor_id;
    ext_id.data=0;
    ext_id.com_type=static_cast<uint32_t>(com_type::SetZero);
    msg.ID = ext_id.toEXTID();
    msg.MSGTYPE = PCAN_MESSAGE_EXTENDED;
    msg.LEN = 8;
    return msg;
}

int MiMotor::float_to_uint(float x, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  if (x > x_max)
    x = x_max;
  else if (x < x_min)
    x = x_min;
  return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

