std_msgs/Header joint
int8 motor_id
# MIT/POS/VEL 0,1,2
# pos：PD控制器 ve：PI控制器
int8 ctrl_type
# 扭矩
float32 torque
# 位置
float32 angle
# 角速度
float32 ang_vel
float32 kp
float32 kd
#PI控制器 非MIT
float32 ki
