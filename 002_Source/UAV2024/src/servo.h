#ifndef SERVO_H
#define SERVO_H

/* ====================================================================
 * Include files
 * ==================================================================== */
// C++標準ライブラリ
#include <vector>

/* ====================================================================
 * Prototype declarations
 * ==================================================================== */
void               servo_Init();
std::vector<float> servo_ReadSensor();
std::vector<float> servo_Control(std::vector<float> ref_angle_rad, std::vector<float> act_angle_rad);
void               servo_Output(std::vector<float> duty_norm);

#endif /* SERVO_H */