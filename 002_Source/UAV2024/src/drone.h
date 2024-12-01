#ifndef DRONE_MAIN_H
#define DRONE_MAIN_H

/* ====================================================================
 * Include files
 * ==================================================================== */
// C++標準ライブラリ
#include <vector>
// 自作クラスの.hファイル
#include "SbusReceiver.h"

/* ====================================================================
 * Prototype declarations
 * ==================================================================== */
void               drone_Init();
void               drone_Control(T6L_Command command);
std::vector<float> drone_getThrust();
std::vector<float> drone_getServoAngle();

#endif /* DRONE_MAIN_H */