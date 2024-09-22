#ifndef THRUST_H
#define THRUST_H

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
void               thrust_Init();
std::vector<float> thrust_Control(std::vector<float> ref_thrust_N, T6L_Command command);
void               thrust_Output(std::vector<float> duty_norm);

#endif /* THRUST_H */