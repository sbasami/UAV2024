/* ====================================================================
 * Include files
 * ==================================================================== */
#include "ThrustController.h"

/* ====================================================================
 * Public functions
 * ==================================================================== */
/**
 * @brief ThrustControllerコンストラクタ関数
 */
ThrustController::ThrustController()
{
    // Do nothing
}

/**
 * @brief FF制御計算関数
 * @param [in] ref_thrust_N 目標推力[N]
 */
void ThrustController::calculateFF(float ref_thrust_N)
{
    float ref_T;  // 数式が長くなるのを防ぐためのtemp変数
    float ret_ff;

    ref_T = ref_thrust_N;

    // Excelによる3次多項式近似
    // x : 推力, y : Duty比[0 ~ 1] ← 左記の関係を実験で求めた
    // y = 0.3264 * x^3 - 0.7585 * x^2 + 0.9477 * x + 0.1156
    // ret_ff = (0.3264 * ref_T * ref_T * ref_T) - (0.7585 * ref_T * ref_T) + (0.9477 * ref_T) + 0.1156;
    ret_ff = ref_T;

    // ====================================================================
    // 特記事項
    // 近似式の関係で推力が0N要求でもDuty比が0.1156になるのでモータが回転してしまう
    // 推力要求が0Nの場合は強制的に0とする
    // ====================================================================
    if (0 == ref_T) ret_ff = 0;

    // 出力制限
    if (ret_ff > 1)
        ret_ff = 1;
    else if (ret_ff < 0)
        ret_ff = 0;

    retFF_ = ret_ff;
};

/**
 * @brief FF制御計算結果取得関数
 * @return 推力の操作量
 */
float ThrustController::getFF()
{
    return retFF_;
}