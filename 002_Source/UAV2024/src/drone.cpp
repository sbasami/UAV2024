/* ====================================================================
 * Include files
 * ==================================================================== */
#include "drone.h"
// 自作クラスの.hファイル
#include "AttitudePlanner.h"
#include "AttitudeSensor.h"
#include "AttitudeController.h"
#include "PositionController.h"
#include "Distributor.h"
// その他必要な.hファイル
#include "config.h"

/* ====================================================================
 * Class declarations
 * ==================================================================== */
// 機体の姿勢制御に関するクラス
static AttitudePlanner    attitudePlanner;
static AttitudeSensor     attitudeSensor(SPI, CS_PIN_ACCEL, CS_PIN_GYRO);
static AttitudeController attitudeController(CONFIG_CONTROL_CYCLE_sec);
// 機体の位置制御に関するクラス
static PositionController positionController(CONFIG_POSITION_CONTROL_PERIOD_sec);
// 機体の操作量の分配に関するクラス
static Distributor distributor;

std::vector<float> ref_thrust(4);  // プロペラの目標推力
std::vector<float> ref_servo(4);   // サーボの目標角度

/* ====================================================================
 * Public functions
 * ==================================================================== */
/**
 * @brief 初期化処理
 */
void drone_Init()
{
    // 姿勢センサの初期化
    attitudeSensor.begin(CONFIG_CONTROL_FREQUENCY_Hz);

    // チルト飛行の設定
    attitudePlanner.SetTiltFlightTargetAngle(30);
    attitudePlanner.SetTiltFlightUpdateFrequency(CONFIG_CONTROL_FREQUENCY_Hz);
    attitudePlanner.SetTiltFlightTransitionTime(5);
}

/**
 * @brief 機体制御処理
 * @param [in] command プロポからの指令値
 * @return プロペラの目標推力[N]
 */
void drone_Control(T6L_Command command)
{
    Eigen::Matrix3f    cur_R;                 // 現在姿勢の回転行列
    Eigen::Matrix3f    ref_R;                 // 目標姿勢の回転行列
    Eigen::Matrix3f    ref_R_without_offset;  // 目標姿勢の回転行列(オフセットなし)
    std::vector<float> cur_rpy(3);            // 現在姿勢のオイラー角
    std::vector<float> F_attitude(3);         // 機体の姿勢操作量
    std::vector<float> F_position(3);         // 機体の位置操作量

    // 現在姿勢の計算
    attitudeSensor.update();
    cur_R   = attitudeSensor.getRotationMatrix();
    cur_rpy = attitudeSensor.getRPY_deg();

    // 目標姿勢の計算
    attitudePlanner.calculateRefAttitude(command, cur_rpy);
    ref_R                = attitudePlanner.getRefRotMatrix();
    ref_R_without_offset = attitudePlanner.getRefRotMatrixWithoutOffset();

    // 姿勢制御
    attitudeController.calculatePID(ref_R, cur_R);
    F_attitude = attitudeController.getPID();

    // 位置制御
    positionController.calculateFF(command, ref_R_without_offset);
    F_position = positionController.getFF();

    // 操作量の分配
    distributor.calculateThrustAndServoAngle(F_attitude, F_position);
    ref_thrust = distributor.getThrust();
    ref_servo  = distributor.getServoAngle();

    // for debug
    // Serial.print(cur_rpy[0]);
    // Serial.print("\t");
    // Serial.print(cur_rpy[1]);
    // Serial.print("\t");
    // Serial.print(cur_rpy[2]);
    // Serial.print("\t");

    // Serial.print(F_attitude[0]);
    // Serial.print("\t");
    // Serial.print(F_attitude[1]);
    // Serial.print("\t");
    // Serial.print(F_attitude[2]);
    // Serial.print("\t");
    // Serial.print(F_position[0]);
    // Serial.print("\t");
    // Serial.print(F_position[1]);
    // Serial.print("\t");
    // Serial.print(F_position[2]);
    // Serial.print("\t");
}

std::vector<float> drone_getThrust()
{
    return ref_thrust;
}

std::vector<float> drone_getServoAngle()
{
    return ref_servo;
}
