/* ====================================================================
 * Include files
 * ==================================================================== */
// Arduino標準ライブラリ
#include <Arduino.h>
// 自作ライブラリ
#include "config.h"
#include "SbusReceiver.h"
#include "drone.h"
#include "servo.h"
#include "thrust.h"

/* ====================================================================
 * Class declarations
 * ==================================================================== */
SbusReceiver  sbusReceiver(&Serial1);
IntervalTimer Timer1;
IntervalTimer Timer2;

/* ====================================================================
 * Prototype declarations
 * ==================================================================== */
void task_1ms();
void task_20ms();

/* ====================================================================
 * Public functions
 * ==================================================================== */
void setup()
{
    /* PCとのシリアル通信 */
    Serial.begin(115200);

    /* SBUS */
    sbusReceiver.begin();

    /* drone */
    drone_Init();

    /* Servo */
    servo_Init();

    /* Thrust */
    thrust_Init();

    /* タイマー割込み */
    Timer1.priority(190);
    Timer1.begin(task_1ms, CONFIG_CONTROL_CYCLE_sec * CONFIG_S_TO_US);

    Timer2.priority(200);
    Timer2.begin(task_20ms, CONFIG_POSITION_CONTROL_PERIOD_sec * CONFIG_S_TO_US);
}
/************************************************************************/
void loop()
{
    /* 受信データの更新 */
    sbusReceiver.update();
}
/************************************************************************/
void task_1ms()
{
    T6L_Command        command;
    std::vector<float> ref_servo(4);
    std::vector<float> cur_servo(4);
    std::vector<float> duty_servo(4);
    std::vector<float> ref_thrust(4);
    std::vector<float> duty_thrust(4);

    // 指令値の取得
    command = sbusReceiver.getCommand();

    // 機体の制御
    drone_Control(command);
    ref_thrust = drone_getThrust();
    ref_servo  = drone_getServoAngle();

    // 推力の制御
    duty_thrust = thrust_Control(ref_thrust, command);

    // サーボの制御
    cur_servo  = servo_ReadSensor();
    duty_servo = servo_Control(ref_servo, cur_servo);

    // モータへ出力
    thrust_Output(duty_thrust);
    servo_Output(duty_servo);

    // for debug
    // Serial.print(ref_servo[0] * RAD_TO_DEG);
    // Serial.print("\t");
    // Serial.print(ref_servo[1] * RAD_TO_DEG);
    // Serial.print("\t");
    // Serial.print(ref_servo[2] * RAD_TO_DEG);
    // Serial.print("\t");
    // Serial.print(ref_servo[3] * RAD_TO_DEG);
    // Serial.print("\t");

    // Serial.print(duty_servo[0]);
    // Serial.print("\t");
    // Serial.print(duty_servo[1]);
    // Serial.print("\t");
    // Serial.print(duty_servo[2]);
    // Serial.print("\t");
    // Serial.print(duty_servo[3]);
    // Serial.print("\n");
}
/************************************************************************/
void task_20ms()
{
    // for debug
    // T6L_Command command;

    // command = sbusReceiver.getCommand();

    // Serial.print(command.aileron);
    // Serial.print("\t");
    // Serial.print(command.elevator);
    // Serial.print("\t");
    // Serial.print(command.knob);
    // Serial.print("\t");
    // Serial.print(command.rudder);
    // Serial.print("\t");
    // Serial.print(command.sw);
    // Serial.print("\t");
    // Serial.print(command.throttle);
    // Serial.print("\n");
}
