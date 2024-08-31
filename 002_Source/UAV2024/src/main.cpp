/* ====================================================================
 * Include files
 * ==================================================================== */
// Arduino標準ライブラリ
#include <Arduino.h>
// 自作ライブラリ
#include "config.h"
#include "SbusReceiver.h"
#include "servo.h"

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

    /* Servo */
    servo_Init();

    /* タイマー割込み */
    Timer1.priority(190);
    Timer1.begin(task_1ms, 1000);

    Timer2.priority(200);
    Timer2.begin(task_20ms, 20000);
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
    std::vector<float> servo_ref_angle_rad(4);
    std::vector<float> servo_act_angle_rad(4);
    std::vector<float> servo_duty_norm(4);

    /* 受信データの取得 */
    command = sbusReceiver.getCommand();

    /* T6LのSWがONの場合に更新 */
    if (command.sw > SWITCH_THRESHOLD)
    {
        /* T6Lのつまみの値を目標値として使用 */
        for (int i = 0; i < CONFIG_SERVO_NUM; i++)
        {
            servo_ref_angle_rad[i] = command.knob * 360 * DEG_TO_RAD;
        }
    }

    /* サーボの制御 */
    servo_act_angle_rad = servo_ReadSensor();
    servo_duty_norm     = servo_Control(servo_ref_angle_rad, servo_act_angle_rad);

    /* モータへ出力 */
    servo_Output(servo_duty_norm);
}
/************************************************************************/
void task_20ms()
{
    T6L_Command command;

    /* 受信データの取得 */
    command = sbusReceiver.getCommand();

    /* 受信データの表示 */
    Serial.print(command.aileron);
    Serial.print("\t");
    Serial.print(command.elevator);
    Serial.print("\t");
    Serial.print(command.knob);
    Serial.print("\t");
    Serial.print(command.rudder);
    Serial.print("\t");
    Serial.print(command.sw);
    Serial.print("\t");
    Serial.print(command.throttle);
    Serial.print("\n");
}
