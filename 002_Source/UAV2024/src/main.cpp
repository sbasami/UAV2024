/* ====================================================================
 * Include files
 * ==================================================================== */
// Arduino標準ライブラリ
#include <Arduino.h>
// 自作ライブラリ
#include "config.h"
#include "SbusReceiver.h"
#include "ServoEncoder.h"
#include "ServoDriver.h"
#include "ServoController.h"

/* ====================================================================
 * Class declarations
 * ==================================================================== */
SbusReceiver  sbus_c(&Serial1);
IntervalTimer Timer1;
IntervalTimer Timer2;

static ServoEncoder servoEncoder[4] =
    {ServoEncoder(CONFIG_PIN_SERVO1_ENC_A, CONFIG_PIN_SERVO1_ENC_B),
     ServoEncoder(CONFIG_PIN_SERVO2_ENC_A, CONFIG_PIN_SERVO2_ENC_B),
     ServoEncoder(CONFIG_PIN_SERVO3_ENC_A, CONFIG_PIN_SERVO3_ENC_B),
     ServoEncoder(CONFIG_PIN_SERVO4_ENC_A, CONFIG_PIN_SERVO4_ENC_B)};

static ServoDriver servoMotor[4] =
    {ServoDriver(CONFIG_PIN_SERVO1_PWM, CONFIG_PIN_SERVO1_DIR),
     ServoDriver(CONFIG_PIN_SERVO2_PWM, CONFIG_PIN_SERVO2_DIR),
     ServoDriver(CONFIG_PIN_SERVO3_PWM, CONFIG_PIN_SERVO3_DIR),
     ServoDriver(CONFIG_PIN_SERVO4_PWM, CONFIG_PIN_SERVO4_DIR)};

static ServoController servoController[4] =
    {ServoController(0.001),
     ServoController(0.001),
     ServoController(0.001),
     ServoController(0.001)};

T6L_Command command;

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
    sbus_c.begin();

    for (int i = 0; i < 4; i++)
    {
        servoMotor[i].begin();
    }

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
    sbus_c.update();
}
/************************************************************************/
#define SERVO 2
void task_1ms()
{
    // 現在角度の取得
    for (int i = 0; i < 4; i++)
    {
        servoEncoder[i].readSensor();

        if (command.sw > SWITCH_THRESHOLD)
        {
            servoController[i].calculateIPD(command.knob * 360 * 2 * DEG_TO_RAD, servoEncoder[i].getAngle_rad(), 7.4);
            servoMotor[i].outputPWM(servoController[i].getIPD_duty());
        }
        else
        {
            servoController[i].calculateIPD(0, servoEncoder[i].getAngle_rad(), 7.4);
            servoMotor[i].outputPWM(servoController[i].getIPD_duty());
        }
    }

    // servoMotor[SERVO].outputPWM(0.1);
}
/************************************************************************/
void task_20ms()
{
    /* 受信データの取得 */
    command = sbus_c.getCommand();

    /* 受信データの表示 */
    Serial.print(servoEncoder[0].getAngle_deg());
    Serial.print("\t");
    Serial.print(servoEncoder[1].getAngle_deg());
    Serial.print("\t");
    Serial.print(servoEncoder[2].getAngle_deg());
    Serial.print("\t");
    Serial.print(servoEncoder[3].getAngle_deg());
    Serial.print("\t");
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
