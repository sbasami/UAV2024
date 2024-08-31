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

/* ====================================================================
 * Class declarations
 * ==================================================================== */
SbusReceiver  sbus_c(&Serial1);
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

/* ====================================================================
 * Prototype declarations
 * ==================================================================== */
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

    /* タイマー割込み */
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
void task_20ms()
{
    T6L_Command command;

    /* 受信データの取得 */
    command = sbus_c.getCommand();

    // 現在角度の取得
    for (int i = 0; i < 4; i++)
    {
        servoEncoder[i].readSensor();
    }

    servoMotor[3].outputPWM(0.1);

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
