#ifndef SBUSRECEIVER_H
#define SBUSRECEIVER_H

/* ====================================================================
 * Include files
 * ==================================================================== */
// C++標準ライブラリ
#include <vector>
// Arduino標準ライブラリ
#include <Arduino.h>
// オープンソースライブラリ
#include "sbus.h"

#define THROTTLE_THRESHOLD (0.2) /**< プロポのスロットル閾値(値の範囲：0~1) */
#define SWITCH_THRESHOLD   (0.5) /**< プロポのスイッチ閾値(値の範囲：0~1) */

typedef struct T6L_COMMAND
{
    float rudder;   /**< ラダー(値の範囲：0~1) */
    float elevator; /**< エレベーター(値の範囲：0~1) */
    float throttle; /**< スロットル(値の範囲：0~1) */
    float aileron;  /**< エルロン(値の範囲：0~1) */
    float sw;       /**< T6Lプロポ左上のスイッチ(値の範囲：0~1) */
    float knob;     /**< T6Lプロポ右上のノブ(値の範囲：0~1) */
} T6L_Command;

/* ====================================================================
 * Class declarations
 * ==================================================================== */
/**
 * @brief Receiverクラス
 * @par 説明
 * 以下の処理を提供する
 * - SBUS受信データの更新
 * - SBUS受信データの取得
 */
class SbusReceiver : public SbusRx
{
   private:
    std::vector<int> recvChannel_; /**< Chごとの受信データ */
   public:
    SbusReceiver(HardwareSerial *bus);
    void        begin();
    void        update();
    T6L_Command getCommand();
};

#endif /* SBUSRECEIVER_H */