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

constexpr float THROTTLE_THRESHOLD = 0.2; /**< プロポのスロットル閾値 */
constexpr float SWITCH_THRESHOLD   = 0.5; /**< プロポのスイッチ閾値 */

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
    static constexpr int SBUS_RESOLUSION_MAX = 0x07FF; /**< SBUSの分解能(11bit)の最大値 */
    // ====================================================================
    // 特記事項
    // 下記はT6L Sportのデータ格納情報なので他のプロポを使用する場合は異なる場合がある
    // ====================================================================
    static constexpr int INDEX_RUDDER   = 0; /**< Rudder情報が格納されている配列Index */
    static constexpr int INDEX_ELEVATOR = 1; /**< Elevator情報が格納されている配列Index */
    static constexpr int INDEX_THROTTLE = 2; /**< Throttle情報が格納されている配列Index */
    static constexpr int INDEX_AILERON  = 3; /**< Aileron情報が格納されている配列Index */
    static constexpr int INDEX_SWITCH   = 4; /**< Switch情報が格納されている配列Index */
    static constexpr int INDEX_KNOB     = 5; /**< Knob情報が格納されている配列Index */
    std::vector<int>     recvChannel_;       /**< Chごとの受信データ */
   public:
    explicit SbusReceiver(HardwareSerial *bus);
    void        begin();
    void        update();
    T6L_Command getCommand();
};

#endif /* SBUSRECEIVER_H */