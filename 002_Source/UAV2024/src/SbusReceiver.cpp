/* ====================================================================
 * Include files
 * ==================================================================== */
#include "SbusReceiver.h"

/* ====================================================================
 * Public functions
 * ==================================================================== */
/**
 * @brief SbusReceiverコンストラクタ関数
 */
SbusReceiver::SbusReceiver(HardwareSerial *bus)
    : SbusRx(bus)
{
    int ch_size = SbusRx::rx_channels().size();

    // メンバ変数の初期化
    recvChannel_.assign(ch_size, 0);

    recvChannel_[INDEX_RUDDER]   = 0.5 * SBUS_RESOLUSION_MAX;  // 初期値はニュートラル値
    recvChannel_[INDEX_ELEVATOR] = 0.5 * SBUS_RESOLUSION_MAX;  // 初期値はニュートラル値
    recvChannel_[INDEX_THROTTLE] = 0;                          // 初期値は最小値
    recvChannel_[INDEX_AILERON]  = 0.5 * SBUS_RESOLUSION_MAX;  // 初期値はニュートラル値
    recvChannel_[INDEX_SWITCH]   = 0;                          // 初期値は最小値
    recvChannel_[INDEX_KNOB]     = 0;                          // 初期値は最小値
}

/**
 * @brief 初期化関数
 */
void SbusReceiver::begin()
{
    SbusRx::Begin();
}

/**
 * @brief SBUS受信更新関数
 */
void SbusReceiver::update()
{
    int ch_size = SbusRx::rx_channels().size();

    if (SbusRx::Read())
    {
        for (int i = 0; i < ch_size; i++)
        {
            recvChannel_[i] = SbusRx::rx_channels()[i];
        }
    }
}

/**
 * @brief 指令データ取得関数
 * @return プロポからの指令値[0 ~ 1]
 */
T6L_Command SbusReceiver::getCommand()
{
    T6L_Command recv_command;

    // 11bit分解能(0 ~ 2047)を正規化(0 ~ 1)して値を渡す
    recv_command.rudder   = (float)recvChannel_[INDEX_RUDDER] / SBUS_RESOLUSION_MAX;
    recv_command.elevator = (float)recvChannel_[INDEX_ELEVATOR] / SBUS_RESOLUSION_MAX;
    recv_command.throttle = (float)recvChannel_[INDEX_THROTTLE] / SBUS_RESOLUSION_MAX;
    recv_command.aileron  = (float)recvChannel_[INDEX_AILERON] / SBUS_RESOLUSION_MAX;
    recv_command.sw       = (float)recvChannel_[INDEX_SWITCH] / SBUS_RESOLUSION_MAX;
    recv_command.knob     = (float)recvChannel_[INDEX_KNOB] / SBUS_RESOLUSION_MAX;

    return recv_command;
}
