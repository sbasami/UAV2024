/* ====================================================================
 * Include files
 * ==================================================================== */
#include "AttitudeSensor.h"

/* ====================================================================
 * Public functions
 * ==================================================================== */
/**
 * @brief AttitudeSensorコンストラクタ
 * @param [in] bus 使用するSPIのバス情報
 * @param [in] cs_pin_accel 加速度センサのCSピン
 * @param [in] cs_pin_gyro ジャイロセンサのCSピン
 */
AttitudeSensor::AttitudeSensor(SPIClass &bus, uint8_t cs_pin_accel, uint8_t cs_pin_gyro)
    : accel_(bus, cs_pin_accel), gyro_(bus, cs_pin_gyro)
{
    // Do nothing
}

/**
 * @brief 姿勢センサ初期化処理
 * @param [in] sample_frequency Madgwickフィルタのサンプリング周波数[Hz]
 */
void AttitudeSensor::begin(float sample_frequency_Hz)
{
    int status;

    // BMI088初期化
    status = accel_.begin();
    if (status < 0)
    {
        /* 原因不明だが加速度センサの初期化が失敗(CHIP IDを正常に読み出せていない)することがあるのでリトライ処理を設ける */
        status = accel_.begin();
        if (status < 0)
        {
            Serial.println("Accel Initialization Error");
            Serial.println(status);
        }
    }
    status = gyro_.begin();
    if (status < 0)
    {
        Serial.println("Gyro Initialization Error");
        Serial.println(status);
    }

    // Madgwickフィルタ初期化
    madgwick_.begin(sample_frequency_Hz);
}

/**
 * @brief 姿勢データ更新処理
 * @attention Madgwickフィルタ初期化時に設定したサンプリング周波数[Hz]でこの関数を呼ばないとフィルタが正常に更新されない
 */
void AttitudeSensor::update()
{
    std::vector<float> accel(3);
    std::vector<float> gyro(3);

    // センサデータの更新
    accel_.readSensor();
    gyro_.readSensor();

    // センサデータの取得
    accel = getAccel_mss();
    gyro  = getGyro_degs();

    // Madgwickフィルタの更新
    madgwick_.updateIMU(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
}

/**
 * @brief 加速度[m/s^2]取得処理
 * @return 加速度[m/s^2]
 */
std::vector<float> AttitudeSensor::getAccel_mss()
{
    std::vector<float> ret(3);

    ret[0] = accel_.getAccelX_mss();
    ret[1] = accel_.getAccelY_mss();
    ret[2] = accel_.getAccelZ_mss();

    return ret;
}

/**
 * @brief 加速度(絶対座標系)[m/s^2]取得処理
 * @return 加速度(絶対座標系)[m/s^2]
 */
std::vector<float> AttitudeSensor::getGlobalAccel_mss()
{
    std::vector<float> ret(3);
    Eigen::Vector3f    vec{0, 0, 0};
    Eigen::Matrix3f    R;

    R = getRotationMatrix();

    vec.x() = accel_.getAccelX_mss();
    vec.y() = accel_.getAccelY_mss();
    vec.z() = accel_.getAccelZ_mss();

    vec = R * vec;

    ret[0] = vec.x();
    ret[1] = vec.y();
    ret[2] = vec.z();

    return ret;
}

/**
 * @brief 角速度[rad/s]取得処理
 * @return 角速度[rad/s]
 */
std::vector<float> AttitudeSensor::getGyro_rads()
{
    std::vector<float> ret(3);

    ret[0] = gyro_.getGyroX_rads();
    ret[1] = gyro_.getGyroY_rads();
    ret[2] = gyro_.getGyroZ_rads();

    return ret;
}

/**
 * @brief 角速度[deg/s]取得処理
 * @return 角速度[deg/s]
 */
std::vector<float> AttitudeSensor::getGyro_degs()
{
    std::vector<float> ret(3);

    ret[0] = gyro_.getGyroX_rads() * RAD_TO_DEG;
    ret[1] = gyro_.getGyroY_rads() * RAD_TO_DEG;
    ret[2] = gyro_.getGyroZ_rads() * RAD_TO_DEG;

    return ret;
}

/**
 * @brief 現在姿勢[rad]取得処理
 * @return 現在姿勢[rad]
 */
std::vector<float> AttitudeSensor::getRPY_rad()
{
    std::vector<float> ret(3);

    ret[0] = madgwick_.getRollRadians();
    ret[1] = madgwick_.getPitchRadians();
    ret[2] = madgwick_.getYawRadians();

    return ret;
}

/**
 * @brief 現在姿勢[deg]取得処理
 * @return 現在姿勢[deg]
 */
std::vector<float> AttitudeSensor::getRPY_deg()
{
    std::vector<float> ret(3);

    ret[0] = madgwick_.getRoll();
    ret[1] = madgwick_.getPitch();
    ret[2] = madgwick_.getYaw();

    return ret;
}

/**
 * @brief クォータニオン取得処理
 * @return クォータニオン
 */
Eigen::Quaternionf AttitudeSensor::getQuaternion()
{
    Eigen::Quaternionf quat;

    quat.w() = madgwick_.getQw();
    quat.x() = madgwick_.getQx();
    quat.y() = madgwick_.getQy();
    quat.z() = madgwick_.getQz();

    return quat;
}

/**
 * @brief 回転行列取得処理
 * @return 回転行列
 */
Eigen::Matrix3f AttitudeSensor::getRotationMatrix()
{
    Eigen::Quaternionf quat;
    Eigen::Matrix3f    R;

    quat = getQuaternion();
    R    = quat.toRotationMatrix();

    return R;
}
