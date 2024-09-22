#ifndef THRUSTCONTROLLER_H
#define THRUSTCONTROLLER_H

/* ====================================================================
 * Class declarations
 * ==================================================================== */
/**
 * @brief ThrustControllerクラス
 * @par 説明
 * 以下の処理を提供する
 * - FF制御の計算
 * - FF制御の計算結果の取得
 */
class ThrustController
{
private:
    float retFF_;
public:
    ThrustController();
    void calculateFF(float ref_thrust_N);
    float getFF();
};

#endif /* THRUSTCONTROLLER_H */