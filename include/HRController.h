#ifndef HRCONTROLLER_H
#define HRCONTROLLER_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include <ArduinoEigen.h>

class HRController
{
public:
    HRController()
    {
        theta_hips << 0.0f, 0.0f;
    }

    void attachServos(int left_pin, int right_pin)
    {
        ESP32PWM::allocateTimer(0);
        ESP32PWM::allocateTimer(1);

        left_servo.setPeriodHertz(50);
        right_servo.setPeriodHertz(50);

        left_servo.attach(left_pin, 500, 2500);
        right_servo.attach(right_pin, 500, 2500);

        delay(500);
        Eigen::Vector2f init_theta;
        init_theta << 0.0f, 0.0f;
        controlHipServos(init_theta);
    }

    void printStatus()
    {
        Serial.printf("RH: %.2f deg | LH: %.2f deg\n",
                      theta_hips(0) * 180.0f / M_PI,
                      theta_hips(1) * 180.0f / M_PI);
    }

    void controlHipServos(const Eigen::Vector2f theta_hips_)
    {
        theta_hips = theta_hips_;

        float deg_R = theta_hips(0) * 180.0f / M_PI;
        float deg_L = theta_hips(1) * 180.0f / M_PI;

        // Servo.write() 는 0-180도 범위만을 받기 때문에
        // 270도 서보 보정 비율 적용
        int angle_RH = constrain(90 - (int)(deg_R * 90.0f / 135.0f), SERVO_MIN, SERVO_MAX);
        int angle_LH = constrain(90 - (int)(deg_L * 90.0f / 135.0f), SERVO_MIN, SERVO_MAX);

        // 예컨대, deg_R = deg_L = 0 으로 초기화하면 write(90)로 중앙값 출력
        right_servo.write(angle_RH);
        left_servo.write(angle_LH);
    }

    void controlHipServos2(const Eigen::Vector2f theta_hips_)
    {
        theta_hips = theta_hips_;

        // 1. 라디안 → 도 변환 후 ±55도 안전 범위 제한
        float deg_R = constrain(theta_hips(0) * 180.0f / M_PI, DEG_MIN, DEG_MAX);
        float deg_L = constrain(theta_hips(1) * 180.0f / M_PI, DEG_MIN, DEG_MAX);

        // 2. 270도 서보 절대 각도 계산 (Center = 135도)
        // POL이 각도 반전을 일으키므로, 부호를 일치시켜야함    [cite: POL.h 431-435]
        // float target_deg_RH = 135.0f - deg_R;
        // float target_deg_LH = 135.0f + deg_L;    // 수동제어
        float target_deg_RH = 135.0f - deg_R;
        float target_deg_LH = 135.0f - deg_L; // 자동제어

        // 3. 펄스 폭 계산 (0~270도 → 500~2500µs)
        float us_RH = (target_deg_RH / 270.0f) * 2000.0f + 500.0f;
        float us_LH = (target_deg_LH / 270.0f) * 2000.0f + 500.0f;

        // 4. 펄스 폭 하드웨어 한계 제한
        us_RH = constrain(us_RH, 500, 2500);
        us_LH = constrain(us_LH, 500, 2500);

        right_servo.writeMicroseconds((int)round(us_RH));
        left_servo.writeMicroseconds((int)round(us_LH));
    }

private:
    // 안전 범위 (도 단위)

    const int SERVO_C = 90;
    const int EQUIV_MAX = 37;   // 55  * (90 / 135) = 36.67
    const int SERVO_MIN = 53;   // write(90 - 37)
    const int SERVO_MAX = 127;  // write(90 + 37)

    static constexpr float DEG_MIN = -55.0f;
    static constexpr float DEG_MAX = 55.0f;
    Servo left_servo;
    Servo right_servo;
    Eigen::Vector2f theta_hips;
};

#endif