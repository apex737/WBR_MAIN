#ifndef VYB_CONTROLLER_H
#define VYB_CONTROLLER_H

#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include "Params.h"
#include "MGServo.h"
#include <algorithm>

/**
 * @class VYBController
 * @brief 제어 시스템의 LQR 기반 동작을 처리하는 클래스
 */
class VYBController
{
private:
  MGServo &ServoRW; ///< 우측 휠 서보
  MGServo &ServoLW; ///< 좌측 휠 서보

  std::vector<Eigen::Matrix<float, 2, 4>> Ks; ///< LQR 게인 행렬들의 벡터
  Eigen::Matrix<float, 2, 4> K;               ///< 현재 사용 중인 LQR 게인
  Eigen::Matrix<float, 2, 1> u;               ///< 제어 입력 벡터

  float iq_factor;       ///< 전류 변환 계수 (A/LSB)
  float torque_constant; ///< 토크 상수 (Nm/A)
  float LW_factor = 0.001043224f;
  float RW_factor = 0.000857902f;

  float AngleFixRate = 0.1;

  float saturation; ///< input saturation
  Eigen::Matrix<float, 2, 1> saturation_vec;
  const int RW_bias = 12; ///< 우측 휠 모터의 바이어스 값
  const float V_ERR_MAX = 1.0f;

public:
  float theta_d;
  float u_V, u_P, u_D, u_Y;
  /**
   * @brief 생성자: VYBController 초기화
   * @param ServoRW_ 우측 휠 서보 객체 참조
   * @param ServoLW_ 좌측 휠 서보 객체 참조
   */
  VYBController(MGServo &ServoRW_, MGServo &ServoLW_)
      : ServoRW(ServoRW_), ServoLW(ServoLW_)
  {
    theta_d = 0.f;
    // 전류 및 토크 상수 초기화
    iq_factor = 0.001611328f; // (A/LSB) 3.3 / 2048
    torque_constant = 0.7f;   // (Nm/A) * reduction ratio
    saturation = iq_factor * torque_constant * MAX_TORQUE_COMMAND;
    // saturation_vec << MAX_TORQUE_COMMAND * RW_factor, MAX_TORQUE_COMMAND * LW_factor;
    saturation_vec << MAX_TORQUE, MAX_TORQUE;

    // LQR 게인 초기화 (하드코딩된 데이터 삽입)
    Eigen::Matrix<float, 2, 4> mat;
    //////////////////////////////////////////////////////////
    /* ----------------------------------------
    Q = diag([80.0, 0.0, 5.0, 5.0])
    R = diag([170.0, 170.0])
    v = 1.0m/s 클램핑 기준
    --------------------------------------------------- */
    mat << 1.04268079f, 0.14609268f, 0.11351781f, -0.11473299f,
        -1.05378950f, -0.14839968f, -0.11400699f, -0.11450148f;
    Ks.push_back(mat);

    mat << 1.14203427f, 0.14301997f, 0.11275333f, -0.11465966f,
        -1.15348585f, -0.14536032f, -0.11302156f, -0.11468094f;
    Ks.push_back(mat);

    mat << 1.22304494f, 0.14094096f, 0.11209441f, -0.11460259f,
        -1.23482905f, -0.14337015f, -0.11220871f, -0.11480983f;
    Ks.push_back(mat);

    mat << 1.29167272f, 0.13965856f, 0.11151868f, -0.11455935f,
        -1.30384274f, -0.14221710f, -0.11152809f, -0.11490366f;
    Ks.push_back(mat);

    mat << 1.35111937f, 0.13912858f, 0.11102110f, -0.11452748f,
        -1.36375680f, -0.14184795f, -0.11096433f, -0.11496998f;
    Ks.push_back(mat);

    mat << 1.40346388f, 0.13932375f, 0.11060041f, -0.11450466f,
        -1.41665862f, -0.14222862f, -0.11050854f, -0.11501367f;
    Ks.push_back(mat);

    mat << 1.45015242f, 0.14021126f, 0.11025547f, -0.11448877f,
        -1.46399091f, -0.14332076f, -0.11015339f, -0.11503832f;
    Ks.push_back(mat);

    mat << 1.49221141f, 0.14174915f, 0.10998404f, -0.11447789f,
        -1.50676937f, -0.14507764f, -0.10989150f, -0.11504675f;
    Ks.push_back(mat);

    mat << 1.53035888f, 0.14388639f, 0.10978260f, -0.11447029f,
        -1.54569660f, -0.14744385f, -0.10971486f, -0.11504132f;
    Ks.push_back(mat);

    mat << 1.56507061f, 0.14656353f, 0.10964649f, -0.11446434f,
        -1.58122962f, -0.15035563f, -0.10961489f, -0.11502398f;
    Ks.push_back(mat);

    mat << 1.59663440f, 0.14971403f, 0.10957035f, -0.11445859f,
        -1.61363458f, -0.15374182f, -0.10958267f, -0.11499652f;
    Ks.push_back(mat);

    mat << 1.62523391f, 0.15326941f, 0.10954844f, -0.11445209f,
        -1.64307063f, -0.15752896f, -0.10960906f, -0.11496110f;
    Ks.push_back(mat);

    mat << 1.65110531f, 0.15717471f, 0.10957502f, -0.11444592f,
        -1.66974568f, -0.16165682f, -0.10968466f, -0.11492198f;
    Ks.push_back(mat);

    mat << 1.67472716f, 0.16141434f, 0.10964504f, -0.11444624f,
        -1.69410150f, -0.16610408f, -0.10979960f, -0.11488910f;
    Ks.push_back(mat);
  }

  /**
   * @brief 현재 제어 입력 벡터를 반환
   * @return Eigen::Matrix<float, 2, 1> u
   */
  Eigen::Matrix<float, 2, 1> getInputVector()
  {
    return u;
  }
  /**
   * @brief 모터 속도 측정을 수행하여 측정 벡터에 반영
   * @param z 모터 속도 측정값을 저장할 벡터
   */
  void getMotorSpeedMeasurement(Eigen::Matrix<float, 8, 1> &z)
  {
    z(6) = ServoRW.getMotorSpeed() * M_PI / 180;
    z(7) = ServoLW.getMotorSpeed() * M_PI / 180;
  }

  /**
   * @brief 모터 Current 측정값 update
   * @param iq_vec 모터 current 측정값을 저장할 벡터
   */
  void getMotorCurrentMeasurement(Eigen::Matrix<float, 2, 1> &iq_vec)
  {
    iq_vec << ServoRW.getMotorIq(), ServoLW.getMotorIq();
  }

  /**
   * @brief 현재 높이에 따라 LQR 게인 K를 계산
   * @param h 현재 높이 (m)
   */
  void computeGainK(const float h)
  {
    float temp = (h - HEIGHT_MIN) / 0.01; // 구간을 10mm당 하나씩 나눔
    int idx = static_cast<int>(temp);     // 구간의 정수 인덱스 계산

    if (idx >= 0 && idx < static_cast<int>(Ks.size()) - 1)
    {
      // 보간 비율 계산
      float ratio = temp - idx; // 현재 위치가 구간 내에서 차지하는 비율

      // 보간 수행
      K = Ks.at(idx) * (1.0f - ratio) + Ks.at(idx + 1) * ratio;
    }
    else if (idx < 0)
    {
      // h가 HEIGHT_MIN 이하일 경우 최소값 사용
      K = Ks.front();
    }
    else
    {
      // h가 범위를 벗어날 경우 최대값 사용
      K = Ks.back();
    }
  }

  /**
   * @brief 상태 벡터를 기반으로 제어 입력 벡터를 계산
   * @param x_d 목표 상태 벡터
   * @param x 현재 상태 벡터
   */
  void computeInput(Eigen::Matrix<float, 4, 1> &x_d, Eigen::Matrix<float, 4, 1> &x)
  {
    // 1. 순수 에러 벡터 계산
    Eigen::Matrix<float, 4, 1> err = x_d - x;

    // 2. V항의 토크 예산 독점 방지
    // 속도를 줄이는 데 쓰는 토크를 제한하여 u_P를 무조건 확보함
    // Q[2] 게인에 맞춰 조절 필요
    err(2) = std::max(-V_ERR_MAX, std::min(V_ERR_MAX, err(2)));

    // 3. 기여도 로깅 및 최종 토크 계산
    u_P = K(0, 0) * err(0);
    u_D = K(0, 1) * err(1);
    u_V = K(0, 2) * err(2);
    u_Y = K(0, 3) * err(3);
    u = K * err;

    // 4. Input saturation (최종 출력 모터 토크 제한)
    for (int j = 0; j < 2; j++)
    {
      if (u(j) > saturation_vec(j))
        u(j) = saturation_vec(j);
      else if (u(j) < -saturation_vec(j))
        u(j) = -saturation_vec(j);
    }
  }

  /**
   * @brief 계산된 제어 명령을 서보에 전송
   */
  void sendControlCommand()
  {
    // float u_RW = u(0) / (iq_factor * torque_constant);
    // float u_LW = u(1) / (iq_factor * torque_constant);

    // // Right Wheel motor의 마찰로 인해 발생하는 torque 문제를 조정해줌
    // if (u_RW < 0) {
    //   u_RW -= RW_bias;
    // } else if (u_RW > 0) {
    //   u_RW += RW_bias;
    // }

    float u_RW = u(0) / RW_factor;
    float u_LW = u(1) / LW_factor;

    ServoRW.sendTorqueControlCommand(static_cast<int16_t>(u_RW));
    ServoLW.sendTorqueControlCommand(static_cast<int16_t>(u_LW));
  }

  void sendDirectControlCommand(Eigen::Matrix<float, 2, 1> u_)
  {
    float u_RW = u_(0) / (iq_factor * torque_constant);
    float u_LW = u_(1) / (iq_factor * torque_constant);

    // Right Wheel motor의 마찰로 인해 발생하는 torque 문제를 조정해줌
    if (u_RW < 0)
    {
      u_RW -= RW_bias;
    }
    else if (u_RW > 0)
    {
      u_RW += RW_bias;
    }

    ServoRW.sendTorqueControlCommand(static_cast<int16_t>(u_RW));
    ServoLW.sendTorqueControlCommand(static_cast<int16_t>(u_LW));
  }

  void sendReadStateCommand()
  {
    ServoRW.sendCommandReadMotorState2();
    ServoLW.sendCommandReadMotorState2();
  }
};

#endif // VYB_CONTROLLER_H
