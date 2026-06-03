#include "Params.h"        // 모든 물리 상수가 정의된 헤더
#include "Bluetooth.h"     // 블루투스 커맨드 수신 클래스 (Receiver 대체)
#include "HRController.h"  // 고관절(Hip) 서보 제어기
#include "VYBController.h" // 바퀴(Wheel) 밸런싱 제어기
#include "MGServo.h"       // RS485 통신 기반 휠 모터 드라이버
#include "IMU.h"           // MPU6050 센서 데이터 처리
#include "POL.h"           // 로봇의 물리 모델(Physics of Locomotion)
#include "EKF.h"           // 확장 칼만 필터(상태 추정기)
#include "Logger.h"        // WiFi 기반 데이터 로거
#include "Timer.h"         // 샘플링 주기를 위한 정밀 타이머

// --- 객체 생성 및 물리 설정 ---
const Properties properties = createDefaultProperties(); // 로봇의 질량, 길이 등 기본 파라미터
POL Pol(properties);                                     // 물리 모델에 파라미터 주입
HardwareSerial RS485(1);                                 // RS485 통신을 위한 시리얼 포트 설정
MGServo ServoLW(1, RS485);                               // 1번 모터: 왼쪽 바퀴
MGServo ServoRW(2, RS485);                               // 2번 모터: 오른쪽 바퀴
WBRBluetooth ble;                                        // 블루투스 수신용 객체 (기존 Receiver 대체)
IMU MPU6050;                                             // 자이로/가속도 센서 객체

HRController HR_controller;                     // 고관절 제어 인스턴스
VYBController VYB_controller(ServoRW, ServoLW); // 바퀴 제어 인스턴스
Logger WIFI_Logger(ssid, password);             // 데이터 전송용 와이파이 설정
EKF Estimator(Pol);                             // 모델 기반 상태 추정기

// --- 행렬 선언 (Eigen 라이브러리 활용) ---

/* 현재 상태 벡터 x = [θ, θ̇ , v, ψ̇ ]
θ, θ̇ : IMU (MPU6050) 기반 추정된 Pitch 각도/각속도
v    : 좌우 바퀴 속도 합
ψ̇    : 좌우 바퀴 속도 차 */
Eigen::Matrix<float, 4, 1> x = Eigen::Matrix<float, 4, 1>::Zero();
// 목표 상태 벡터 x_d
Eigen::Matrix<float, 4, 1> x_d = Eigen::Matrix<float, 4, 1>::Zero();
// 시간 지연 보상을 위한 이전 토크 u_prev
Eigen::Matrix<float, 2, 1> u_prev = Eigen::Matrix<float, 2, 1>::Zero();
// 현재 토크 u 입력
Eigen::Matrix<float, 2, 1> u = Eigen::Matrix<float, 2, 1>::Zero();
// 센서 측정값 벡터 z (가속도, 각속도 등 8개 항목)
Eigen::Matrix<float, 8, 1> z = Eigen::Matrix<float, 8, 1>::Zero();
// 모터의 실제 전류 피드백값
Eigen::Matrix<float, 2, 1> iq_vec = Eigen::Matrix<float, 2, 1>::Zero();

int i = 0;
Timer log_timer(Timer::TimerType::Millis);
Timer sampling_timer(Timer::TimerType::Millis);
Timer temp_timer(Timer::TimerType::Millis);
float h_d = HEIGHT_MAX, phi_d = 0;
float v_d = 0, dpsi_d = 0;

void serialPrintStates();
void Ready2Log();
void LogData();

// ==============================================================================
//                                    DEBUG
// ==============================================================================
// Off 모드에서 Height 반전문제 디버깅
void sim_off_mode_gain();
void dbgHeight();

// ==============================================================================
//                                    SETUP
// ==============================================================================
void setup()
{
  // Serial 통신, Bluetooth 초기화
  Serial.begin(115200);
  delay(1000);                                                        // 디버깅용 PC 시리얼 통신 시작
  ble.begin("ESP32_WBR_Control", &ServoLW, &ServoRW, &HR_controller); // 블루투스 수신 대기 (기존 receiver.begin)
  HR_controller.attachServos(LH_PIN, RH_PIN);                         // 고관절 서보 핀 할당

  // IMU (MPU6050) 초기화
  if (!MPU6050.begin())
  {
    Serial.println("[ERROR] Fail to initialize IMU.");
  }
  // RS485 송수신 방향 제어 핀 설정
  pinMode(RS485_DE_RE, OUTPUT);
  // 기본 상태를 수신(Read) 모드로 설정
  digitalWrite(RS485_DE_RE, LOW);
  // 모터와 통신할 고속 라인 개통
  RS485.begin(460800, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  // WIFI 연결 (CSV 로깅 서비스 시작)
  WIFI_Logger.begin();

  // PSRAM 상태 확인 및 초기화
  // --- ESP32 PSRAM(외부 메모리) 체크 ---
  if (psramFound())
  { // 대용량 로깅을 위해 PSRAM이 있는지 확인
    if (!psramInit())
    {
      while (1)
      {
        Serial.println("PSRAM Fail!");
      } // 메모리 초기화 실패 시 멈춤
    }
  }
  else
  {
    while (1)
    {
      Serial.println("PSRAM not available.");
    } // PSRAM 없으면 구동 불가
  }

  // CSV 컬럼 (모니터링 타겟) 리스트 준비
  Ready2Log();

  // 시간 측정 시작
  log_timer.start();
  sampling_timer.start();
}

// ==============================================================================
//                                    LOOP
// ==============================================================================
void loop()
{
  // --- 1. 주기 제어 (Sampling Time Check) ---
  // dt(0.008) * 1000 = 8ms가 경과했는지 확인합니다.
  if (sampling_timer.getDuration() >= dt * 1000)
  {
    sampling_timer.start(); // sampling timer 초기화

    // 조종기(블루투스)의 'Run' 상태가 켜져 있을 때만 제어 로직을 실행합니다.
    if (ble.isRun())
    {
      // 현재 추정된 상태 벡터 x를 물리 모델(Pol)에 주입합니다.
      Pol.setState(x);

      // 제어 루프의 지연(Latency)을 고려하여, 이번 주기 명령(u)이 아닌
      // 이전 주기 명령(u_prev)을 모델에 입력하여 동역학을 예측합니다.
      Pol.setInput(u_prev);

      // 센서 데이터 수집 (Measurement Update)
      // IMU(MPU6050)에서 현재의 가속도와 각속도를 읽어옵니다
      MPU6050.readData();
      // 읽어온 데이터를 측정 벡터 z의 전반부(0~5번 인덱스)에 채웁니다
      MPU6050.getIMUMeasurement(z);

      // RS-485 통신을 통해 좌/우 바퀴 모터에 상태 요청 명령을 보냅니다
      VYB_controller.sendReadStateCommand();
      // 모터의 엔코더에서 환산된 회전 속도를 읽어 z의 후반부(6~7번 인덱스)에 채웁니다
      VYB_controller.getMotorSpeedMeasurement(z);
      // 모터의 현재 소모 전류(iq) 값을 iq_vec에 저장합니다
      VYB_controller.getMotorCurrentMeasurement(iq_vec);

      // ---  상태 추정 (EKF State Estimation)  ---
      // 측정 벡터 z와 물리 모델의 예측값을 비교하여 최적의 상태 x를 도출합니다.
      // EKF 내부에서 predict(예측)와 update(보정)가 동시에 일어납니다
      if (!Estimator.estimate_state(x, z))
      {
        // --- 예외 처리: Diverge Safe Guard ---
        // 만약 수치 연산 중 에러가 발생하여 필터가 발산하면 즉시 정지합니다.
        ServoLW.sendTorqueControlCommand(0); // 왼쪽 바퀴 토크 0
        ServoRW.sendTorqueControlCommand(0); // 오른쪽 바퀴 토크 0

        while (true)
        {
          Serial.println("Mass matrix Singularity Error. Change mode to Off Mode...");
          x.setZero();                 // 상태 초기화
          u.setZero();                 // 출력 초기화
          Estimator.reset_estimator(); // 추정기 초기화

          // 사용자가 'Run' 상태를 끌 때까지 무한 루프에서 대기합니다.
          delay(500);
          if (!ble.isRun())
          {
            return;
          }
        }
      }

      // BLE 입력으로부터 목표 상태 설정 //
      h_d = ble.getH();      // 목표 몸체 높이(m) 가져오기
      phi_d = 0;             // 롤(Roll) 제어는 현재 비활성화
      v_d = ble.getV();      // 목표 전진 속도(m/s) 가져오기
      dpsi_d = ble.getYaw(); // 목표 회전 속도(rad/s) 가져오기

      // 목표 상태 벡터 x_d의 속도와 요(Yaw) 속도 성분을 채웁니다.
      x_d.segment<2>(2) << v_d, dpsi_d; // x_d[2] = v_d, x_d[3] = dpsi_d

      /* 무게중심 변화에 따른 물리량 재계산 (POL)
         목표 높이(h_d) 입력=> [기구학] => POL 값 변경 => 변경된 값으로 목표 피치(θ) 결정 */

      Pol.setHR(h_d, phi_d);           // 모델에 목표 높이와 롤 각도 설정
      Pol.calculate_com_and_inertia(); // 목표 높이 h_d가 되려면 hip 서보가 몇 도여야 하는지 계산.
      Pol.get_theta_eq(x_d(0));        // 목표 무게중심(CoM) 기준, 균형을 잡기 위한 목표 피치(θ) 계산
      // x_d(0) -= CoM_OFFSET;
      //// 게인 스케줄링 및 LQR 제어 (VYB) ////
      // 목표 높이(h_d)에 가장 적합한 LQR 제어 게인 행렬 K(2x4)를 보간법으로 선택
      VYB_controller.computeGainK(h_d);
      // u = K * (x_d - x) 수식을 통해 목표와 현재 상태 오차를 줄이는 토크(u)를 계산
      VYB_controller.computeInput(x_d, x);

      //// 하드웨어 명령 하달 ////
      // POL에서 계산된 역기구학 결과를 바탕으로 고관절 서보 모터를 구동합니다.
      HR_controller.controlHipServos(Pol.get_theta_hips());
      // 계산된 토크(u)를 좌/우 바퀴 모터에 전송합니다.
      VYB_controller.sendControlCommand();

      // 이번 주기에 보낸 명령 u를 u_prev에 저장하여 다음 주기 EKF 예측에 사용합니다.
      u_prev = u;
      u = VYB_controller.getInputVector();

      LogData();
    }

    else if (ble.isReset()) // Estimator Reset
    {
      x_d.setZero();
      VYB_controller.theta_d = 0.f;
      x.setZero();
      u.setZero();
      Estimator.reset_estimator();
      WIFI_Logger.resetLogData();
      log_timer.start();
    }

    else // Off Mode
    {
      ServoLW.sendTorqueControlCommand(0);
      ServoRW.sendTorqueControlCommand(0);
      u.setZero();

      // WIFI 무한루프 돌면서 찾은 뒤, RUN Mode 동안의 Log Data 전송
      // 주석처리하지 말것! : main 진입전에 죽음
      WIFI_Logger.handleClientRequests(); 

      Pol.setHR(h_d, phi_d);
      Pol.calculate_com_and_inertia();
      Pol.get_theta_eq(x_d(0));

      Pol.setState(x);
      Pol.setInput(u);

      // measurement update
      MPU6050.readData();
      MPU6050.getIMUMeasurement(z);
      VYB_controller.getMotorSpeedMeasurement(z);
      // state estimation (Off Mode에서도 추정기는 돌려야 Run 전환 시 튀지 않음)
      if (!Estimator.estimate_state(x, z))
      {
        ServoLW.sendTorqueControlCommand(0);
        ServoRW.sendTorqueControlCommand(0);
        while (true)
        {
          Serial.println("Mass matrix Singularity Error. Change mode to Off Mode...");
          x.setZero();
          u.setZero();
          Estimator.reset_estimator();
          delay(500);
          if (!ble.isRun())
          {
            return;
          }
        }
      }

      sim_off_mode_gain();
    }
  }
}

void serialPrintStates()
{
  Serial.print("theta:");
  Serial.print(x(0) * 180 / M_PI, 6);
  Serial.print(" ");
  Serial.print("theta_dot:");
  Serial.print(x(1) * 180 / M_PI, 6);
  Serial.print(" ");
  Serial.print("v:");
  Serial.print(x(2), 6);
  Serial.print(" ");
  Serial.print("psi_dot:");
  Serial.print(x(3) * 180 / M_PI, 6);
  Serial.print(" ");
  Serial.print("u_RW:");
  Serial.print(u(0), 6);
  Serial.print(" ");
  Serial.print("u_LW:");
  Serial.print(u(1), 6);
  Serial.print(" ");
}

void Ready2Log()
{
  WIFI_Logger.readyToLogValue("cal_time");
  WIFI_Logger.readyToLogTimeStamp(); // 시간 기록
  WIFI_Logger.readyToLogValue("h_d");
  WIFI_Logger.readyToLogValue("theta_d");
  WIFI_Logger.readyToLogValue("v_d");
  WIFI_Logger.readyToLogValue("psi_dot_d");
  WIFI_Logger.readyToLogValue("theta_hat");
  WIFI_Logger.readyToLogValue("theta_dot_hat");
  WIFI_Logger.readyToLogValue("v_hat");
  WIFI_Logger.readyToLogValue("psi_dot_hat");
  WIFI_Logger.readyToLogValue("tau_RW");
  WIFI_Logger.readyToLogValue("tau_LW");
  WIFI_Logger.readyToLogValue("acc_x");
  WIFI_Logger.readyToLogValue("acc_y");
  WIFI_Logger.readyToLogValue("acc_z");
  WIFI_Logger.readyToLogValue("gyr_x");
  WIFI_Logger.readyToLogValue("gyr_y");
  WIFI_Logger.readyToLogValue("gyr_z");
  WIFI_Logger.readyToLogValue("theta_dot_RW");
  WIFI_Logger.readyToLogValue("theta_dot_LW");
  WIFI_Logger.readyToLogValue("current_RW");
  WIFI_Logger.readyToLogValue("current_LW");
  WIFI_Logger.readyToLogValue("deg_R");
  WIFI_Logger.readyToLogValue("deg_L");
  WIFI_Logger.readyToLogValue("angle_RH");
  WIFI_Logger.readyToLogValue("angle_LH");
}

void LogData()
{
  // hip 각도 계산 결과
  Eigen::Vector2f hips = Pol.get_theta_hips();

  // 클램핑 전 raw 값 계산
  float deg_R = hips(0) * 180.0f / M_PI;
  float deg_L = hips(1) * 180.0f / M_PI;
  float angle_RH = 90 - (int)(deg_R * 90.0f / 135.0f);
  float angle_LH = 90 - (int)(deg_L * 90.0f / 135.0f);

  WIFI_Logger.logValue("cal_time", sampling_timer.getDuration());
  WIFI_Logger.logTimeStamp(log_timer.getDuration());
  WIFI_Logger.logValue("h_d", h_d);
  WIFI_Logger.logValue("theta_d", x_d(0));
  WIFI_Logger.logValue("v_d", x_d(2));
  WIFI_Logger.logValue("psi_dot_d", x_d(3));
  WIFI_Logger.logValue("theta_hat", x(0));
  WIFI_Logger.logValue("theta_dot_hat", x(1));
  WIFI_Logger.logValue("v_hat", x(2));
  WIFI_Logger.logValue("psi_dot_hat", x(3));
  WIFI_Logger.logValue("tau_RW", u(0));
  WIFI_Logger.logValue("tau_LW", u(1));
  WIFI_Logger.logValue("acc_x", z(0));
  WIFI_Logger.logValue("acc_y", z(1));
  WIFI_Logger.logValue("acc_z", z(2));
  WIFI_Logger.logValue("gyr_x", z(3));
  WIFI_Logger.logValue("gyr_y", z(4));
  WIFI_Logger.logValue("gyr_z", z(5));
  WIFI_Logger.logValue("theta_dot_RW", z(6));
  WIFI_Logger.logValue("theta_dot_LW", z(7));
  WIFI_Logger.logValue("current_RW", iq_vec(0));
  WIFI_Logger.logValue("current_LW", iq_vec(1));
  WIFI_Logger.logValue("deg_R", deg_R);
  WIFI_Logger.logValue("deg_L", deg_L);
  WIFI_Logger.logValue("angle_RH", angle_RH);
  WIFI_Logger.logValue("angle_LH", angle_LH);
}


void sim_off_mode_ekf_check()
/* OFF 모드에서 EKF의 개입(왜곡) 여부 시리얼 관찰
1. RAW 데이터(가속도, 자이로, 엔코더)가 올바른 부호를 내는지 확인
2. EKF를 거친 결과(x)가 RAW 데이터의 경향성(부호)을 그대로 잘 따라가는지 확인
*/
{
  static unsigned long last = 0;
  if (millis() - last > 500) 
  {
    last = millis();

    // ----------------------------------------------------
    // [ Before EKF ] : 센서 원시 데이터 (z 벡터) 기반 임시 계산
    // ----------------------------------------------------
    // 1. 가속도계 기반 Pitch 추정
    // z[8]: { acc[3], gyro[3], V_right, V_left };
    // OFF 모드여도, 손으로 직접 밀면서 (V_right, V_left != 0) 을 만들거니까 의미가 있음
    float raw_theta = atan2(-z(0), z(2)); 

    // 2. 자이로스코프 Y축 (Pitch Rate)
    float raw_theta_dot = z(4); 

    // 3. 엔코더 기반 속도 추정
    // 모터 속도 = 로봇 회전 각속도 - (로봇 전진 속도 / 바퀴 반지름)
    // 따라서 v_raw = R * (raw_theta_dot - 평균 모터 회전속도)
    float avg_motor_speed = (z(6) + z(7)) / 2.0f;
    float raw_v = Pol.R * (raw_theta_dot - avg_motor_speed);

    // ----------------------------------------------------
    // [ 시리얼 출력 ] : Before vs After 정면 비교
    // ----------------------------------------------------
    Serial.println("========================================");
    
    // 1. 각도 비교 (가속도계 vs EKF)
    Serial.println("[ 1. 각도 (Theta) ]");
    Serial.printf("RAW(가속도): %+.2f deg | EKF: %+.2f deg\n", 
                  raw_theta * 180 / M_PI, x(0) * 180 / M_PI);
    
    // 2. 각속도 비교 (자이로 vs EKF)
    Serial.println("[ 2. 각속도 (Theta_dot) ]");
    Serial.printf("RAW(자이로): %+.2f deg/s | EKF: %+.2f deg/s\n", 
                  raw_theta_dot * 180 / M_PI, x(1) * 180 / M_PI);

    // 3. 속도 비교 (엔코더 vs EKF)
    Serial.println("[ 3. 전진 속도 (Velocity) ]");
    Serial.printf("RAW(엔코더): %+.3f m/s | EKF: %+.3f m/s\n", 
                  raw_v, x(2));
                  
    Serial.println("========================================");
  }
}


void sim_off_mode_gain()
/* OFF 모드에서 시리얼 관찰
Step 1: 입력단(센서)의 '앞' 통일하기 
1) 각도/각속도 확인: 로봇의 상체를 손으로 잡고 앞으로 기울입니다.
- Pass 조건: theta 와 theta_dot 이 모두 양수(+)가 찍혀야 합니다.
2) 엔코더 속도 확인: 상체는 세워둔 채, 바퀴를 바닥에 대고 앞으로 죽 밀어줍니다. 
- Pass 조건: v 가 양수(+)가 찍혀야 합니다.


Step 2: LQR 확인하기
1) P/D항 확인: 상체를 앞으로 확 엎어뜨려 봅니다. (theta > 0, theta_dot > 0)
- Pass 조건: u_P < 0 , u_D < 0  
2) V항 확인: 바퀴를 앞으로 죽 밀어봅니다. 
- Pass 조건: u_V < 0

*/
{
  static unsigned long last = 0;
  if (millis() - last > 500) 
  {
    last = millis();
    
    // 목표 상태 (보통 x_d = 0)
    float err_theta = x_d(0) - x(0);
    float err_theta_dot = x_d(1) - x(1);
    float err_v = x_d(2) - x(2);

    // K 배열의 0번째 인덱스 (h=0.13 근처의 하드코딩된 RW 게인 대략적용)
    // offMode 실험이므로, RUN mode 게인을 대입해서 실제 거동 시뮬레이션 
    float K0 = 1.63f; 
    float K1 = 0.18f; 
    float K2 = 0.23f; 

    float u_P = K0 * err_theta;      // P 제어력 (각도)
    float u_D = K1 * err_theta_dot;  // D 제어력 (각속도 댐핑)
    float u_V = K2 * err_v;          // V 제어력 (속도 억제)

    Serial.println("========================================");
    Serial.printf("[상태 변수] theta: %+.2f | theta_dot: %+.2f | v: %+.2f\n", 
                  x(0) * 180 / M_PI, x(1) * 180 / M_PI, x(2));
                  
    // P와 D가 같은 방향의 힘을 내는지(음의 피드백), 반대로 싸우는지(양의 피드백) 확인
    /* v_d = 0인 상태입니다. 기체를 억지로 앞으로 굴렸으므로 (v > 0), 
       LQR은 u_V 항에 음수(-)가 찍혀야 정상입니다. */
    Serial.printf("[RW 토크 기여도] u_P(각도): %+.2f | u_D(댐핑): %+.2f | u_V(속도): %+.2f\n", 
                  u_P, u_D, u_V);
    Serial.println("========================================");
  }
}

void dbgHeight()
// params.h SERVO_LINK_ANGLE 결정을 위한 디버깅 함수
{
  static unsigned long last = 0;
  if (millis() - last > 1000)
  {
    last = millis();
    h_d = ble.getH();
    Pol.setHR(h_d, phi_d);
    Pol.calculate_com_and_inertia();

    Eigen::Vector2f hips = Pol.get_theta_hips();
    Serial.printf("h=%.3f RH=%.2f LH=%.2f\n",
                  h_d, hips(0) * 180 / M_PI, hips(1) * 180 / M_PI);
    float theta_eq_val;
    Pol.get_theta_eq(theta_eq_val);
    Serial.printf("h=%.3f theta_eq=%.3f theta_hat=%.3f\n",
                  h_d,
                  theta_eq_val * 180 / M_PI,
                  x(0) * 180 / M_PI);
    // IMU 원시값
    Serial.printf("acc: %.3f %.3f %.3f | gyr: %.3f %.3f %.3f\n",
        MPU6050.acc_vec(0), MPU6050.acc_vec(1), MPU6050.acc_vec(2),
        MPU6050.gyr_vec(0), MPU6050.gyr_vec(1), MPU6050.gyr_vec(2));
    // EKF 상태
    Serial.printf("eq=%.3f hat=%.3f hat_dot=%.3f v=%.4f\n",
        (theta_eq_val) * 180/M_PI,
        x(0) * 180/M_PI,
        x(1) * 180/M_PI,
        x(2));
    // z 측정벡터
    Serial.printf("z: %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
        z(0),z(1),z(2),z(3),z(4),z(5),z(6),z(7));
  }
}
