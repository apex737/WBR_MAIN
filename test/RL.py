"""
=============================================================================
 WBR 시뮬레이션 환경 (Gymnasium)
 Wheeled Inverted Pendulum 물리 시뮬레이션

 Isaac Sim 없이 순수 Python으로 로봇 물리를 시뮬레이션합니다.
 PPO 강화학습 학습용 환경입니다.
=============================================================================
"""

import math
import numpy as np
import gymnasium as gym
from gymnasium import spaces


class WBRBalanceEnv(gym.Env):
    """
    WBR (Wheeled Bipedal Robot) 밸런싱 시뮬레이션 환경

    물리 모델: Wheeled Inverted Pendulum (바퀴 달린 역진자)

    상태(State):
        [theta, theta_dot, x, x_dot]
        theta     = 몸체 pitch 각도 (rad, 0=수직)
        theta_dot = pitch 각속도 (rad/s)
        x         = 바퀴 위치 (m)
        x_dot     = 전진 속도 (m/s)

    관측(Observation):
        [sin(theta), cos(theta), theta_dot, x_dot]
        sin/cos를 사용하여 각도의 연속성 보장

    행동(Action):
        [normalized_torque]  범위: [-1, 1]
        -1 = 최대 후진 토크, +1 = 최대 전진 토크
        좌우 바퀴에 동일 토크 적용 (밸런싱 목적)

    리워드(Reward):
        +1.0  살아있는 매 스텝
        -5.0 * theta²     기울어질수록 벌점
        -0.1 * theta_dot²  진동 벌점
        -0.5 * x_dot²     이동 벌점
        -0.01 * action²   에너지 벌점
        -100   넘어지면 (|theta| > 45°)
    """

    metadata = {"render_modes": ["human"], "render_fps": 50}

    def __init__(self, render_mode=None, domain_randomize=True):
        super().__init__()

        self.render_mode = render_mode
        self.domain_randomize = domain_randomize

        # ─── 기본 물리 파라미터 (레퍼런스 기반) ───
        self._base_params = {
            'body_mass': 1.5,           # kg (바퀴 제외 상체)
            'wheel_mass': 0.3,          # kg (양쪽 바퀴+모터 합)
            'com_height': 0.12,         # m (바퀴축~무게중심 거리)
            'wheel_radius': 0.0725,     # m (config.yaml 기준)
            'friction_rot': 0.01,       # 회전 마찰 계수
            'friction_trans': 0.1,      # 병진 마찰 계수
            'max_torque_per_wheel': 0.75,  # Nm (config.yaml 기준)
        }

        # ─── 시뮬레이션 설정 ───
        self.dt = 0.008                 # 8ms = 125Hz (실제 로봇과 동일)
        self.max_episode_steps = 3750   # 30초 (125Hz × 30)

        # ─── 행동 공간: 정규화된 토크 [-1, 1] ───
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(1,), dtype=np.float32
        )

        # ─── 관측 공간: [sin(θ), cos(θ), θ̇, x, v] ───
        obs_high = np.array([1.0, 1.0, 15.0, 5.0, 3.0], dtype=np.float32)
        self.observation_space = spaces.Box(
            low=-obs_high, high=obs_high, dtype=np.float32
        )

        # ─── 내부 상태 ───
        self.state = None       # [theta, theta_dot, x, x_dot]
        self.step_count = 0
        self.params = {}

    # ─────────────────────────────────────────────────────────────────
    #  환경 리셋
    # ─────────────────────────────────────────────────────────────────

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        # 도메인 랜덤화 적용
        self._randomize_params()

        # 초기 상태: 약간 기울어진 상태에서 시작
        theta = self.np_random.uniform(-0.05, 0.05)
        theta_dot = self.np_random.uniform(-0.1, 0.1)
        x = 0.0
        x_dot = self.np_random.uniform(-0.05, 0.05)

        self.state = np.array([theta, theta_dot, x, x_dot], dtype=np.float64)
        self.step_count = 0

        return self._get_obs(), {}

    # ─────────────────────────────────────────────────────────────────
    #  한 스텝 진행
    # ─────────────────────────────────────────────────────────────────

    def step(self, action):
        action_val = float(np.clip(action[0], -1.0, 1.0))

        # 토크 → 접지면 힘으로 변환
        F = action_val * self.params['max_force']

        # 랜덤 외란 (가끔 밀어줌 → 외란 대응 학습)
        if self.np_random.random() < 0.002:     # 0.2% 확률
            F += self.np_random.uniform(-3.0, 3.0)

        # RK4 적분으로 물리 시뮬레이션
        self.state = self._rk4_step(self.state, F)

        theta = self.state[0]
        theta_dot = self.state[1]
        x = self.state[2]
        x_dot = self.state[3]

        # ─── 리워드 계산 ───
        reward = 1.0                            # 살아있으면 +1
        reward -= 8.0 * theta ** 2              # 기울기 벌점 (LQR Q 매트릭스 비율 반영하여 가중치 증가)
        reward -= 0.1 * theta_dot ** 2          # 진동 벌점
        reward -= 0.5 * x ** 2                  # 위치 이탈 벌점
        reward -= 0.5 * x_dot ** 2              # 이동 속도 벌점
        reward -= 0.1 * action_val ** 2         # 에너지 벌점 (토크 한계 0.75Nm 부근 진동을 막기 위해 페널티 강화)

        # 넘어짐 판정
        terminated = bool(abs(theta) > 0.7854)  # 45도
        if terminated:
            reward -= 100.0

        self.step_count += 1
        truncated = self.step_count >= self.max_episode_steps

        # 관측에 노이즈 추가 (sim-to-real)
        obs = self._get_obs()
        if self.domain_randomize:
            noise = self.np_random.normal(0, 0.005, size=obs.shape).astype(np.float32)
            obs = obs + noise

        return obs, float(reward), terminated, truncated, {
            'theta': theta,
            'theta_dot': theta_dot,
            'velocity': x_dot,
            'torque': action_val,
        }

    # ─────────────────────────────────────────────────────────────────
    #  관측 벡터 생성
    # ─────────────────────────────────────────────────────────────────

    def _get_obs(self):
        theta, theta_dot, x, x_dot = self.state
        return np.array([
            math.sin(theta),
            math.cos(theta),
            np.clip(theta_dot, -15.0, 15.0),
            np.clip(x, -1.0, 1.0),      # LQR의 V_err 클램프(1.0) 아이디어 적용: 위치 오차 제한
            np.clip(x_dot, -1.0, 1.0),  # LQR의 V_err 클램프(1.0) 아이디어 적용: 속도 오차 제한
        ], dtype=np.float32)

    # ─────────────────────────────────────────────────────────────────
    #  물리 엔진 (역진자 운동방정식)
    # ─────────────────────────────────────────────────────────────────

    def _dynamics(self, state, F):
        """
        Wheeled Inverted Pendulum 운동방정식

        질량 행렬:
            [I+mL²   mLcosθ] [θ̈ ]   [mgLsinθ - b_r·θ̇              ]
            [mLcosθ  M+m   ] [ẍ ] = [mLsinθ·θ̇² + F - b_t·ẋ  ]
        """
        theta, theta_dot, x, x_dot = state
        p = self.params

        M_b = p['body_mass']
        M_w = p['wheel_mass']
        L = p['com_height']
        I_b = p['body_inertia']
        g = 9.81
        b_r = p['friction_rot']
        b_t = p['friction_trans']

        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        # 질량 행렬 요소
        a11 = I_b + M_b * L * L
        a12 = M_b * L * cos_t
        a22 = M_w + M_b

        # 우변 (힘/토크)
        rhs1 = M_b * g * L * sin_t - b_r * theta_dot
        rhs2 = M_b * L * sin_t * theta_dot * theta_dot + F - b_t * x_dot

        # 연립방정식 풀기
        det = a11 * a22 - a12 * a12
        if abs(det) < 1e-12:
            det = 1e-12 * (1.0 if det >= 0 else -1.0)

        theta_ddot = (a22 * rhs1 - a12 * rhs2) / det
        x_ddot = (a11 * rhs2 - a12 * rhs1) / det

        return np.array([theta_dot, theta_ddot, x_dot, x_ddot])

    def _rk4_step(self, state, F):
        """4차 Runge-Kutta 적분 (정밀한 물리 시뮬레이션)"""
        dt = self.dt
        k1 = self._dynamics(state, F)
        k2 = self._dynamics(state + 0.5 * dt * k1, F)
        k3 = self._dynamics(state + 0.5 * dt * k2, F)
        k4 = self._dynamics(state + dt * k3, F)
        return state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    # ─────────────────────────────────────────────────────────────────
    #  도메인 랜덤화
    # ─────────────────────────────────────────────────────────────────

    def _randomize_params(self):
        """매 에피소드마다 물리 파라미터를 랜덤하게 변경"""
        p = {k: v for k, v in self._base_params.items()}

        if self.domain_randomize:
            p['body_mass'] *= self.np_random.uniform(0.7, 1.3)
            p['com_height'] *= self.np_random.uniform(0.8, 1.2)
            p['friction_rot'] *= self.np_random.uniform(0.5, 2.0)
            p['friction_trans'] *= self.np_random.uniform(0.5, 2.0)

        # 관성모멘트 계산 (봉 근사: I = mL²/3)
        p['body_inertia'] = p['body_mass'] * p['com_height'] ** 2 / 3.0

        # 최대 접지 힘 (양쪽 바퀴 합)
        p['max_force'] = 2.0 * p['max_torque_per_wheel'] / p['wheel_radius']

        self.params = p


# ─── 환경 등록 (gymnasium) ───
gym.register(
    id='WBRBalance-v0',
    entry_point='wbr_sim_env:WBRBalanceEnv',
    max_episode_steps=3750,
)


# ─── 테스트용 ───
if __name__ == '__main__':
    env = WBRBalanceEnv(domain_randomize=False)
    obs, _ = env.reset()
    print(f"초기 관측: {obs}")
    print(f"관측 공간: {env.observation_space}")
    print(f"행동 공간: {env.action_space}")
    print(f"물리 파라미터: {env.params}")

    # 랜덤 행동으로 테스트
    total_reward = 0
    for i in range(500):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        if terminated:
            print(f"  넘어짐! step={i}, pitch={math.degrees(info['theta']):.1f}°")
            break

    print(f"총 보상: {total_reward:.1f}")
    print(f"생존 시간: {env.step_count * env.dt:.2f}초")
