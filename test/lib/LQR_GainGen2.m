clc; clear; close all;
addpath("lib\");
format long;

load('dynamic_properties.mat');
load('dynamics_functions.mat');

%% ==========================================================
% 1. 물리적 한계 및 타겟 목표 설정 (Constraints & Target)
% ==========================================================
MAX_TORQUE = 0.75;      % 모터의 절대 물리적 한계 (Nm)
V_MAX      = 1.0;       % 클램핑으로 제한한 최대 속도 오차 (m/s)
D_GAIN_MAX = 0.165;     % 센서 노이즈가 증폭되어 진동하지 않는 최대 D게인 상한선

% [핵심 추가] 최소한 보장받고 싶은 "타겟 복원 각도" (도 단위)
TARGET_THETA_DEG = 25.0; 

Ts = 0.008;             % 샘플링 타임 (125Hz)
nominal_h = 0.13;       % 탐색 기준 높이 (CoM)
phi = 0;

%% 2. 기준 모델(Nominal Model) 추출
model = Pol(properties, dynamic_functions);
model.setState(zeros(4,1), zeros(2,1), nominal_h);
theta_eq = model.get_theta_eq(nominal_h, phi);
x_eq = [theta_eq; 0; 0; 0];
u_eq = zeros(2,1);
model.setState(x_eq, u_eq, nominal_h);
model.calculateDynamics();
model.calculateJacobian();

A_ = calculate_fx(model.M, model.dM_dtheta, model.nle, model.dnle_dtheta, model.dnle_dqdot);
B_ = calculate_fu(model.M, model.B);

sys_c = ss(A_, B_, [], []);
sys_d = c2d(sys_c, Ts, 'zoh');
Ad = sys_d.A;
Bd = sys_d.B;

%% 3. 타겟 복원 각도 보장형 파라미터 스윕
best_Q = diag([0 0 0 0]);
best_R = diag([150 150]);
best_score = inf; 
opt_theta_max_deg = 0;

fprintf('==========================================\n');
fprintf('타겟 복원 각도(>%.1f도) 보장 최적 게인 탐색 시작...\n', TARGET_THETA_DEG);

for q0 = 80:5:250        % 각도 패널티 스윕
    for q2 = 5:2.5:20    % 속도 패널티 스윕
        for r_val = 100:10:200 % 토크 패널티 스윕
            
            Q_test = diag([q0, 0, q2, 5]);
            R_test = diag([r_val, r_val]);
            
            [K_test, ~, ~] = dlqr(Ad, Bd, Q_test, R_test);
            
            % [안전장치] D게인 진동 방지
            if K_test(1,2) > D_GAIN_MAX
                continue; 
            end
            
            % --- [핵심 계산] 이 게인으로 낼 수 있는 최대 복원 각도는? ---
            % 1. V항이 최대로 갉아먹는 예산을 뺀 'P항 전용 예산' 계산
            tau_P_avail = MAX_TORQUE - abs(K_test(1,3)) * V_MAX;
            
            % V항이 너무 커서 남은 예산이 없으면 탈락
            if tau_P_avail <= 0 
                continue; 
            end
            
            % 2. 남은 예산으로 버틸 수 있는 최대 각도 계산 (포화 임계점)
            theta_max_rad = tau_P_avail / K_test(1,1);
            theta_max_deg_val = theta_max_rad * 180 / pi;
            
            % [Gatekeeper] 타겟 복원 각도(20도)를 못 넘기면 무조건 탈락!
            if theta_max_deg_val < TARGET_THETA_DEG
                continue;
            end
            
            % --- [Objective Function] 응답성 최적화 ---
            % 20도를 버틸 수 있는 녀석들 중에서 가장 수렴 속도가 빠른 것을 고름
            poles = eig(Ad - Bd * K_test);
            max_pole_mag = max(abs(poles));
            
            if max_pole_mag < best_score
                best_score = max_pole_mag;
                best_Q = Q_test;
                best_R = R_test;
                opt_K = K_test;
                opt_theta_max_deg = theta_max_deg_val;
            end
        end
    end
end

if isinf(best_score)
    fprintf('[경고] 설정하신 조건(%.1f도 버티기)을 만족하는 게인이 존재하지 않습니다.\n', TARGET_THETA_DEG);
    fprintf('모터 체급을 올리거나, V_MAX를 줄이거나, TARGET 각도를 낮춰주세요.\n');
else
    fprintf('탐색 완료! 성능과 생존력을 모두 갖춘 스위트 스팟:\n');
    fprintf('Q = diag([%.1f, %.1f, %.1f, %.1f])\n', best_Q(1,1), best_Q(2,2), best_Q(3,3), best_Q(4,4));
    fprintf('R = diag([%.1f, %.1f])\n', best_R(1,1), best_R(2,2));
    fprintf('==========================================\n');
    fprintf('★ 100%% 토크 풀파워 시 최대 복원 가능 각도: %.2f 도\n', opt_theta_max_deg);
    fprintf('   - V항 최대 소모 예산: %.3f Nm\n', opt_K(1,3)*V_MAX);
    fprintf('   - P항 확보 가능 예산: %.3f Nm\n', MAX_TORQUE - opt_K(1,3)*V_MAX);
    fprintf('★ D 게인 (노이즈 민감도): %.3f (상한: %.3f)\n', opt_K(1,2), D_GAIN_MAX);
    fprintf('==========================================\n\n');
end

%% 4. 최적 파라미터 기반 전구간(Height) 게인 행렬 C++ 코드 생성
if ~isinf(best_score)
    Ks = [];
    for h = 0.07:0.01:0.2
        model.setState(zeros(4,1), zeros(2,1), h);
        theta_eq = model.get_theta_eq(h, phi);
        x_eq = [theta_eq; 0; 0; 0];
        u_eq = zeros(2,1);
        model.setState(x_eq, u_eq, h);
        model.calculateDynamics();
        model.calculateJacobian();
        
        A_ = calculate_fx(model.M, model.dM_dtheta, model.nle, model.dnle_dtheta, model.dnle_dqdot);
        B_ = calculate_fu(model.M, model.B);
        
        sys_c = ss(A_, B_, [], []);
        sys_d = c2d(sys_c, Ts, 'zoh');
        
        K_d = dlqr(sys_d.A, sys_d.B, best_Q, best_R);
        Ks = [Ks; K_d];
    end

    [numRows, ~] = size(Ks);
    for i = 1:numRows/2
        fprintf("mat << ");
        fprintf('   % .8ff,  % .8ff,  % .8ff,  % .8ff,\n', Ks(2*i-1,1), Ks(2*i-1,2), Ks(2*i-1,3), Ks(2*i-1,4));
        fprintf('   % .8ff,  % .8ff,  % .8ff,  % .8ff;\n', Ks(2*i,1), Ks(2*i,2), Ks(2*i,3), Ks(2*i,4));
        fprintf('Ks.push_back(mat);\n\n');
    end
end