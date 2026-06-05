clc; clear; close all;
addpath("lib\");
format short;

load('dynamic_properties.mat');
load('dynamics_functions.mat');

%% ==========================================================
% 1. 제약 조건 및 상수 설정
% ==========================================================
MAX_TORQUE = 0.75;      % 모터 최대 물리적 한계 (Nm)
V_MAX      = 1.2;       % V_err 클램핑 상한선 (m/s)
D_GAIN_MAX = 0.165;     % 진동 방지 최대 D게인

Ts = 0.008;
nominal_h = 0.13;
phi = 0;

%% 2. 기준 모델 추출
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

%% 3. Q[2] 고정 및 Q[0] 스윕을 통한 예산 매핑
R_val = 150; % 비교를 위해 R은 고정
R_mat = diag([R_val, R_val]);

fprintf('========================================================================\n');
fprintf('     [ Q[2] 고정 (5.0 ~ 7.5) & Q[0] 스윕: 토크 예산 분배 리포트 ]\n');
fprintf('========================================================================\n');
fprintf(' Q[0] | Q[2] |  K_P(각도)  |  K_V(속도)  | V소모예산 | P남은예산 | 최대버티기\n');
fprintf('------------------------------------------------------------------------\n');

best_Q0 = 0; best_Q2 = 0; opt_K = zeros(2,4);
max_angle_record = 0;

for q2 = 5.0 : 1.25 : 7.5     % 5.0, 6.25, 7.5 탐색
    for q0 = 80 : 20 : 160    % P 게인 스윕
        
        Q_mat = diag([q0, 0, q2, 5]);
        [K, ~, ~] = dlqr(Ad, Bd, Q_mat, R_mat);
        
        % D게인 상한선 체크
        if K(1,2) > D_GAIN_MAX
            continue; 
        end
        
        % 예산 계산 (V항 클램핑 1.5 기준)
        v_budget = abs(K(1,3)) * V_MAX;
        p_budget = MAX_TORQUE - v_budget;
        
        if p_budget <= 0
            continue;
        end
        
        % 버티기 각도 계산
        theta_max_deg = (p_budget / K(1,1)) * (180 / pi);
        
        % 최고 기록 저장
        if theta_max_deg > max_angle_record
            max_angle_record = theta_max_deg;
            best_Q0 = q0;
            best_Q2 = q2;
            opt_K = K;
        end
        
        % 결과 표 출력
        fprintf(' %3d  | %4.1f |   %8.4f  |  %8.4f  |  %4.3f Nm |  %4.3f Nm |  %5.1f 도\n', ...
            q0, q2, K(1,1), K(1,3), v_budget, p_budget, theta_max_deg);
    end
    fprintf('------------------------------------------------------------------------\n');
end

fprintf('\n[결론] 생존력(최대 각도)이 가장 높은 스위트 스팟: Q[0]=%d, Q[2]=%.1f\n\n', best_Q0, best_Q2);

%% 4. 선택된 최적 파라미터 기반 C++ 코드 생성
best_Q = diag([best_Q0, 0, best_Q2, 5]);

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
    
    K_d = dlqr(sys_d.A, sys_d.B, best_Q, R_mat);
    Ks = [Ks; K_d];
end

[numRows, ~] = size(Ks);
for i = 1:numRows/2
    fprintf("mat << ");
    fprintf('   % .8ff,  % .8ff,  % .8ff,  % .8ff,\n', Ks(2*i-1,1), Ks(2*i-1,2), Ks(2*i-1,3), Ks(2*i-1,4));
    fprintf('   % .8ff,  % .8ff,  % .8ff,  % .8ff;\n', Ks(2*i,1), Ks(2*i,2), Ks(2*i,3), Ks(2*i,4));
    fprintf('Ks.push_back(mat);\n\n');
end