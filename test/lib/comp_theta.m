clc; clear; close all;
addpath("lib\"); % 필수: Pol.m 등 라이브러리 경로 추가
format long;

% 1. CSV 데이터 읽기 및 측정 각도 계산
filename = 'runmode_bwd.csv';
data = readtable(filename);

% 마지막 30% 데이터만 평균 (수렴 후 구간)
% n = height(data);
% stable_idx = round(n * 0.7) : n;
% theta_hat_rad = mean(data.theta_hat(stable_idx));
% theta_hat_deg = theta_hat_rad * 180 / pi; % 디그리 변환
theta_hat_deg = 2.26;
theta_hat_rad = theta_hat_deg * pi / 180;  % ← 이거 추가

fprintf('1. 측정된 평형 각도 (theta_hat): %.4f°\n', theta_hat_deg);

% 2. 물리 모델 로드 및 기준 평형각 계산
load('dynamic_properties.mat');
load('dynamics_functions.mat');

h = 0.13;
phi = 0;
model = Pol(properties, dynamic_functions);
theta_eq_ref_rad = model.get_theta_eq(h, phi);
theta_eq_ref_deg = theta_eq_ref_rad * 180 / pi;

fprintf('2. 모델 기준 평형 각도 (theta_eq_ref): %.4f°\n', theta_eq_ref_deg);

% 3. 편차(Delta) 계산
delta_deg = theta_hat_deg - theta_eq_ref_deg;
fprintf('3. 각도 편차 (Delta): %.4f°\n', delta_deg);

% 4. 새로운 CoM_x 역산 및 갱신 가이드 출력
CoM_z_ref = 0.03492;
CoM_x_new = -tan(theta_hat_rad) * (h + CoM_z_ref);

fprintf('==================================================\n');
if abs(delta_deg) < 1.0
    fprintf('[OK] 편차가 1° 미만입니다. 현재 게인을 그대로 사용해도 좋습니다.\n');
elseif abs(delta_deg) < 2.0
    fprintf('[WARN] 편차가 1~2° 사이입니다. 우선 테스트해보고 필요시 아래 값을 갱신하세요.\n');
else
    fprintf('[FAIL] 편차가 2° 이상입니다. 무게중심 보정 파이프라인을 실행하세요!\n');
end
fprintf('--------------------------------------------------\n');
fprintf('>>> 새롭게 갱신할 CoM_x (m 단위): %.8f\n', CoM_x_new);
fprintf('>>> generate_dynamic_properties.m 에 붙여넣을 값 (mm 단위): %.5f\n', CoM_x_new * 1000);
fprintf('==================================================\n');