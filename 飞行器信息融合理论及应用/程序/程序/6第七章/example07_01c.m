%{
功能描述：灵敏度分析之设计性能与最优性能间的比较(qa)
作    者：Jr9910
日    期：2014/12/22
版    本：V1.0
%}
clear
close all
clc
% Constants
w_rf = 2 * pi * 1575.42e6;           % L1 nominal frequncy, unit: rad/s
speed_of_light = 2.99792458e8;       % Unit: m/s
% Design Parameters
n_st = 3;                               % Dim of state
T = 0.02;                            % PIT, unit: s
C_N0 = 35;                           % Unit: dB-Hz
% Clock model
h0 = 2e-25;                          % Unit: s
h_2 = 6e-25;                         % Unit: 1/s
Sf = h0 / 2;                     
Sg = 2 * pi^2 * h_2;
% Dynamic
F = [0 1 0; 0 0 1; 0 0 0];                      % System matrix
G = diag([w_rf, w_rf, w_rf / speed_of_light]);  % Processing matrix
H = [1 -T/2 T^2/6];                         % Measurement matrix
% Discrete time matrix computation
R = CalMeasVar(C_N0, T);
%% Convariance Analysis
% Optimal gain
T_end = 1.5;                                       % Unit: s
N = T_end / T;                                     % Num of samples
P_init = diag([0, 0, 0].^2);
n_trials = 6;
K = zeros(3, N, n_trials);
Pp_err = zeros(1, n_trials);                       % 最优条件下相位标准差
jerk_set = 0.001;                                  % jerk, Units: g/s
for n = 1: n_trials
    P_pred = P_init;
    Sa = (10^(n-1) * jerk_set * 9.8)^2 * T;        % LOS acceleration random walk PSD, Unit: m^2/s^5
    Q = diag([Sf, Sg, Sa]);
    [Phi, Qd] = disc_model(F, G, Q, T);    
    for k = 1: N
        K(:, k, n) = P_pred * H' / (H * P_pred * H' + R);
        P_update = (eye(n_st) - K(:, k, n) * H) * P_pred;
        P_pred = Phi * P_update * Phi' + Qd;
    end
    Pp_err(n) = rad2deg(P_update(1, 1).^0.5);
end
%% Sensitivity Analysis
jerk_set = 0.001; 
P_err = zeros(1, n_trials);
n_set = 5;                                        % 选择设计点，实际上是选择前面计算的增益
for n = 1: n_trials
    P_pred = P_init;
    Sa = (10^(n-1) * jerk_set * 9.8)^2 * T;
    Q = diag([Sf, Sg, Sa]);
    [Phi, Qd] = disc_model(F, G, Q, T);  
    for k = 1: N
        P_update = (eye(n_st) - K(:, k, n_set) * H) * P_pred * (eye(n_st) - K(:, k, n_set) * H)' + K(:, k, n_set) * R * K(:, k, n_set)';
        P_pred = Phi * P_update * Phi' + Qd;
    end
    P_err(n) = rad2deg(P_update(1, 1)^0.5);
end
figure
semilogx(0.001*10.^(0: 5), P_err, '-o')
hold on
semilogx(0.001*10.^(0: 5), Pp_err, '-*')
grid on
hold off
legend('设计性能', '最优性能')
% title('Optimal vs. Design')
xlabel('\surdq_a(g/s/\surdHz)')
ylabel('\sigma_\phi(deg)')