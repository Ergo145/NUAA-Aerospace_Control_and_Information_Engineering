clear
close all
clc
% Constants
w_rf = 2 * pi * 1575.42e6;           % L1 nominal frequncy, unit: rad/s
speed_of_light = 2.99792458e8;       % Unit: m/s
% Design Parameters
n = 3;                               % Dim of state
T = 0.02;                            % PIT, unit: s
C_N0 = 35;                           % Unit: dB-Hz
% Clock model
h0 = 2e-25;                          % Unit: s
h_2 = 6e-25;                         % Unit: 1/s
% h0 = 2e-19;                          % Unit: s
% h_2 = 3e-20;                         % Unit: 1/s
Sf = h0 / 2;                     
Sg = 2 * pi^2 * h_2;
% Dynamic
Sa = (0.2)^2;                               % LOS acceleration random walk PSD, Unit: m^2/s^5
F = [0 1 0; 0 0 1; 0 0 0];                      % System matrix
G = diag([w_rf, w_rf, w_rf / speed_of_light]);  % Processing matrix
Q = diag([Sf, Sg, Sa]);
H = [1 -T/2 T^2/6];                         % Measurement matrix
% Discrete time matrix computation
[Phi, Qd] = disc_model(F, G, Q, T);
c_n0 = 10^(C_N0 / 10);               % Unit: Hz
R0 = 1/(2 * c_n0 * T) * (1 + 1/(2 * c_n0 * T));    % Unit: rad^2
R = R0;
%% Convariance Analysis
% Optimal gain
T_end = 1;                          % Unit: s
N = T_end / T;                       % Num of samples
P_init = diag([2*pi, 2*pi*1000, 0].^2);
P_pred = P_init;
K = zeros(3, N);
Pn_err = zeros(3, N);
Pp_err = zeros(3, N);
for k = 1: N
    K(:, k) = P_pred * H' / (H * P_pred * H' + R);
    P_update = (eye(n) - K(:, k) * H) * P_pred;
    P_pred = Phi * P_update * Phi' + Qd;
    Pp_err(:, k) = diag(P_update);
    Pn_err(:, k) = diag(P_pred);
end

%% Error budget
phase_err = zeros(5, N);
% Error driven by carrier phase random walk
P_pred = zeros(3);
Q = diag([Sf, 0, 0]);
R = 0;
[Phi, Qd] = disc_model(F, G, Q, T);
for k = 1: N
    P_update = (eye(3) - K(:, k) * H) * P_pred * (eye(3) - K(:, k) * H)' + K(:, k) * R * K(:, k)';
    P_pred = Phi * P_update * Phi' + Qd;
    phase_err(1, k) = P_update(1, 1);
end
% Error driven by carrier frequency random walk
P_pred = zeros(3);
Q = diag([0, Sg, 0]);
R = 0;
[Phi, Qd] = disc_model(F, G, Q, T);
for k = 1: N
    P_update = (eye(3) - K(:, k) * H) * P_pred * (eye(3) - K(:, k) * H)' + K(:, k) * R * K(:, k)';
    P_pred = Phi * P_update * Phi' + Qd;
    phase_err(2, k) = P_update(1, 1);
end
% Error driven by carrier acceleration random walk
P_pred = zeros(3);
Q = diag([0, 0, Sa]);
R = 0;
[Phi, Qd] = disc_model(F, G, Q, T);
for k = 1: N
    P_update = (eye(3) - K(:, k) * H) * P_pred * (eye(3) - K(:, k) * H)' + K(:, k) * R * K(:, k)';
    P_pred = Phi * P_update * Phi' + Qd;
    phase_err(3, k) = P_update(1, 1);
end
% Error driven by carrier phase measurement noise
P_pred = zeros(3);
Q = zeros(3);
R = R0;
[Phi, Qd] = disc_model(F, G, Q, T);
for k = 1: N
    P_update = (eye(3) - K(:, k) * H) * P_pred * (eye(3) - K(:, k) * H)' + K(:, k) * R * K(:, k)';
    P_pred = Phi * P_update * Phi' + Qd;
    phase_err(4, k) = P_update(1, 1);
end
% Error driven by init P
P_pred = P_init;
Q = zeros(3);
R = 0;
[Phi, Qd] = disc_model(F, G, Q, T);
for k = 1: N
    P_update = (eye(3) - K(:, k) * H) * P_pred * (eye(3) - K(:, k) * H)' + K(:, k) * R * K(:, k)';
    P_pred = Phi * P_update * Phi' + Qd;
    phase_err(5, k) = P_update(1, 1);
end
figure(1)
% semilogy((1: N)*T, rad2deg(phase_err(1, :).^0.5))
semilogy((1: N)*T, rad2deg(phase_err(1, :).^0.5), '-ob', 'LineWidth', 1, 'MarkerSize', 4)
hold on
semilogy((1: N)*T, rad2deg(phase_err(2, :).^0.5), '-*r', 'LineWidth', 1, 'MarkerSize', 4)
semilogy((1: N)*T, rad2deg(phase_err(3, :).^0.5), '-+g', 'LineWidth', 1, 'MarkerSize', 4)
semilogy((1: N)*T, rad2deg(phase_err(4, :).^0.5), '-xm', 'LineWidth', 1, 'MarkerSize', 4)
semilogy((1: N)*T, rad2deg(phase_err(5, :).^0.5), '-dk', 'LineWidth', 1, 'MarkerSize', 4)
% semilogy((1: N)*T, rad2deg(Pp_err(1, :).^0.5), '-', 'LineWidth', 2)
% 加上了图片效果比较差
hold off
legend('q_b', 'q_d', 'q_\alpha', 'R', 'P_0', 'Location', 'north', 'Orientation', 'horizontal')
xlabel('Time (s)')
% title('TCXO')
ylabel('\sigma_\Delta_\phi (\circ)')
fig = gcf;
fig.Units = 'centimeter';
fig.Position = [5 5 12 8];
ax = gca;
ax.FontSize = 10;
ax.FontWeight = 'bold';
ax.TitleFontWeight = 'normal';
ax.YTickMode = 'manual';
ax.XTickMode = 'manual';
ax.XTick = [0, 0.2, 0.4, 0.6, 0.8, 1.0];
%% 计算各误差在稳态误差中的占比
figure(2)
x = [sum(phase_err(1: 2, end)), phase_err(3: 4, end)'] / sum(phase_err(1: 4, end));
names = {'q_b+q_d:'; 'q_a: '; 'R: '};
PlotPie(x, names)
disp(rad2deg([sum(phase_err(1: 2, end)), phase_err(3: 4, end)'].^0.5))

%%设计的增益阵
figure(3)
plot((1:N)*T,K(1,:),'-o',(1:N)*T,K(2,:),'-*',(1:N)*T,K(3,:),'--')
xlabel('Time(s)'),ylabel('增益阵')
legend('K(1)','K(2)','K(3)')