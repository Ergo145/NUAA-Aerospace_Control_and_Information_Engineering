%% Phase 1 [I-85]
% main.m
clear;clc;

gyro = importdata('./first/Gyroscope.csv');
gps = importdata('./first/GPS.csv');
acc = importdata('./first/Accelerometer.csv');

start_index = 2493;
end_index = 13475;
gps_delay = 2   ; % [s]
gps_delay_index = gps_delay*10; % [s]

data_gyro = gyro.data(1:end_index,2:end);
data_gps = gps.data(1+gps_delay_index:end_index+gps_delay_index,2:end);
data_acc = acc.data(1:end_index,2:end);

[data_size,~] = size(data_gps);

dt = 0.1;    % [s]

% Initialization

% orientation
% body frame NED
angle_0 = [-pi/2, 0, pi/2]; % initial frame angle from NED, Z-Y-X to phone body frame (NED -> phone)
dcm_0 = angle2dcm(-angle_0(1),-angle_0(2),-angle_0(3),'ZYX');  % following DCM in AircraftAndSim book
quat_0 = angle2quat(-angle_0(1),-angle_0(2),-angle_0(3),'ZYX');

X0 = [0;            % x [m] (NED)
      0;            % y [m] (NED)
      0;            % z [m] (NED)
      quat_0';      % orientation [rad] (NED quat)
      0;            % v_x [m/s] (vehicle frame)
      0;            % v_y [m/s] (vehicle frame)
      0];            % v_z [m/s] (vehicle frame)
%       zeros(3,1)];  % angular vel [rad/s] (vehicle frame)
      % (10,1)

[n,~] = size(X0);
      
Y0 = [33.78207;         % Latitude
      -84.39507         % Longitude
      278];             % Altitude
  
%%% Variance & bias correction

g = 9.807;
  
P0 = diag([100,100,100,...
            0.0001,0.00001,0.00001,0.00001,...
            0.1,0.1,0.1]);
%             deg2rad(10),deg2rad(10),deg2rad(10)]);
        
Q = diag([1.755e-2,1.352e-2,0.983e-2,...
        9.3639e-05,2.7136e-06,2.6480e-07,7.0453e-08,...
        1.755e-2,1.352e-2,0.983e-2]);
%         0.0867,0.0195,0.07]);       % State error covariance matrix
    
R = diag([1.1964e-12,1.1964e-12,0.0947]);   % measurment error covariance matrix

data_acc(:,1) = data_acc(:,1) - 0.0251;
data_acc(:,2) = data_acc(:,2) + 0.1228 - g;
data_acc(:,3) = data_acc(:,3) - 0.0409;

% data_gyro(:,1) = data_acc(:,1) + 0.0023;
% data_gyro(:,2) = data_acc(:,2) - 9.5031e-4;
% data_gyro(:,3) = data_acc(:,3) - 0.0047;

% Unscented Kalman Filter Algorithm
tic
for k = 1:data_size
    if k == 1
        X_post = X0;
        P_post = P0;
        prop_nums = 0;  % detect 1s update of gps even if it do not move
        gps_prev = Y0';
        X_est = [X0];
        y_est = [Y0];
    else
        gps_prev = data_gps(k-1,:);
        X_post = X_post_new;
        P_post = P_post_new;
    end
    
    acc_k = data_acc(k,:);      % (1,3)
    gyro_k = data_gyro(k,:);    % (1,3)
    gps_k = data_gps(k,:);      % (1,3)

    P_var = diag(P_post);
    pos_var = P_var(1:3);

    gps_diffalt = gps_k(3) - gps_prev(3);   % use diff for alt b/c lat long diff is too small
    gps_diff2d = norm(gps_k(1:2) - gps_prev(1:2));

    % GPS ignorance condition
%     propagate = 1;                          % mere IMU propagation
%     propagate = ((gps_diff2d == 0) && (prop_nums < 10));     % not filtering 
    propagate = (gps_diff2d > 3*sqrt(sum(pos_var))) ||...
            ((gps_diff2d == 0) && (prop_nums < 30));
    
    %%%%%%%%%%%%%%%%%%%%%%%%% UKF Filtering %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%% Time update %%%%%%%
    % 1. generate sigma point
    [root,p] = chol(n*P_post);
    
    for j = 1:n
        sigma(:,j) = X_post + root(j,:)';
        sigma(:,j+n) = X_post - root(j,:)';
    end
    
    % 2. propagate with nonlin
     for j = 1:2*n
         dX = X_kinematics(sigma(:,j), acc_k, gyro_k)*dt;
         prop_sigma(:,j) = sigma(:,j) + dX;
     end
     
     % 3. generate apriori of x
     X_prior = zeros(n,1);
     for j = 1:2*n
         X_prior = X_prior + prop_sigma(:,j);
     end
     X_prior = X_prior./(2*n);

     % 4. generate apriori of P
     P_prior = zeros(n,n);
     for j = 1:2*n
         P_prior = P_prior + (prop_sigma(:,j) - X_prior)*...
                             (prop_sigma(:,j) - X_prior)';
     end
     P_prior = P_prior/(2*n) + Q;
     
     if propagate
         X_post_new = X_prior;
         P_post_new = P_prior;
         prop_nums = prop_nums + 1;
         
     else
         %%%%%%% Measurment update %%%%%%% (updated every 1s with gps)
         % 1. generate sigma points
         [root,p] = chol(n*P_prior);

        for j = 1:n
            sigma(:,j) = X_prior + root(j,:)';
            sigma(:,j+n) = X_prior - root(j,:)';
        end

        % 2. generate measurment to sigma points
        for j = 1:2*n
            dsigma_y = ned2gps(sigma(:,j), Y0);
            sigma_y(:,j) = Y0 + dsigma_y;
        end

        % 3. combine to generate predicted meas at time t_k
        y_hat = zeros(3,1);
         for j = 1:2*n
             y_hat = y_hat + sigma_y(:,j);
         end
         y_hat = y_hat/(2*n);

         % 4. generate measurement variance
%          R = diag([25,deg2rad(0.1)^2,deg2rad(0.1)^2]);
         Py = zeros(3,3);
         for j = 1:2*n
             Py = Py + (sigma_y(:,j) - y_hat)*...
                             (sigma_y(:,j) - y_hat)';
         end
         Py = Py/(2*n) + R; % (3*3)

         % 5. estimate cross cov between x,y
         Pxy = zeros(n,3);
         for j = 1:2*n
             Pxy = Pxy + (prop_sigma(:,j) - X_prior)*...
                             (sigma_y(:,j) - y_hat)'; % (13*1)*(1*3)
         end
         Pxy = Pxy/(2*n); % (6*3)

         % 6. Kalman Gain
         K = Pxy*pinv(Py); % (6*3)

         % 7. compute Measurement update
         X_post_new = X_prior + K*(gps_k' - y_hat);
         P_post_new = P_prior - K * Py * K';
         
         prop_nums = 0;
     end
     X_est = [X_est, X_post_new];
     dy = ned2gps(X_post_new,Y0);
     y_est = [y_est, dy+Y0];
end
toc

%% plot the result

% 3D轨迹图 - 路径1
figure(1)
plot3(X_est(2,1:end),X_est(1,1:end),-X_est(3,1:end))
grid on
xlabel('East [m]')
ylabel('North [m]')
zlabel('Height [m]')
title('3D Trajectory - Path 1 (I-85 Highway)')
legend('UKF Estimated Path')

% 2D平面图 - 路径1
figure(2)
plot(X_est(2,1:end),X_est(1,1:end), 'LineWidth', 2)
grid on
xlabel('East [m]')
ylabel('North [m]')
title('2D Trajectory - Path 1 (I-85 Highway)')
legend('UKF Estimated Path')

% 卫星地图 - 路径1
figure(3)
geoplot(y_est(1,:),y_est(2,:),'r','LineWidth',2)
geobasemap('satellite')
title('Satellite View - Path 1 (I-85 Highway)')
hold on
geoplot(data_gps(:,1),data_gps(:,2),'b.','LineWidth',1)
hold off
legend('UKF Filtered','Raw GPS','Location','best')

%卫星图UKF预测轨迹
figure(4)
geoplot(y_est(1,:),y_est(2,:),'r','LineWidth',2)
geobasemap('satellite')
title('UKF Estimated Path1 in Satellite View(I-85 Highway)')
hold on
legend('UKF Filtered','Location','best')
%% 误差
%% ---- 轨迹误差分析 ----
earth_radius = 6371000; % 地球半径 [m]

% 保证长度一致
min_len = min(size(y_est,2), size(data_gps,1));
y_est = y_est(:,1:min_len);
data_gps = data_gps(1:min_len,:);

% 经纬度、高度误差（单位：m）
lat_err_m = (y_est(1,:) - data_gps(:,1)') * (pi/180) * earth_radius;
lon_err_m = (y_est(2,:) - data_gps(:,2)') * (pi/180) * earth_radius .* cosd(data_gps(:,1)');
alt_err_m = (y_est(3,:) - data_gps(:,3)');

% 误差时间轴
t = (0:length(lat_err_m)-1) * dt;

% 绘制误差随时间变化图
figure(5);
subplot(3,1,1)
plot(t, lat_err_m, 'LineWidth', 1.5);
xlabel('Time [s]')
ylabel('Latitude Error [m]')
title('UKF Latitude Error vs GPS - Path 1')

subplot(3,1,2)
plot(t, lon_err_m, 'LineWidth', 1.5);
xlabel('Time [s]')
ylabel('Longitude Error [m]')
title('UKF Longitude Error vs GPS - Path 1')

subplot(3,1,3)
plot(t, alt_err_m, 'LineWidth', 1.5);
xlabel('Time [s]')
ylabel('Altitude Error [m]')
title('UKF Altitude Error vs GPS- Path 1')

sgtitle('UKF Trajectory Estimation Errors - Path 1')

% % 三维误差可视化（可选）
% figure(6);
% plot3(lon_err_m, lat_err_m, alt_err_m, 'r', 'LineWidth', 1.5)
% grid on
% xlabel('Longitude Error [m]')
% ylabel('Latitude Error [m]')
% zlabel('Altitude Error [m]')
% title('3D View of UKF Trajectory Errors')

%% 
[~,y_index]=size(y_est);
num_csvs = fix(y_index / 2000);

for i=1:num_csvs-1
    y_est(:,i*2000+1:(i+1)*2000+1);
end

% for i = 1:num_csvs-1
%     writematrix(y_est(:,num_csvs*2000+1:(num_csvs+1)*2000+1),'Est_y.csv')
% end


%% Phase 2 [Non-highway]

clear;
gyro = importdata('./second/Gyroscope.csv');
gps = importdata('./second/GPS.csv');
acc = importdata('./second/Accelerometer.csv');

start_index = 1000;
end_index = 11500;
gps_delay = 0; % [s]
gps_delay_index = gps_delay*10; % [s]

data_gyro = gyro.data(1:end_index,2:end);
data_gps = gps.data(1+gps_delay_index:end_index+gps_delay_index,2:end);
data_acc = acc.data(1:end_index,2:end);

[data_size,~] = size(data_gps);

dt = 0.1;    % [s]

% Initialization

% orientation
% body frame NED
angle_0 = [pi/2, 0, deg2rad(-50)]; % initial angle from NED, Z-Y-X from phone to body frame
% dcm_0 = angle2dcm(angle_0(1),angle_0(2),angle_0(3),'ZYX');
quat_0 = angle2quat(-angle_0(1),-angle_0(2),-angle_0(3),'ZYX');

X0 = [0;            % x [m] (NED)
      0;            % y [m] (NED)
      0;            % z [m] (NED)
      quat_0';      % orientation [rad] (NED quat)
      0;            % v_x [m/s] (phone frame)
      0;            % v_y [m/s] (phone frame)
      0];            % v_z [m/s] (phone frame)
%       zeros(3,1)];  % angular vel [rad/s] (phone frame)
      % (10,1)

[n,~] = size(X0);
      
Y0 = [33.908439;         % Latitude
     -84.287319;         % Longitude
      311];             % Altitude
  
%%% Variance & bias correction

g = 9.807;
  

P0 = diag([100,100,100,...
            0.0001,0.0001,0.0001,0.0001,...
            0.1,0.1,0.1]);
%             deg2rad(10),deg2rad(10),deg2rad(10)]);
        
Q = diag([1.755e-2,1.352e-2,0.983e-2,...
        9.3639e-05,2.7136e-06,2.6480e-07,7.0453e-08,...
        1.755e-2,1.352e-2,0.983e-2]);
    
R = diag([1.1964e-12,1.1964e-12,0.0947]);   % measurment error covariance matrix

data_acc(:,1) = data_acc(:,1) - 0.0251;
data_acc(:,2) = data_acc(:,2) + 0.1228 - g;
data_acc(:,3) = data_acc(:,3) - 0.0409;

% data_gyro(:,1) = data_acc(:,1) + 0.0023;
% data_gyro(:,2) = data_acc(:,2) - 9.5031e-4;
% data_gyro(:,3) = data_acc(:,3) - 0.0047;

% Unscented Kalman Filter Algorithm
tic
for k = 1:data_size
    if k == 1
        X_post = X0;
        P_post = P0;
        prop_nums = 0;  % detect 1s update of gps even if it do not move
        gps_prev = Y0';
        X_est = [X0];
        y_est = [Y0];
    else
        gps_prev = data_gps(k-1,:);
        X_post = X_post_new;
        P_post = P_post_new;
    end
    
    acc_k = data_acc(k,:);      % (1,3)
    gyro_k = data_gyro(k,:);    % (1,3)
    gps_k = data_gps(k,:);      % (1,3)

    P_var = diag(P_post);
    pos_var = P_var(1:3);

    gps_diffalt = gps_k(3) - gps_prev(3);   % use diff for alt b/c lat long diff is too small
    gps_diff2d = norm(gps_k(1:2) - gps_prev(1:2));

%    % GPS ignorance condition
%     propagate = 1;                          % mere IMU propagation
    propagate = ((gps_diff2d == 0) && (prop_nums < 10));     % not filter out 3sigma
%     propagate = (gps_diff2d > 3*sqrt(sum(pos_var))) ||...
%             ((gps_diff2d == 0) && (prop_nums < 30));
    
    %%%%%%%%%%%%%%%%%%%%%%%%% UKF Filtering %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%% Time update %%%%%%%
    % 1. generate sigma point
    [root,p] = chol(n*P_post);
    
    for j = 1:n
        sigma(:,j) = X_post + root(j,:)';
        sigma(:,j+n) = X_post - root(j,:)';
    end
    
    % 2. propagate with nonlin
     for j = 1:2*n
         prop_sigma(:,j) = sigma(:,j) + X_kinematics(sigma(:,j), acc_k, gyro_k)*dt;
     end
     
     % 3. generate apriori of x
     X_prior = zeros(n,1);
     for j = 1:2*n
         X_prior = X_prior + prop_sigma(:,j);
     end
     X_prior = X_prior/(2*n);

     % 4. generate apriori of P
     P_prior = zeros(n,n);
     for j = 1:2*n
         P_prior = P_prior + (prop_sigma(:,j) - X_prior)*...
                             (prop_sigma(:,j) - X_prior)';
     end
     P_prior = P_prior/(2*n) + Q;
     
     if propagate
         X_post_new = X_prior;
         P_post_new = P_prior;
         prop_nums = prop_nums + 1;
         
     else
         %%%%%%% Measurment update %%%%%%% (updated every 1s with gps)
         % 1. generate sigma points
         [root,p] = chol(n*P_prior);

        for j = 1:n
            sigma(:,j) = X_prior + root(j,:)';
            sigma(:,j+n) = X_prior - root(j,:)';
        end

        % 2. generate measurment to sigma points
        for j = 1:2*n
            dsigma_y = ned2gps(sigma(:,j), Y0);
            sigma_y(:,j) = Y0 + dsigma_y;
        end

        % 3. combine to generate predicted meas at time t_k
        y_hat = zeros(3,1);
         for j = 1:2*n
             y_hat = y_hat + sigma_y(:,j);
         end
         y_hat = y_hat/(2*n);

         % 4. generate measurement variance
%          R = diag([25,deg2rad(0.1)^2,deg2rad(0.1)^2]);
         Py = zeros(3,3);
         for j = 1:2*n
             Py = Py + (sigma_y(:,j) - y_hat)*...
                             (sigma_y(:,j) - y_hat)';
         end
         Py = Py/(2*n) + R; % (3*3)

         % 5. estimate cross cov between x,y
         Pxy = zeros(n,3);
         for j = 1:2*n
             Pxy = Pxy + (prop_sigma(:,j) - X_prior)*...
                             (sigma_y(:,j) - y_hat)'; % (13*1)*(1*3)
         end
         Pxy = Pxy/(2*n); % (6*3)

         % 6. Kalman Gain
         K = Pxy*pinv(Py); % (6*3)

         % 7. compute Measurement update
         X_post_new = X_prior + K*(gps_k' - y_hat);
         P_post_new = P_prior - K * Py * K';
         
         prop_nums = 0;
     end
     X_est = [X_est, X_post_new];
     dy = ned2gps(X_post_new,Y0);
     y_est = [y_est, dy+Y0];
end
toc

% 3D轨迹图 - 路径2
figure(7)
plot3(X_est(2,1:end),X_est(1,1:end),-X_est(3,1:end))
grid on
xlabel('East [m]')
ylabel('North [m]')
zlabel('Height [m]')
title('3D Trajectory - Path 2 (Non-highway)')
legend('UKF Estimated Path')

% 2D平面图 - 路径2
figure(8)
plot(X_est(2,1:end),X_est(1,1:end), 'LineWidth', 2)
grid on
xlabel('East [m]')
ylabel('North [m]')
title('2D Trajectory - Path 2 (Non-highway)')
legend('UKF Estimated Path')

% 卫星地图 - 路径2
figure(9)
geoplot(y_est(1,:),y_est(2,:),'r','LineWidth',2)
geobasemap('satellite')
title('Satellite View - Path 2 (Non-highway)')
hold on
geoplot(data_gps(:,1),data_gps(:,2),'b.','LineWidth',1)
hold off
legend('UKF Filtered','Raw GPS','Location','best')

figure(10)
geoplot(y_est(1,:),y_est(2,:),'r','LineWidth',2)
geobasemap('satellite')
title('UKF Estimated Path2 in Satellite View')
legend('UKF Filtered','Location','best')
hold on
%% 误 差
%% ---- 轨迹误差分析 ----
earth_radius = 6371000; % 地球半径 [m]

% 保证长度一致
min_len = min(size(y_est,2), size(data_gps,1));
y_est = y_est(:,1:min_len);
data_gps = data_gps(1:min_len,:);

% 经纬度、高度误差（单位：m）
lat_err_m = (y_est(1,:) - data_gps(:,1)') * (pi/180) * earth_radius;
lon_err_m = (y_est(2,:) - data_gps(:,2)') * (pi/180) * earth_radius .* cosd(data_gps(:,1)');
alt_err_m = (y_est(3,:) - data_gps(:,3)');

% 误差时间轴
t = (0:length(lat_err_m)-1) * dt;

% 绘制误差随时间变化图
figure(11);
subplot(3,1,1)
plot(t, lat_err_m, 'LineWidth', 1.5);
xlabel('Time [s]')
ylabel('Latitude Error [m]')
title('UKF Latitude Error vs GPS - Path 2')

subplot(3,1,2)
plot(t, lon_err_m, 'LineWidth', 1.5);
xlabel('Time [s]')
ylabel('Longitude Error [m]')
title('UKF Longitude Error vs GPS - Path 2')

subplot(3,1,3)
plot(t, alt_err_m, 'LineWidth', 1.5);
xlabel('Time [s]')
ylabel('Altitude Error [m]')
title('UKF Altitude Error vs GPS- Path 2')

sgtitle('UKF Trajectory Estimation Errors - Path 2')

% % 三维误差可视化（可选）
% figure(12);
% plot3(lon_err_m, lat_err_m, alt_err_m, 'r', 'LineWidth', 1.5)
% grid on
% xlabel('Longitude Error [m]')
% ylabel('Latitude Error [m]')
% zlabel('Altitude Error [m]')
% title('3D View of UKF Trajectory Errors')

%% 保存图片
exportgraphics(figure(1), 'E:\Programing\MATLAB\GNSS\figures\1.jpg', 'Resolution', 300);
exportgraphics(figure(2), 'E:\Programing\MATLAB\GNSS\figures\2.jpg', 'Resolution', 300);
exportgraphics(figure(3), 'E:\Programing\MATLAB\GNSS\figures\3.jpg', 'Resolution', 300);
exportgraphics(figure(4), 'E:\Programing\MATLAB\GNSS\figures\4.jpg', 'Resolution', 300);
exportgraphics(figure(5), 'E:\Programing\MATLAB\GNSS\figures\5.jpg', 'Resolution', 300);
% exportgraphics(figure(6), 'E:\Programing\MATLAB\GNSS\figures\6.jpg', 'Resolution', 300);
exportgraphics(figure(7), 'E:\Programing\MATLAB\GNSS\figures\7.jpg', 'Resolution', 300);
exportgraphics(figure(8), 'E:\Programing\MATLAB\GNSS\figures\8.jpg', 'Resolution', 300);
exportgraphics(figure(9), 'E:\Programing\MATLAB\GNSS\figures\9.jpg', 'Resolution', 300);
exportgraphics(figure(10), 'E:\Programing\MATLAB\GNSS\figures\10.jpg', 'Resolution', 300);
exportgraphics(figure(11), 'E:\Programing\MATLAB\GNSS\figures\11.jpg', 'Resolution', 300);
% exportgraphics(figure(12), 'E:\Programing\MATLAB\GNSS\figures\12.jpg', 'Resolution', 300);