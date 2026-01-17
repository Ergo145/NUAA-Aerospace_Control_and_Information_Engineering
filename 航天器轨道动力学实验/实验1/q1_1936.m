clear; clc; close all;

% --- 全局设置 ---
nu_span = 0:1:360; % 绘制完整轨迹所需的真近点角序列
Re = 6371;         % 地球半径 km
mu = 398600.4418;  % 地球引力常数

% 基准轨道参数 [a, e, i, Omega, omega, theta] (角度为度 deg)
base_coe_leo = [7000, 0.1, 45, 30, 30, 0];
% 针对 e=0.8 情况，使用一个大 a (34000km) 确保近地点在地球外部
base_coe_geo = [34000, 0.1, 45, 30, 30, 0]; 

% 定义要分析的六个参数及其测试值
param_list = {'a', 'e', 'i', 'Omega', 'omega', 'theta'};

param_settings = containers.Map('KeyType', 'char', 'ValueType', 'any');
param_settings('a') = {1, [7000, 10000, 15000, 42164], '半长轴 a', base_coe_leo, 'full'};
param_settings('e') = {2, [0, 0.2, 0.5, 0.8], '偏心率 e', base_coe_geo, 'full'}; 
param_settings('i') = {3, [0, 45, 90, 135], '轨道倾角 i', base_coe_leo, 'full'};
param_settings('Omega') = {4, [0, 45, 90, 180], '升交点赤经 \Omega', base_coe_leo, 'full'};
param_settings('omega') = {5, [0, 90, 180, 270], '近地点幅角 \omega', base_coe_leo, 'full'};
param_settings('theta') = {6, [0, 90, 180, 270], '真近点角 \theta', base_coe_leo, 'point'};


% --- 循环绘制所有参数变化的图 ---
for p_name_cell = param_list
    variable_param = p_name_cell{1};
    
    p_data = param_settings(variable_param);
    param_idx = p_data{1};
    test_values = p_data{2};
    param_name = p_data{3};
    base_coe = p_data{4};
    plot_mode = p_data{5};

    
    figure('Name', ['Parameter Analysis: ' param_name], 'Color', 'w');
    colors = lines(length(test_values));
    
    % --- 子图1：三维轨迹 ---
    subplot(1,2,1); hold on; grid on; axis equal;
    
    % 绘制地球 (HandleVisibility off)
    [X,Y,Z] = sphere(20);
    surf(X*Re, Y*Re, Z*Re, 'FaceColor', 'blue', 'EdgeColor', 'none', ...
        'FaceAlpha', 0.3, 'HandleVisibility', 'off'); 

    line_handles = gobjects(1, length(test_values)); 

    % --- 基准参数准备 (用于 'point' 模式) ---
    a_base = base_coe(1); e_base = base_coe(2); 
    p_base = a_base * (1 - e_base^2); h_base = sqrt(mu * p_base);
    Omega_base = deg2rad(base_coe(4)); i_base = deg2rad(base_coe(3)); omega_base = deg2rad(base_coe(5));
    
    
    if strcmp(plot_mode, 'full') % 模式：绘制多条完整的轨迹
        
        for k = 1:length(test_values)
            current_val = test_values(k);
            current_coe = base_coe;
            current_coe(param_idx) = current_val;
            
            % 计算 h
            a_cur = current_coe(1);
            e_cur = current_coe(2);
            p_cur = a_cur * (1 - e_cur^2);
            h_cur = sqrt(mu * p_cur);
            
            % 计算一个周期的轨迹点
            r_history = zeros(3, length(nu_span));
            for j = 1:length(nu_span)
                % 构造新格式 coe_h: [h, e, RA(rad), incl(rad), w(rad), TA(rad)]
                coe_h = [h_cur, e_cur, deg2rad(current_coe(4)), deg2rad(current_coe(3)), deg2rad(current_coe(5)), deg2rad(nu_span(j))];
                
                [r_vec, ~] = sv_from_coe(coe_h);
                r_history(:, j) = r_vec'; % sv_from_coe 返回行向量，转置
            end
            
            % 绘制线条并保存句柄
            line_handles(k) = plot3(r_history(1,:), r_history(2,:), r_history(3,:), ...
                'LineWidth', 1.5, 'Color', colors(k,:));
        end

    elseif strcmp(plot_mode, 'point') % 模式：绘制同一条轨道上的不同位置点 (真近点角)
        
        % 1. 绘制基准完整轨道 (灰色虚线)
        r_history = zeros(3, length(nu_span));
        for j = 1:length(nu_span)
            coe_h = [h_base, e_base, Omega_base, i_base, omega_base, deg2rad(nu_span(j))];
            [r_vec, ~] = sv_from_coe(coe_h);
            r_history(:, j) = r_vec';
        end
        plot3(r_history(1,:), r_history(2,:), r_history(3,:), 'k:', 'LineWidth', 1, 'HandleVisibility', 'off');
                
        % 2. 绘制特定位置点
        for k = 1:length(test_values)
            current_nu_rad = deg2rad(test_values(k));
            
            % 构造 coe_h
            coe_h = [h_base, e_base, Omega_base, i_base, omega_base, current_nu_rad];
            
            [r_vec, v_vec] = sv_from_coe(coe_h);
            
            % 绘制位置点
            line_handles(k) = plot3(r_vec(1), r_vec(2), r_vec(3), 'o', ...
                'MarkerSize', 8, 'MarkerFaceColor', colors(k,:), 'Color', colors(k,:));
            
            % 绘制速度矢量
            scale = 1000;
            quiver3(r_vec(1), r_vec(2), r_vec(3), v_vec(1)*scale, v_vec(2)*scale, v_vec(3)*scale, 0, ...
                'Color', colors(k,:), 'LineWidth', 1.5, 'MaxHeadSize', 0.5, 'HandleVisibility', 'off');
        end
        
    end
    
    xlabel('X (km)'); ylabel('Y (km)'); zlabel('Z (km)');
    title(['三维轨迹: 变化 ' param_name]);
    
    % 生成图例标签 (i, Omega, omega, theta 需要加 "°")
    if param_idx >= 3 && param_idx <= 6
        legend_labels = string(test_values) + "°";
    else
        legend_labels = string(test_values);
    end
    
    legend(line_handles, legend_labels, 'Location', 'best');
    view(3);

    % --- 子图2：二维投影 (X-Y 平面) ---
    subplot(1,2,2); hold on; grid on; axis equal;
    
    % 绘制地球截面 (HandleVisibility off)
    theta_c = linspace(0, 2*pi, 100);
    plot(Re*cos(theta_c), Re*sin(theta_c), 'k--', 'HandleVisibility', 'off');

    if strcmp(plot_mode, 'full') 
        line_handles_2d = gobjects(1, length(test_values));
        for k = 1:length(test_values)
            current_val = test_values(k);
            current_coe = base_coe;
            current_coe(param_idx) = current_val;
            
            a_cur = current_coe(1); e_cur = current_coe(2);
            p_cur = a_cur * (1 - e_cur^2); h_cur = sqrt(mu * p_cur);

            r_history = zeros(3, length(nu_span));
            for j = 1:length(nu_span)
                coe_h = [h_cur, e_cur, deg2rad(current_coe(4)), deg2rad(current_coe(3)), deg2rad(current_coe(5)), deg2rad(nu_span(j))];
                [r_vec, ~] = sv_from_coe(coe_h);
                r_history(:, j) = r_vec';
            end
            line_handles_2d(k) = plot(r_history(1,:), r_history(2,:), 'LineWidth', 1.5, 'Color', colors(k,:));
        end
    
    elseif strcmp(plot_mode, 'point')
        
        % 1. 绘制基准完整轨道 (灰色虚线)
        plot(r_history(1,:), r_history(2,:), 'k:', 'LineWidth', 1, 'HandleVisibility', 'off');
        
        line_handles_2d = gobjects(1, length(test_values));
        
        % 2. 绘制特定位置点
        for k = 1:length(test_values)
            current_nu_rad = deg2rad(test_values(k));
            
            coe_h = [h_base, e_base, Omega_base, i_base, omega_base, current_nu_rad];
            [r_vec, ~] = sv_from_coe(coe_h);
            
            line_handles_2d(k) = plot(r_vec(1), r_vec(2), 'o', ...
                'MarkerSize', 8, 'MarkerFaceColor', colors(k,:), 'Color', colors(k,:));
        end

    end

    xlabel('X (km)'); ylabel('Y (km)');
    title(['X-Y 平面投影: 变化 ' param_name]);
    legend(line_handles_2d, legend_labels, 'Location', 'best');
end