% MAIN_MISSION_DESIGN 航天器轨道动力学实验：地球-金星-火星 (EVM) 任务设计
clear; clc; close all;

%% 1. 参数设置
mu = 1.32712440018e11; 
% 发射窗口 2023-2033 [cite: 9]
start_date = datetime(2023,1,1);
end_date = datetime(2033,1,1);

% 离散化时间步长 (天)
step_days = 30; 
launch_dates = start_date:days(step_days):end_date;

% 飞行时间离散化 (Step 1)
tof_min = 50;  
tof_max = 300; 
arrival_grid = tof_min:10:tof_max;

n_launch = length(launch_dates);
n_tof = length(arrival_grid);
C3_matrix = nan(n_tof, n_launch);

%% 2. 步骤一：地球 -> 金星 Pork-Chop 图
fprintf('正在计算地球-金星转移 Pork-Chop 图...\n');

for i = 1:n_launch
    t_dep = launch_dates(i);
    % 修改：直接传递 datetime 对象
    [r_e, v_e] = get_planet_state(3, t_dep); 
    
    for j = 1:n_tof
        dt_days = arrival_grid(j);
        t_arr = t_dep + days(dt_days);
        
        [r_v, v_v] = get_planet_state(2, t_arr); % 金星
        
        try
            dt_sec = dt_days * 86400;
            [v_dep_sc, v_arr_sc] = solve_lambert(r_e, r_v, dt_sec);
            
            % 计算特征能量 C3
            v_inf_vec = v_dep_sc - v_e;
            C3 = norm(v_inf_vec)^2;
            
            if C3 < 150 % 稍微放宽阈值以便绘图
                C3_matrix(j, i) = C3;
            end
        catch
            % 忽略兰伯特求解失败的点
        end
    end
end

% 绘制 Pork-Chop 图
figure(1);
[X, Y] = meshgrid(datenum(launch_dates), arrival_grid);
contourf(X, Y, C3_matrix, 20);
datetick('x', 'yyyy-mm');
xlabel('地球发射日期');
ylabel('飞行时间 (天)');
title('地球->金星 C3 能量等高线图 (km^2/s^2)');
colorbar;
grid on;
drawnow;

%% 3. 步骤三：地球 -> 金星 -> 火星 (EVM) 借力搜索
% 约束: 总时间 <= 2年 (730天) [cite: 13]
fprintf('正在搜索 EVM 借力轨道 (这可能需要几分钟)...\n');

best_mission.cost = inf;
tof2_grid = 150:20:500; % 金星->火星 飞行时间范围

for i = 1:n_launch
    t1 = launch_dates(i);
    [r1, v1_p] = get_planet_state(3, t1); % 地球
    
    for j = 1:n_tof
        dt1 = arrival_grid(j);
        t2 = t1 + days(dt1);
        [r2, v2_p] = get_planet_state(2, t2); % 金星
        
        % 第一段 Lambert
        try
            [v1_dep, v2_arr] = solve_lambert(r1, r2, dt1 * 86400);
        catch, continue; end
        
        % 金星处到达相对速度 V_inf_in
        v_inf_in = v2_arr - v2_p;
        
        % 搜索第二段
        for k = 1:length(tof2_grid)
            dt2 = tof2_grid(k);
            
            if (dt1 + dt2) > 730 % 总时间约束 [cite: 13]
                continue; 
            end
            
            t3 = t2 + days(dt2);
            [r3, v3_p] = get_planet_state(4, t3); % 火星
            
            % 第二段 Lambert
            try
                [v2_dep, v3_arr] = solve_lambert(r2, r3, dt2 * 86400);
            catch, continue; end
            
            % 金星处出发相对速度 V_inf_out
            v_inf_out = v2_dep - v2_p;
            
            % 借力评估 (速度匹配误差 + 发射DeltaV)
            % 理想借力: |V_inf_in| = |V_inf_out|
            dv_flyby_mismatch = abs(norm(v_inf_in) - norm(v_inf_out));
            launch_dv = norm(v1_dep - v1_p);
            
            % 代价函数 (加权和，优先寻找 mismatch 小的)
            current_cost = launch_dv + dv_flyby_mismatch;
            
            if current_cost < best_mission.cost
                best_mission.cost = current_cost;
                best_mission.dates = [t1, t2, t3];
                best_mission.tofs = [dt1, dt2];
                best_mission.v_inf = [norm(v_inf_in), norm(v_inf_out)];
                best_mission.dv_launch = launch_dv;
                best_mission.dv_flyby = dv_flyby_mismatch;
            end
        end
    end
end

%% 4. 结果输出
fprintf('\n=== 最优任务设计结果 ===\n');
if isinf(best_mission.cost)
    fprintf('未找到满足约束的解。\n');
else
    fprintf('发射日期 (地球): %s\n', datestr(best_mission.dates(1)));
    fprintf('借力日期 (金星): %s\n', datestr(best_mission.dates(2)));
    fprintf('到达日期 (火星): %s\n', datestr(best_mission.dates(3)));
    fprintf('E->V 飞行时间: %d 天\n', best_mission.tofs(1));
    fprintf('V->M 飞行时间: %d 天\n', best_mission.tofs(2));
    fprintf('总飞行时间: %d 天 (要求 <= 730)\n', sum(best_mission.tofs));
    fprintf('金星 V_inf (入): %.2f km/s, (出): %.2f km/s\n', ...
        best_mission.v_inf(1), best_mission.v_inf(2));
    fprintf('速度差 (需机动): %.2f km/s\n', best_mission.dv_flyby);
end