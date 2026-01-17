clear; clc; format long g;
load('task1_result.mat'); 

% 备份任务一的初值
r2_best = r_vec(:,2);
v2_best = v2_vec;

% 初始状态
r2_curr = r2_best;
v2_curr = v2_best;
rho_old = [norm(r_vec(:,1)-R(:,1)); norm(r_vec(:,2)-R(:,2)); norm(r_vec(:,3)-R(:,3))];

max_iter = 60; % 由于不稳定，减少最大迭代次数
tol = 1e-6; 

for k = 1:max_iter
    r2_mag = norm(r2_curr);
    
    % 1. 计算精确的 f, g 
    try
        [f1, g1] = iod_tools.universal_fg_exact(r2_curr, v2_curr, tau1, mu);
        [f3, g3] = iod_tools.universal_fg_exact(r2_curr, v2_curr, tau3, mu);
    catch
        fprintf('Iter %d: 通用变量法计算失败 (数值溢出)。停止迭代。\n', k);
        break;
    end
    
    % 2. 计算 c1, c3 
    denom = f1*g3 - f3*g1;
    if abs(denom) < 1e-12
        fprintf('Iter %d: 分母过小，停止迭代。\n', k);
        break;
    end
    c1 = g3 / denom;
    c3 = -g1 / denom;

    rho1_calc = (1/D0) * (-D(1,1) + (1/c1)*D(2,1) - (c3/c1)*D(3,1));
    rho2_calc = (1/D0) * (-c1*D(1,2) + D(2,2) - c3*D(3,2));
    rho3_calc = (1/D0) * (-(c1/c3)*D(1,3) + (1/c3)*D(2,3) - D(3,3));
    
    damping = 0.1; 
    rho1_new = rho_old(1) + damping * (rho1_calc - rho_old(1));
    rho2_new = rho_old(2) + damping * (rho2_calc - rho_old(2));
    rho3_new = rho_old(3) + damping * (rho3_calc - rho_old(3));
    
    % 4. 更新 r 
    r1_next = R(:,1) + rho1_new * L(:,1);
    r2_next = R(:,2) + rho2_new * L(:,2);
    r3_next = R(:,3) + rho3_new * L(:,3);
    
    % 5. 更新 v2 
    v2_next = (1/denom) * (-f3 * r1_next + f1 * r3_next);
    
    if norm(r2_next) > 200000 || norm(v2_next) > 50
        fprintf('Iter %d: 迭代发散检测 (r=%.1f km). 这是一个共面奇异性的典型表现。\n', k, norm(r2_next));
        fprintf('>>> 停止迭代，保留上一步的最佳结果。\n');
        break;
    end
    
    % 6. 收敛检查
    rho_curr = [rho1_new; rho2_new; rho3_new];
    diff_rho = norm(rho_curr - rho_old);
    
    fprintf('Iter %2d: rho 修正量 = %.4e km, |r2| = %.4f km\n', k, diff_rho, norm(r2_next));
    
    % 接受更新
    r2_curr = r2_next;
    v2_curr = v2_next;
    rho_old = rho_curr;
    
    r2_best = r2_curr;
    v2_best = v2_curr;
    r1_n = r1_next; r2_n = r2_next; r3_n = r3_next;
    
    if diff_rho < tol
        fprintf('>>> 迭代收敛。\n');
        break;
    end
end

fprintf('\n================ 最终结果 (Task 2) ================\n');
fprintf('r2: [%.4f, %.4f, %.4f] km\n', r2_best);
fprintf('v2: [%.4f, %.4f, %.4f] km/s\n', v2_best);

% 为 Task 3 保存数据
r2_iter = r2_best;
v2_iter = v2_best;
% 如果迭代完全失败导致 r1_n 未定义，使用 Task 1 的值
if ~exist('r1_n', 'var')
    r1_n = r_vec(:,1); r2_n = r_vec(:,2); r3_n = r_vec(:,3);
end

save('task2_result.mat', 'r2_iter', 'v2_iter', 'mu', 'Re', 'R', 'r1_n', 'r2_n', 'r3_n');