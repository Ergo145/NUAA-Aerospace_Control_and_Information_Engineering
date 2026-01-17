function E = M_to_E(M, e)
    % 将平近点角 M 转换为偏近点角 E (牛顿法求解)
    M = deg2rad(M);
    % 初始猜测 (适用于所有 e)
    if M < pi
        E = M + e;
    else
        E = M - e;
    end
    
    % 牛顿法迭代
    tolerance = 1e-8;
    for k = 1:20 
        E_new = E - (E - e * sin(E) - M) / (1 - e * cos(E));
        if abs(E_new - E) < tolerance
            break;
        end
        E = E_new;
    end
end
