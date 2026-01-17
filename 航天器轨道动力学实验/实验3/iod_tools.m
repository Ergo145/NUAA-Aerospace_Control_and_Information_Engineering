classdef iod_tools
    % IOD_TOOLS 包含初始轨道确定实验所需的数学工具函数
    
    methods(Static)
        
        function R = calc_site_position(phi, H, lst, Re, f)
            % 根据纬度、高度、恒星时计算测站位置矢量 (ECI)
            e2 = 2*f - f^2;
            sin_phi = sin(phi);
            N = Re / sqrt(1 - e2 * sin_phi^2);
            
            x = (N + H) * cos(phi) * cos(lst);
            y = (N + H) * cos(phi) * sin(lst);
            z = (N * (1 - e2) + H) * sin_phi;
            R = [x; y; z];
        end

        function [f, g] = universal_fg_exact(r_vec, v_vec, dt, mu)
            % 利用通用变量法求精确的 Lagrange 系数 f 和 g
            r = norm(r_vec);
            v = norm(v_vec);
            vr = dot(r_vec, v_vec) / r;
            alpha = 2/r - v^2/mu;
            
            % 【已修正】：采用更鲁棒的初始猜测
            chi = sqrt(mu) * dt / r;
            
            % Newton 迭代
            ratio = 1;
            max_it = 100;
            it = 0;
            while abs(ratio) > 1e-8 && it < max_it
                z = alpha * chi^2;
                [S, C] = iod_tools.stumpff(z);
                
                r_val = (chi^2)*C + r*(1 - alpha*chi^2*S) + vr*chi*(1 - z*S);
                t_val = (chi^3*S + r*chi*(1 - alpha*chi^2*S) + vr*chi^2*C) / sqrt(mu);
                
                ratio = (t_val - dt) / r_val;
                chi = chi - ratio;
                it = it + 1;
            end
            
            z = alpha * chi^2;
            [S, C] = iod_tools.stumpff(z);
            
            f = 1 - (chi^2 / r) * C;
            g = dt - (chi^3 / sqrt(mu)) * S;
        end

        function [S, C] = stumpff(z)
            % Stumpff 函数实现
            if z > 1e-6
                S = (sqrt(z) - sin(sqrt(z))) / (sqrt(z))^3;
                C = (1 - cos(sqrt(z))) / z;
            elseif z < -1e-6
                S = (sinh(sqrt(-z)) - sqrt(-z)) / (sqrt(-z))^3;
                C = (cosh(sqrt(-z)) - 1) / (-z);
            else
                S = 1/6; C = 1/2;
            end
        end

        function coe = rv2coe(r_vec, v_vec, mu)
            % 状态矢量转轨道根数
            r = norm(r_vec); v = norm(v_vec);
            h_vec = cross(r_vec, v_vec); h = norm(h_vec);
            n_vec = cross([0;0;1], h_vec); n = norm(n_vec);
            e_vec = (1/mu) * ((v^2 - mu/r)*r_vec - dot(r_vec, v_vec)*v_vec);
            e = norm(e_vec);
            xi = v^2/2 - mu/r; a = -mu / (2*xi);
            inc = acos(h_vec(3)/h) * 180/pi;
            
            if n~=0, Omega = acos(n_vec(1)/n)*180/pi; if n_vec(2)<0, Omega=360-Omega; end; else, Omega=0; end
            if n~=0 && e~=0, w = acos(dot(n_vec,e_vec)/(n*e))*180/pi; if e_vec(3)<0, w=360-w; end; else, w=0; end
            if e~=0, nu = acos(dot(e_vec,r_vec)/(e*r))*180/pi; if dot(r_vec,v_vec)<0, nu=360-nu; end; else, nu=0; end
            
            coe = [a, e, inc, Omega, w, nu];
        end
    end
end