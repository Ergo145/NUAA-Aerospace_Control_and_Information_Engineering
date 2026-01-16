function [d_quat_i] = quat_kinematics(omega_b,quat_i)
% quat_kinematics
% EULER_KINEMATICS Derivative of euler angles in body frame
% get body(intertial) frame angle derivatives
% QUAT_KINEMATICS 计算四元数导数
% 输入:
%   omega_b - 机体角速度 (rad/s)
%   quat_i - 当前四元数
% 输出:
%   d_quat_i - 四元数导数
    P = omega_b(1);
    Q = omega_b(2);
    R = omega_b(3);
    
    conv = [0, -P, -Q, -R;
            P, 0, R, -Q;
            Q, -R, 0, P;
            R, Q, -P, 0];
        
    d_quat_i = 0.5 * conv * quat_i;
end

