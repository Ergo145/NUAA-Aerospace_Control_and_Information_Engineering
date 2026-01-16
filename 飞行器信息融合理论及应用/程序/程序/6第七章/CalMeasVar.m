function R = CalMeasVar(C_N0, T)
% 功能描述：根据C_N0计算量测噪声协方差
% 输出参数：R，rad^2, 量测噪声方差
% 输入参数：C_N0，dB-Hz
%          T, s相干积分时间
% 作    者：Jr9910
% 版    本：V1.0
c_n0 = 10^(C_N0 / 10);               % Unit: Hz
R = 1/(2 * c_n0 * T) * (1 + 1/(2 * c_n0 * T));    % Unit: rad^2
