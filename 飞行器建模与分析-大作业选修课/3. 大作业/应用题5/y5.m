clc;clear;
m = 100;
g = 9.8;
a1 = 1;
a2 = 1;
for k= 1:0.01:100        
    simOut = sim('y5s');  % 运行y5s
    y = simOut.y;         % 读取 To Workspace 输出的结构体
      % 提取数值
    if min(y)>0
        break;
    end
end
disp(['最小安全k为：',num2str(k)]);