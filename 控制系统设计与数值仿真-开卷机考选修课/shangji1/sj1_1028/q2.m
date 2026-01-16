clc;clear;

%% 仿真设置
tspan=[0,20];
y0 = [0;0;0;0];

% y = [ y y' y'' y''']
[t,y] = ode45(@odefun,tspan,y0);

figure;
% subplot(2,2,1);
hold on;grid on;
plot(t,y(:,1),"b","LineWidth",2);
xlabel("t");ylabel("y");

%% 仿真函数
function d5y = odefun(t,y)
    d0y = y(1);
    d1y = y(2);
    d2y = y(3);
    d3y = y(4);

    u = 5*exp(-0.5*t)*cos(t)+6*sin(t + pi/3);
    
    du = -( 5*0.5*exp(-0.5*t)*cos(t) + 5*exp(-0.5*t)*sin(t) ) + 6*cos(t+pi/3);

    d4y = -6*d3y -18*d2y - 24*d1y - 16*d0y - du + 3*u;

    d5y = [d1y;d2y;d3y;d4y];

end