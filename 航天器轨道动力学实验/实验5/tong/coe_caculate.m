function result=coe_caculate(R,V)
global mu; eps=1e-10;
mu = 3.986004418e5;  % 地球引力常数 (km^3/s^2)
r=norm(R); v=norm(V); vr=dot(R,V)/r;
H=cross(R,V); h=norm(H);
%轨道倾角
incl=acos(H(3)/h);
%节点矢量
N=cross([0 0 1],H); n=norm(N);
%升交点赤经
if n~=0
    RA=acos(N(1)/n);
    if N(2)<0, RA=2*pi-RA; end
else
    RA=0;
end
%偏心率
E=(1/mu)*((v^2-mu/r)*R - r*vr*V); e=norm(E);
%近地点幅角
if n~=0
    if e>eps
        w=acos(dot(N,E)/(n*e));
        if E(3)<0, w=2*pi-w; end
    else
        w=0;
    end
else
    w=0;
end
%真近点角
if e>eps
    TA=acos(dot(E,R)/(e*r));
    if vr<0, TA=2*pi-TA; end
else
    cp=cross(N,R);
    if cp(3)>=0
        TA=acos(dot(N,R)/(n*r));
    else
        TA=2*pi-acos(dot(N,R)/(n*r));
    end
end
%半长轴
a=h^2/(mu*(1-e^2));
result=[h e RA incl w TA a];
coe=result;
%输出结果
fprintf('-------由状态向量解算出的轨道根数展示如下----------\n');
fprintf('地球引力常数 (km³/s²) = %g\n',mu);
fprintf('\n状态矢量:\n');
fprintf('位置 r (km) = [%g %g %g]\n',R(1),R(2),R(3));
fprintf('速度 v (km/s) = [%g %g %g]\n',V(1),V(2),V(3));
fprintf('角动量 (km²/s) = %g\n',coe(1));
fprintf('偏心率 = %g\n',coe(2));
fprintf('升交点赤经 (度) = %g\n',rad2deg(coe(3)));
fprintf('轨道倾角 (度) = %g\n',rad2deg(coe(4)));
fprintf('近地点幅角 (度) = %g\n',rad2deg(coe(5)));
fprintf('真近点角 (度) = %g\n',rad2deg(coe(6)));
fprintf('半长轴 (km) = %g\n',coe(7))
R_earth=6378;
r_di_list=[];
r_jiao_list=[];
v_jiao_list=[];
v_di_list=[];
%轨道参数初始化
w=rad2deg(w);
incl=rad2deg(incl);
RA=rad2deg(RA);
h=sqrt(mu*a*(1-e*e));
T = 2*pi*sqrt((h^2/mu)^3/(1-e^2)^3);  % 轨道周期
t_total = T;          % 计算一个完整轨道周期内的位置
t_step = T/3600;      
for t = 0:t_step:t_total
M_e = (2*pi / T) * t;  
M_e = rem(M_e, 2*pi);  
E = solve_kepler_eq(M_e, e);  
theta_rad = 2 * atan( sqrt((1+e)/(1-e)) * tan(E/2) );  % 弧度制
theta_rad = rem(theta_rad, 2*pi);  
%计算近焦点坐标系中的位置，速度矢量
r_jiao = h*h/mu/(1 + e*cos(theta_rad)) * [cos(theta_rad); sin(theta_rad); 0];  % 位置矢量
v_jiao = mu/h * [ -sin(theta_rad); e + cos(theta_rad); 0 ];  % 速度矢量
%计算两个坐标系相互转换的正交变换矩阵
Q_di_jiao=change_di_to_jiao(RA,incl,w);
Q_jiao_di=change_jiao_to_di(RA,incl,w);
%计算地心赤道坐标系中的位置矢量和速度矢量
r_di=Q_jiao_di*r_jiao;
v_di=Q_jiao_di*v_jiao;
r_jiao_list=[r_jiao_list r_jiao];
v_jiao_list=[v_jiao_list v_jiao];
r_di_list=[r_di_list r_di];
v_di_list=[v_di_list v_di];
end
% 绘制地球（简化为球体）
[X, Y, Z] = sphere(50);
R_earth = 6378137/1000;  % 地球赤道半径 (m)
surf(X*R_earth, Y*R_earth, Z*R_earth, 'FaceColor', [0.2, 0.6, 1], 'EdgeColor', 'none', 'FaceAlpha', 0.3);
axis equal;
% （FaceColor：地球蓝色；FaceAlpha：半透明；EdgeColor：隐藏网格线）
hold on;
grid on;
% 2. 绘制航天器轨道
plot3(r_di_list(1,:), r_di_list(2,:), r_di_list(3,:), 'LineWidth',1.5, 'Color','r');
% 3. 标注与美化
xlabel('X 方向 (km)','FontSize',10);
ylabel('Y 方向 (km)','FontSize',10);
zlabel('Z 方向 (km)','FontSize',10);
title('卫星轨道三维轨迹（状态向量解算轨道根数）','FontSize',12);
view(30, 30);  % 调整视角（方位角30°，仰角30°）
hold off;

function Q_di_jiao=change_di_to_jiao(RA,i,w)
RA=deg2rad(RA);
i=deg2rad(i);
w=deg2rad(w);
Q_di_jiao=[cos(RA)*cos(w)-sin(RA)*sin(w)*cos(i) sin(RA)*cos(w)+cos(RA)*cos(i)*sin(w) sin(i)*sin(w)
    -cos(RA)*sin(w)-sin(RA)*cos(i)*cos(w) -sin(RA)*sin(w)+cos(RA)*cos(i)*cos(w) sin(i)*cos(w)
    sin(RA)*sin(i) -cos(RA)*sin(i) cos(i)];
end
function Q_jiao_di=change_jiao_to_di(RA,i,w)
RA=deg2rad(RA);
i=deg2rad(i);
w=deg2rad(w);
Q_di_jiao=[cos(RA)*cos(w)-sin(RA)*sin(w)*cos(i) sin(RA)*cos(w)+cos(RA)*cos(i)*sin(w) sin(i)*sin(w)
    -cos(RA)*sin(w)-sin(RA)*cos(i)*cos(w) -sin(RA)*sin(w)+cos(RA)*cos(i)*cos(w) sin(i)*cos(w)
    sin(RA)*sin(i) -cos(RA)*sin(i) cos(i)];
Q_jiao_di=(Q_di_jiao)';
end
function E = solve_kepler_eq(M_e, e)
    tol = 1e-10;  
    max_iter = 50; 
    if e < 0.8
        E = M_e;  
    else
        E = pi;   
    end  
    % 牛顿迭代求解
    for iter = 1:max_iter
        f_E = E - e*sin(E) - M_e;       
        f_E_prime = 1 - e*cos(E);       
        delta_E = -f_E / f_E_prime;     
        E = E + delta_E;                
        if abs(delta_E) < tol
            break;
        end
    end
    E = rem(E, 2*pi);
end


end