%%%%%%%%%%%%惯导结算程序%%%%%%%%%%

clear all;
close all;
clc;

load dataa

%% 捷联惯导的解算

%------------常值设定---------------
global deltat
deltat = 0.01;
Wie = 0.00007292115;
g0 = 9.7803267714;
deg_rad=pi/180;   %由度转化成弧度
rad_deg=180/pi;   %由弧度转化成度
e = 0.08181919;
%--------------读入轨迹数据------------

Xe=pos_prof_L(:,1);
Ye=pos_prof_L(:,2);
Ze=pos_prof_L(:,3);   % 切平面坐标系内的位置

head=yaw*deg_rad;   %偏航角（北偏东为正）  单位:度
pitch=pitch*deg_rad;
roll=roll*deg_rad;

Wib_err(:,1) = Wib_b(:,1);   %陀螺输出（单位：弧度/秒，坐标轴的定义与比力的相同）
Wib_err(:,2) = Wib_b(:,2);
Wib_err(:,3) = Wib_b(:,3);

fib_err(:,1) = fib_b(:,1);   %比力（fx，fy，fz）   米/秒2
fib_err(:,2) = fib_b(:,2);   %指向右机翼方向为x正方向，指向机头方向为y正向，z轴与x轴和y轴构成右手坐标系 单位：米/秒2
fib_err(:,3) = fib_b(:,3);   %右前上

%———————————————初始化———————————————————
initial_llh = [116*deg_rad,40*deg_rad,0];  % 初始位置的经纬高度
lat_err(1)=initial_llh(2);
lon_err(1)=initial_llh(1);
high_err(1)=initial_llh(3);     %初始经纬高

head_err(1)=head(1);
pitch_err(1)=pitch(1);
roll_err(1)=roll(1);     %初始姿态角

%导航坐标系中的速度
V_err(1,1:3) = [0 0 0];

Cnb = weulr2dcm([pitch_err(1) roll_err(1) head_err(1)]);
Cbn=Cnb';   %初始换姿态矩阵

Cen = wllh2dcm(lat_err(1),lon_err(1));
Cne=Cen';   %初始换位置矩阵
Cep=Cen;    %地球坐标系到切平面坐标系的转换矩阵

%四元数赋初值
q=weulr2qua([pitch_err(1) roll_err(1) head_err(1)]);   %初始四元数

Number1=max(size(Wib_err));
headd(1)=0;
%解算
h = waitbar(0,' Time Loop ');
for i=1:Number1-1
    [Rm,Rn] = wradicurv(lat_err(i));   %地球的球面半径
    
    g = wgravity(lat_err(i),high_err(i));
    Wie_n = [0; Wie*cos(lat_err(i)); Wie*sin(lat_err(i));];
    wib_b=[Wib_err(i,1) ;Wib_err(i,2) ;Wib_err(i,3)];
    Wen_n=[-V_err(i,2)/(Rm+high_err(i)); V_err(i,1)/(Rn+high_err(i)); V_err(i,1)*tan(lat_err(i))/(Rn+high_err(i));];
    Win_n=Wie_n+Wen_n;
    Win_b=Cnb*Win_n;
    Wnb_b=wib_b-Win_b;  %四元数更新时用的姿态矩阵速率
    %----------------速度更新--------------------------------
    %速度更新
    V_err(i+1,1:3) = wV2V(fib_err(i,1:3),Wie_n,Wen_n,V_err(i,1:3),Cnb,g);
    %高度更新
    high_err(i+1)=high_err(i)+V_err(i,3)*deltat;
    %------------------姿态的更新----------------------
    %四元数的更新（一阶欧拉算法）
    q=wq2q(Wnb_b,q);
    %更新姿态矩阵
    Cnb = wqua2dcm(q);
    Cbn=Cnb';
    %求姿态角并判定正负号
    eulv_err = wdcm2eulr(Cnb);
    roll_err(i+1)=eulv_err(2);
    pitch_err(i+1)=eulv_err(1);
    head_err(i+1)=eulv_err(3);
    if head_err(i+1)<0
        head_err(i+1) = head_err(i+1)+2*pi;
    end
    headd(i+1)=head_err(i+1)-head(i+1);
    if headd(i+1)>pi
       headd(i+1)=headd(i+1)-2*pi;
    end
    %-----------位置更新-------------------------------
    Cen = p2p(Cen,Wen_n);
    %经度纬度的计算
    lat_err(i+1)=asin(Cen(3,3));
    lon_err(i+1)=atan2(Cen(3,2),Cen(3,1));
    if lon_err(i+1)<0
        lon_err(i+1)=lon_err(i+1)+2*pi;
    end
    
    Re_eb(i,:) = llh2ecef([lon_err(i),lat_err(i),high_err(i)],Rn,Cep);%地球坐标系中的位置
    
    x(i) = 0;y(i)=0;z(i)=0;
    if i>1
        x(i)= Re_eb(i,4)-Re_eb(1,4);
        y(i)= Re_eb(i,5)-Re_eb(1,5);
        z(i)= Re_eb(i,6)-Re_eb(1,6) ;    %切平面坐标系中的位置
    end
     waitbar(i/Number1)
end
close(h)


%误差求解
DX = Xe(1:Number1-1)'-x;
DY = Ye(1:Number1-1)'-y;
DZ = Ze(1:Number1-1)'-z;

Ved = Ve(1:Number1-1)-V_err((1:Number1-1),1);
Vnd = Vn(1:Number1-1)-V_err((1:Number1-1),2);
Vud = Vu(1:Number1-1)-V_err((1:Number1-1),3);
headd = rad_deg*(head(1:Number1-1)-head_err(1:Number1-1));
pitchd = rad_deg*(pitch(1:Number1-1)-pitch_err(1:Number1-1));
rolld = rad_deg*(roll(1:Number1-1)-roll_err(1:Number1-1));
% e=[DX' DY' DZ' Ved Vnd Vud headd' pitchd' rolld'];
t = deltat*(1:Number1-1);
figure(1)
plot3(Xe(1:Number1-1),Ye(1:Number1-1),Ze(1:Number1-1));
grid,xlabel('X'),ylabel('Y'),zlabel('Z');
figure(2)
plot(t,DX,'-',t,DY,'o-',t,DZ,'*-');
xlabel('时间（s）');ylabel('位置误差（m）');legend('东向','北向','天向');
figure(3)
plot(t,headd,'-',t,pitchd,'o-',t,rolld,'*-');
xlabel('时间（s）');ylabel('姿态误差（度）');legend('偏航','俯仰','滚转');
figure(4)
plot(t,Ved,'-',t,Vnd,'o-',t,Vud,'*-');
xlabel('时间（s）');ylabel('速度误差（m/s）');legend('东向','北向','天向');