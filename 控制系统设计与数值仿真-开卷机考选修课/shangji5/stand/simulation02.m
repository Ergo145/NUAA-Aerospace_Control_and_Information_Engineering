close all,clear,clc,
tic,
%%%%%%%%%%%%%
global m g rho S CQ vt
m=500;%导弹质量
g=9.8;%重力加速度
rho=1.225;%大气密度
S=0.01;%导弹气动特征面积
CQ=0.1;%导弹气动阻力系数
vt=-72*1000/(60*60);%目标移动速度，取值为负表示目标迎面开来
vt=-100*1000/(60*60);
%%%%%%%%%%
xm_initial=0;%导弹纵程初值
ym_initial=5000;%导弹飞行高度初值
vm_initial=500;%导弹飞行速度初值
thetam_initial=0*pi/180;%导弹弹道倾角初值
thetam_initial=30*pi/180;%弹道倾角初始值为30度（以上两行都行，看具体题目要求）

xm=xm_initial;
ym=ym_initial;
vm=vm_initial;
thetam=thetam_initial;

xt_initial=6000;%目标纵程初值
xt=xt_initial;

state=[xm;ym;vm;thetam;xt];
stateout=state;
yt=0;%目标轨迹高度（地面目标）
ytout=yt;%目标轨迹高度序列
%%%%%%%%%%%%
q=-atan(ym/(xt-xm));%弹目视线角
r=sqrt((xt-xm)^2+ym^2);%弹目距离
qdot=vm/r*sin(q-thetam)-vt/r*sin(q);%弹目视线角速度
qout=q;%弹目视线角序列
qdotout=qdot;%弹目视线角速度序列
rout=r;%弹幕距离序列
uout=[];
%%%%%%%%%%
t=0;%仿真起始时间
dt=0.0005;%龙格库塔递推步长
tout=t;%仿真时间序列
%%%%%%%%%%
k=10;%比例制导系数
%%%%%%%%%%%%
while ym>0
    u=k*m*vm*qdot+m*g*cos(thetam);%作为比例制导控制输入的导弹气动升力
    if abs(u/m)>20*g%导弹过载限制
        u=sign(u)*20*m*g;
    end
    uout=[uout,u];
    %%%%%%%%%%%%%
    ke1=stateequation02(t,state,u);
    ke2=stateequation02(t+0.5*dt,state+0.5*ke1*dt,u);
    ke3=stateequation02(t+0.5*dt,state+0.5*ke2*dt,u);
    ke4=stateequation02(t+dt,state+ke3*dt,u);
    state=state+1/6*(ke1+2*ke2+2*ke3+ke4)*dt;
    stateout=[stateout,state];
    %%%%%%%%%%%%%%
    xm=state(1);
    ym=state(2);
    vm=state(3);
    thetam=state(4);
    xt=state(5);
    %%%%%%%%%%%
    q=-atan(ym/(xt-xm));%弹目视线角
    r=sqrt((xt-xm)^2+ym^2);%弹目距离
    qdot=vm/r*sin(q-thetam)-vt/r*sin(q);%弹目视线角速度
    qout=[qout,q];
    rout=[rout,r];
    qdotout=[qdotout,qdot];
    %%%%%%%%%5
    t=t+dt;
    tout=[tout,t];
    %%%%%%%%
    yt=0;
    ytout=[ytout,yt];
end
%%%%%%%%%%%%%%%%%
xmout=stateout(1,:);
ymout=stateout(2,:);
vmout=stateout(3,:);
thetamout=stateout(4,:);
xtout=stateout(5,:);
uout=[uout,uout(end)];
%%%%%%%%%%%%%
figure,
subplot(3,1,1),
plot(tout,vmout,'linewidth',2),
ylabel('vm(m/s)'),
grid on,
%%%%%%%%%%
subplot(3,1,2),
plot(tout,thetamout*180/pi,'linewidth',2),
ylabel('themam(deg)'),
grid on,
%%%%%%%%%%%%
subplot(3,1,3),
plot(tout,uout,'linewidth',2),
ylabel('u(N)'),
grid on,
xlabel('t(s)'),
%%%%%%%%%%%%
figure,
subplot(3,1,1),
plot(tout,qout*180/pi,'linewidth',2),
ylabel('q(deg)'),
grid on,
%%%%%%%%%%%%%
subplot(3,1,2),
plot(tout,qdotout*180/pi,'linewidth',2),
ylabel('qdot(deg/s)'),
grid on,
%%%%%%%%%%%
subplot(3,1,3),
plot(tout,rout,'linewidth',2),
ylabel('r(m)'),
xlabel('t(s)'),
grid on,
%%%%%%%%%%%%
if r<=1
    disp('脱靶量(m)'),
    r,
    disp('脱靶量小于误差范围，导弹命中目标，制导飞行试验圆满成功!'),
end
%%%%%%%%%%%%

figure,
plot(xmout,ymout,'r','linewidth',2),
hold on,
plot(xtout,ytout,'b','linewidth',2)
hold on,
plot(xm,ym,'*r','markersize',20),
set(gca,'fontname','microsoft yahei'),
legend('导弹弹道','目标轨迹'),
grid on,
%%%%%%%%%%%%%%%%

%动态显示导弹弹道和地面目标轨迹（考试时不要求）
figure,
n=length(xmout);
for k=1:100:n%每100个时刻的图画一帧，这样动画显示快。
    xm=xmout(k);
    ym=ymout(k);
    
    xt=xtout(k);
    yt=ytout(k);
    
    axis([xm_initial,xt_initial,0,ym_initial+1000]),%xm_initial和xt_initial分别是xm和xt的初值，ym_initial是hm的初值
    plot(xm,ym,'r.'),
    hold on,
    plot(xt,yt,'b.'),
    pause(0.01),
    grid on,
end
hold on,
plot(xm,ym,'*r','markersize',20),
set(gca,'fontname','microsoft yahei'),
title('导弹弹道和地面目标轨迹'),
xlabel('飞行纵程(m)'),
ylabel('飞行高度(m)'),
%%%%%%%%%%%%%%%%%%%%
toc,
















