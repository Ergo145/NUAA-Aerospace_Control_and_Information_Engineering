close all,clear,clc,
tic,
%%%%%%%%%%
global m1 m2 g l
m1=20;
m2=10;
l=5;
g=9.8;
%%%%%%%%%%%
x=10;
theta=30*pi/180;
xdot=0;
thetadot=0;
state=[x;theta;xdot;thetadot];
stateout=state;
%%%%%%%%
t0=0;
dt=0.005;
tf=100;
tout=t0:dt:tf;
%%%%%%%%%%%
controlout=[];
%%%%%%%%%
%¿ØÖÆÆ÷²ÎÊý
kp=0.1;
kd=1;
%%%%%%%%%%

for t=t0:dt:tf
    F=-m2*(g*cos(theta)+l*thetadot^2)*sin(theta)-kp*(m1+m2*sin(theta)^2)*(m1*x-m2*l*sin(theta))-...
        kd*(m1*xdot-m2*l*thetadot*cos(theta));
    %%%%%%%%%%%
    control=F;
    controlout=[controlout,control];
    %%%%%%%%%
    ke1=dt*stateequation(t,state,control);
    ke2=dt*stateequation(t+0.5*dt,state+0.5*ke1,control);
    ke3=dt*stateequation(t+0.5*dt,state+0.5*ke2,control);
    ke4=dt*stateequation(t+dt,state+ke3,control);
    state=state+1/6*(ke1+2*ke2+2*ke3+ke4);
    stateout=[stateout,state];
    %%%%%%%%%%
    x=state(1);
    theta=state(2);
    xdot=state(3);
    thetadot=state(4);
end
%%%%%%%%%%%%%

xout=stateout(1,:);
thetaout=stateout(2,:);
xdotout=stateout(3,:);
thetadotout=stateout(4,:);
%%%%%%%%%

Fout=controlout;
Fout=[Fout,Fout(end)];
%%%%%%%%

tout=[tout,tout(end)+dt];
%%%%%%%%%%%

figure,
subplot(2,2,1),
plot(tout,xout,'linewidth',2),
xlabel('t(s'),
ylabel('x(m)'),
grid on,

subplot(2,2,2),
plot(tout,thetaout*180/pi,'linewidth',2),
xlabel('t(s'),
ylabel('theta(deg)'),
grid on,

subplot(2,2,3),
plot(tout,Fout,'linewidth',2),
xlabel('t(s)'),
ylabel('F'),
grid on,
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
toc,


    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    