close all,clear,clc
global CQ CY S alpha m rhol g

m = 100;
d2r = pi/180;
x0 = 0;
y0 = 0;
v0 = 50;
theta0 = 30*d2r;
g = 9.8;
rhol = 1.225;
S = 0.01;
CQ = 1;
CY = 0.2;
alpha = 3*d2r;



dt = 0.01;

state = [x0;y0;v0;theta0];

stateout = [state];

y = 100;
t = 0;
tout = [t];
Pout = [];

while y>0
    t = t+dt;
    if t<=11 & t>1
        P = 500;


    else
        P = 0;
    end
    Pout = [Pout,P];
    ke1 = rocket1023(state,t,P);
    ke2  =rocket1023(state+0.5*dt*ke1,t+0.5*dt,P);
    ke3  =rocket1023(state+0.5*dt*ke2,t+0.5*dt,P);
    ke4  =rocket1023(state+dt*ke3,t+dt,P);
    
    state = state + 1/6*(ke1+2*ke2+2*ke3+ke4)*dt;

    stateout = [stateout,state];
    tout = [tout,t];
    y = state(2);
end
Pout = [Pout,Pout(end)];

subplot(3,2,1)
plot(tout,stateout(1,:),LineWidth=2);
ylabel('x(m)');

subplot(3,2,2)
plot(tout,stateout(2,:),LineWidth=2);
ylabel('y(m)');

subplot(3,2,3)
plot(tout,stateout(3,:),LineWidth=2);
ylabel('v(m)');

subplot(3,2,4)
plot(tout,stateout(4,:)/d2r,LineWidth=2);
ylabel('theta(degree)');

subplot(3,2,5)
plot(tout,Pout,LineWidth=2);
ylabel('P(N)');











