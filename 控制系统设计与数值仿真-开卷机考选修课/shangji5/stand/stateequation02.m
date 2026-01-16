function statedot=stateequation02(t,state,u)
global m g rho S CQ vt
%%%%%%%%%%%%%
xm=state(1);%导弹飞行纵程
ym=state(2);%导弹飞行高度
vm=state(3);%导弹飞行速度
thetam=state(4);%导弹弹道倾角
xt=state(5);%地面目标纵程
%%%%%%%%%
xmdot=vm*cos(thetam);
ymdot=vm*sin(thetam);
Q=1/2*rho*vm^2*S*CQ;
vmdot=-g*sin(thetam)-Q/m;
Y=u;%导弹气动升力，在此就是制导控制输入量
thetamdot=-g/vm*cos(thetam)+Y/(m*vm);
xtdot=vt;
%%%%%%%%%
statedot=[xmdot;ymdot;vmdot;thetamdot;xtdot];

