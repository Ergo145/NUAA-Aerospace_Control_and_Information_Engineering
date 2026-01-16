function output=stateequation(t,state,control)
global m1 m2 g l
%%%%%%%%%%
x=state(1);
theta=state(2);
xdot=state(3);
thetadot=state(4);
%%%%%%%%%
F=control;
%%%%%%%%%%

% xdotdot=1/(m1+m2*sin(theta)^2)*(F+m2*(g*cos(theta)+l*thetadot^2)*sin(theta));
% thetadotdot=-(F+m2*(g*cos(theta)+l*thetadot^2)*sin(theta))*cos(theta)/(l*(m1+m2*sin(theta)^2))-...
%     g*sin(theta)/l;

juzhen11=m1+m2;
juzhen12=m2*l*cos(theta);
juzhen21=m2*cos(theta);
juzhen22=m2*l;
juzhen=[juzhen11 juzhen12;
    juzhen21 juzhen22];

liezhen1=F+m2*l*thetadot^2*sin(theta);
liezhen2=-m2*g*sin(theta);
liezhen=[liezhen1;liezhen2];

dotdot=inv(juzhen)*liezhen;
xdotdot=dotdot(1);
thetadotdot=dotdot(2);
%%%%%%%%%

output=[xdot;thetadot;xdotdot;thetadotdot];








