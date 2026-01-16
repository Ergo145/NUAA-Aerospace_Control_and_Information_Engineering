function statedot = rocket1023(state,t,P)
global CQ CY S alpha m rhol g

x = state(1);
y =  state(2);
v = state(3);
theta = state(4);

Q = 1/2*rhol*v^2*S*CQ;
Y  = 1/2*rhol*v^2*S*CY;

xdot = v*cos(theta);
ydot = v*sin(theta);
vdot = -g*sin(theta)-Q/m+P*sin(alpha);
thetadot = (-g*cos(theta)+Y/m+P*sin(alpha)/m)/v;

statedot = [xdot;ydot;vdot;thetadot];
end