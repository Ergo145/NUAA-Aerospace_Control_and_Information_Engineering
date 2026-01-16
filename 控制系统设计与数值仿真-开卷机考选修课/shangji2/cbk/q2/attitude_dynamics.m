function state_dot = attitude_dynamics(t,state,M1,M2,M3,f1,f2,f3)
global j1 j2 j3

yita1 = state(1);
yita2 = state(2);
yita3 = state(3);
theta1 = state(4);
theta2 = state(5);
theta3 = state(6);
w1 = state(7);
w2 = state(8);
w3 = state(9);

yita1dot = theta1;
yita2dot = theta2;
yita3dot = theta3;
theta1dot = w1 - w2*cos(theta1)*tan(theta3) + w3*sin(theta1)*tan(theta3);
theta2dot = w2*cos(theta1)/cos(theta3) - w3*sin(theta1)/cos(theta3);
theta3dot = w2*sin(theta1) + w3*cos(theta1);
w1dot = (j2-j3)*w2*w3/j1 + (M1+f1)/j1;
w2dot = (j3-j1)*w3*w1/j2 + (M2+f2)/j2;
w3dot = (j1-j2)*w1*w2/j3 + (M3+f3)/j3;

state_dot = [yita1dot; yita2dot; yita3dot; theta1dot; theta2dot; theta3dot; w1dot; w2dot; w3dot];
end