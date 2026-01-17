function [Rdot,Vdot]=stateequation(R,V)
global mu
r=norm(R);
Rdot=V;
Vdot=-mu/r^3*R;
end