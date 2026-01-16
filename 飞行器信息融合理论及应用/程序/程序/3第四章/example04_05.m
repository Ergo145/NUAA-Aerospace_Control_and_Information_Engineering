% Program for example 4-5

clear all;
close all;

z1=2;
z2=1;
z3=4;
h1=[1 1];
h2=[0 1];
h3=[1 2];
sigma1=1;
sigma2=0.5;
sigma3=5;

% Classic Least Squares
Z3=[z1;z2;z3];
H3=[h1;h2;h3];
R3=[1 0 0;0 0.25 0;0 0 25];
xc3_est=inv(H3'*H3)*H3'*Z3
pc3=inv(H3'*H3)*H3'*R3*H3*inv(H3'*H3)

% Weighted Least Squares
W3=inv(R3);
xw3_est=inv(H3'*W3*H3)*H3'*W3*Z3
pw3=inv(H3'*W3*H3)

% Iterative Weighted Least Squares
Z2=[z1;z2];
H2=[h1;h2];
R2=[1 0;0 0.25];
W2=inv(R2);
piw2=inv(H2'*W2*H2);
xiw2_est=piw2*H2'*W2*Z2;
w3=1/sigma3^2;
piw3=piw2-piw2*h3'*inv(1/w3+h3*piw2*h3')*h3*piw2
xiw3_est=xiw2_est+piw3*h3'*w3*(z3-h3*xiw2_est)