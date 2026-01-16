clear all
close all

phi=[1 1 0.5;0 1 1;0 0 1];
h1 = [1 0 0;0 1 0];
h2 = [0 1 0];

phu1 = [h1;h1*phi;h1*phi*phi];
% phui1 = phu1';
[u1,s1,v1] = svd(phu1);

phu2 = [h2;h2*phi;h2*phi*phi];
% phui2 = phu2';
[u2,s2,v2]=svd(phu2);