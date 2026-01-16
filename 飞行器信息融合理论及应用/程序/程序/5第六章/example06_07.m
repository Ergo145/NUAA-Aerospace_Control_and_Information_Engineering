clear all
close all
clc

load dsarma.mat;
y = yu(:,3);
y = y-mean(y);
N = length(y);

delta_sigma = 0.1;
M1 = floor(N/2);
M2 = floor(N/(1+1/2/delta_sigma^2));

M = M2;

sigma_allan = [];
for m = 2:M
    K1 = floor((N-1)/(m-1))-1;
    somega = 0 ;
    temp1 = sum(y(1:m,1))/m;
    for i = 2:K1
        temp2 = sum(y((i-1)*(m-1)+1:(i*(m-1)+1),1))/m;
        somega = somega + (temp2-temp1)^2;
        temp1 = temp2;
    end
    sigma_allan = [sigma_allan,somega/2/K1];
end
sigma_allan2 = [];
for m2 = 2:M;
    K2 = floor(N/m2)-1;
    somega2 = 0;
    temp1 = sum(y(1:m2,1))/m2;
    for i = 2:K2
        temp2 = sum(y((i-1)*m2+1:i*m2,1))/m2;
        somega2 = somega2 + (temp2-temp1)^2;
        temp1 = temp2;
    end 
    sigma_allan2 = [sigma_allan2,somega2/2/K2];
end

sigma_allan_std = sqrt(sigma_allan);
t = (2:M)/100;
% figure(1),loglog(t,sigma_allan_std),grid
sigma_allan_std2 = sqrt(sigma_allan2);

Tt = zeros(M-1,2);
tt = t';
Tt(:,1) = 1./(tt.^2);
Tt(:,2) = 1./tt;
Tt(:,3) = ones(M-1,1);
% Tt(:,4) = tt;
% Tt(:,5) = tt.^2;
Sigma = sigma_allan2';
A = Tt\Sigma;

Qz = sqrt(A(1)/3);
Q = sqrt(A(2));
B = sqrt(A(3)*pi/(2*log(2)));
% K = sqrt(A(3)*3);
% R = sqrt(2*A(5));

sigma_allan_std_est = sqrt(A(1)./tt.^2+A(2)./tt+A(3));
% sigma_allan_std_est = sqrt(3)*Qz./tt+Q./sqrt(tt)+sqrt(2*log(2)/pi)*B;%+K/sqrt(3)*sqrt(tt)+R/sqrt(2)*tt;

figure(1),plot((1:N)*0.01,y),xlabel('T(s)'),ylabel('Gyroscope Error(\circ/s)')
figure(2),loglog(t,sigma_allan_std2,'b',t,sigma_allan_std_est,'r'),grid,xlim([0.02 2.4]) 
xlabel('T(s)'),ylabel('\sigma(T)'),legend('Original','Fitted')