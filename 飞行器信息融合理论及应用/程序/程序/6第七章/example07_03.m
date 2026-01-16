clear all;
close all;

%% The parameters for CEP error
tstandard=load('tstandard_total.txt');
tstd=tstandard(35,7);
Zp = [0,0.12538,0.25293,0.38488,0.52400,0.67419,0.84146,...
    0.87776,0.91526,0.95410,0.99442,1.03643,1.08035,...
    1.12646,1.17509,1.22667,1.28173,1.34097,1.40532,...
    1.47608,1.55510,1.64521,1.75168,1.88121,2.05419,2.32679];
Zpp = [Zp(1),Zp(22)];

%% Discreted the time-invariant system
deltat = 1;
q2 = 0.01;q1 = 0.1;
F = [0 1;0 0];
G = zeros(2,2);
G(1,1) = q1; G(2,2) = q2;
W = zeros(2,2);
W(2,2) = 1; W(1,1) = 1;
A = zeros(4,4);
A(1:2,1:2) = -1*F;
A(1:2,3:4) = G*W*G';
A(3:4,3:4) = F';
A = A*deltat;
B = expm(A);
PHI = B(3:4,3:4);
PHI = PHI';
Q = PHI * B(1:2,3:4);
H = [0 1];%[1 0 0;0 1 0];

%% The constants for measurement error
%sigmax = 10;
sigmav = 10;
%Rx = sigmax^2;
Rv = sigmav^2;
R = Rv;%R = diag([Rx Rv]);
N = chol(R);
N = N';
N = inv(N);

%% initialization for KF
span = 1024;
I = eye(2);
seps = 1e-6;   % threshold for validating the singular output
iterative_number = 102;  % simulation with Monte Carlo method
cx = [];cxs = [];
cp = [];cps = [];
cxy = [];

time_p = []; time_s = []; 
%% KF
h = waitbar(0,' Kalman Filtering ');
for j = 1:iterative_number
    %% Generate the real trajectory
    xtrue = zeros(span,2);
    randn('state', sum(100*clock));
    qk1 = sqrt(Q(1,1))*randn(span,1);
    randn('state', sum(100*clock));
    qk2 = sqrt(Q(2,2))*randn(span,1);
    randn('state', sum(100*clock));
    xtrue(1,1) = qk1(span);
    xtrue(1,2) = 1.0+qk2(span);
    
    for i = 2:span
        xtrue(i,1) = xtrue(i-1,1)    +deltat*xtrue(i-1,2)+qk1(i-1);
        xtrue(i,2) =                         xtrue(i-1,2)+qk2(i-1);
    end
    
    %% Generate the measurement output
    z = zeros(span,1);%2);
    randn('state', sum(100*clock));
%     noisex = sigmax*randn(span,1);
    noisev = sigmav*randn(span,1);
    z = xtrue(:,2) + noisev;
%     z(:,1) = xtrue(:,1) + noisex;
%     z(:,2) = xtrue(:,2) + noisev;
%     z = z';

      
    %% Initialization of KF
    xi_pre(1) = 0; xi_pre(2) = 1.0; % initial state prediction
    Pi_pre = zeros(2);
    Pi_pre(1,1)=(xi_pre(1)-xtrue(1,1))^2;
    Pi_pre(2,2)=(xi_pre(2)-xtrue(1,2))^2;
       
    %% Standard KF
    x_pre = xi_pre';
    P_pre = Pi_pre;
    x = [];xy = [];
    p = [];
    K = P_pre*H'*inv(H*P_pre*H'+R);
    x_est = x_pre+K*(z(1)-H*x_pre);
    P = (I-K*H)*P_pre*(I-K*H)'+K*R*K';
    x = [x;x_est(1)-xtrue(1,1)];
    p = [p;P(1,1)];
    xy = [xy;x_est(2)-xtrue(1,2)];
    tic
    for i = 2:span
        x_pre = PHI*x_est;
        P_pre = PHI*P*PHI' + Q;
        K = P_pre*H'*inv(H*P_pre*H' + R);   % form the Kalman gain matrix
        x_est = x_pre + K*(z(i) - H*x_pre);  % form the Kalman estimate
        P = (I - K*H)*P_pre*(I - K*H)'+K*R*K';   % form the Kalman estimation error covariance matrix
        x = [x;x_est(1)-xtrue(i,1)];
        p = [p;P(1,1)];
        xy = [xy;x_est(2)-xtrue(i,2)];
    end
    tt = toc;
    time_p =[time_p;tt];
    cp = [cp,p];
    cx = [cx,x];
    cxy = [cxy,xy]; 
    
    waitbar(j/iterative_number,h)
end
close(h)

%% Calculate CEP
cx_mean = mean(cx,2); cx_std = std(cx,1,2);
cp_mean = mean(cp,2);cp_std = std(cp,1,2);

CEPx = [];
CEPp = [];
h = waitbar(0,' Calculating CEP errors ');
for i=1:span-1
    %% Get rid of the singular outputs from the calculation of CEP
    cepx = [];
    cepp = [];
    for j=1:iterative_number
        if cx_std(i)<seps
            cepx = [cepx,cx(i,j)];
        else
            tao_x = (cx(i,j)-cx_mean(i))/cx_std(i);
            t_x = tao_x*sqrt((iterative_number-2)/(iterative_number-1-tao_x^2));
            if (abs(t_x)<=tstd)
                cepx = [cepx,cx(i,j)];
            end
        end
        if cp_std(i)<seps
            cepp = [cepp,cp(i,j)];
        else
            tao_p = (cp(i,j)-cp_mean(i))/cp_std(i);
            t_p = tao_p*sqrt((iterative_number-2)/(iterative_number-1-tao_p^2));
            if (abs(t_p)<=tstd)
                cepp = [cepp,cp(i,j)];
            end
        end        
    end
    
    %% Calculating CEP
    cepx_mean = mean(cepx);
    cepp_mean = mean(cepp);
    cepx_std = std(cepx);
    cepp_std = std(cepp);
    cepx_rho = cepx_std^4+2*cepx_std^2*cepx_mean^2;
    cepp_rho = cepp_std^4+2*cepp_std^2*cepp_mean^2;
    cepx_yeta = cepx_std^2+cepx_mean^2+eps;
    cepp_yeta = cepp_std^2+cepp_mean^2+eps;
    cepx_zz=2*cepx_rho/(9*cepx_yeta^2);
    cepp_zz=2*cepp_rho/(9*cepp_yeta^2);
    cepx_zstd=sqrt(cepx_zz);
    cepp_zstd=sqrt(cepp_zz);
    cepx_zmean=1-cepx_zz;
    cepp_zmean=1-cepp_zz;
    cepx_rerror=[sqrt(cepx_yeta*(cepx_zstd*Zpp(1)+cepx_zmean)^3),sqrt(cepx_yeta*(cepx_zstd*Zpp(2)+cepx_zmean)^3)];
    cepp_rerror=[sqrt(cepp_yeta*(cepp_zstd*Zpp(1)+cepp_zmean)^3),sqrt(cepp_yeta*(cepp_zstd*Zpp(2)+cepp_zmean)^3)];
    CEPx = [CEPx;cepx_rerror];
    CEPp = [CEPp;cepp_rerror];
    waitbar(i/(span-1),h)
end
close(h)    

lspan = 1:span-1;
lenspan = length(lspan);

xmean = norm(CEPx(lspan,1))/sqrt(lenspan)

figure
plot(lspan,abs(CEPx(lspan,1))),grid
xlabel('滤波时间(s)'),ylabel('位置误差CEP(50%)(m)')
figure
plot(lspan,abs(CEPx(lspan,2))),grid
xlabel('滤波时间(s)'),ylabel('位置误差CEP(95%)(m)')
figure
plot(lspan,CEPp(lspan,1)),grid
xlabel('滤波时间(s)'),ylabel('位置估计偏差协方差CEP')
ccx = [];
for i = 1:1024
    ccx(i,:) = cx(1025-i,:);
end
figure
plot(1:span,cx,1:span,ccx)
axis([0 1024 -320 300])
xlabel('滤波时间(s)'),ylabel('每次滤波位置误差(m)')