function ParticleEx1
% Particle filter example, adapted from Gordon, Salmond, and Smith paper.

x = 0.1; % initial state
Q = 1; % process noise covariance
R = 1; % measurement noise covariance
tf = 50; % simulation length

N = 100; % number of particles in the particle filter

xhat = x;
P = 2;
xhatPart = x;
xhatU = x;

% Initialize the particle filter.
for i = 1 : N
    xpart(i) = x + sqrt(P) * randn;
end

xArr = [x];
yArr = [x^2 / 20 + sqrt(R) * randn];
xhatArr = [x];
PArr = [P];
xhatPartArr = [xhatPart];
xhatUArr = [xhatU];

%parameter setting for UKF
alpha=0.01;
beta=2;
L=1;%状态维数
kappa=3-L;
lamda=alpha^2*(L+kappa)-L;
gama=sqrt(L+lamda);
% sigma点权重值设置(SUT,如需变化在此改动采样策略),这里生成的权值向量为行向量
Wm(1)=lamda/(L+lamda);
Wc(1)=lamda/(L+lamda)+1-alpha^2+beta;
for i=2:(2*L+1),
    Wm(i)=1/(2*(L+lamda));
    Wc(i)=1/(2*(L+lamda));
end

PUest = P;
PUrest = P;

close all;

for k = 1 : tf
    % System simulation
    x = 0.5 * x + 25 * x / (1 + x^2) + 8 * cos(1.2*(k-1)) + sqrt(Q) * randn;%状态方程
    y = x^2 / 20 + sqrt(R) * randn;%观测方程
    % Extended Kalman filter
    F = 0.5 + 25 * (1 - xhat^2) / (1 + xhat^2)^2;
    P = F * P * F' + Q;
    xhat = 0.5 * xhat + 25 * xhat / (1 + xhat^2) + 8 * cos(1.2*(k-1));%预测
    H = xhat / 10;
    K = P * H' * (H * P * H' + R)^(-1);
    xhat = xhat + K * (y - xhat^2 / 20);%更新
    P = (1 - K * H) * P;
    
    % UKF
    sqrtPk = sqrt(PUest);
    Xsigmak(1)=xhatU;
    Xsigmak(2)=Xsigmak(1)+gama*sqrtPk;
    Xsigmak(3)=Xsigmak(1)-gama*sqrtPk;
    Xsigmakf=zeros(1,3);
    for i=1:3
        Xsigmakf(i)=0.5*Xsigmak(i)+25*Xsigmak(i)/(1+Xsigmak(i)^2)+8*cos(1.2*(k-1));
    end;
    %系统状态转移函数，根据具体情况编制，注意要求输入中Xak为L*（2L+1）的矩阵，uk为控制向量，输出为dimx*（2L+1）的矩阵
    xhatU=Xsigmakf*Wm';%均值预测
    Pkpre=Q;
    for i=1:(2*L+1),
        Pkpre=Pkpre+Wc(i)*(Xsigmakf(i)-xhatU)*(Xsigmakf(i)-xhatU);
    end;
    Zsigmakh=zeros(1,3);
    for i=1:3
        Zsigmakh(i)=Xsigmakf(i)^2/20;
    end;
    zkpre=Zsigmakh*Wm';%量测预测
    Pzzk=R;
    Pxzk=0;
    for i=1:(2*L+1),
        Pzzk=Pzzk+Wc(i)*(Zsigmakh(i)-zkpre)*(Zsigmakh(i)-zkpre);
        Pxzk=Pxzk+Wc(i)*(Xsigmakf(i)-xhatU)*(Zsigmakh(i)-zkpre);
    end
    % 量测更新
    vnewk=y-zkpre;
    K=Pxzk/Pzzk;
    xhatU=xhatU+K*vnewk;
    PUest=Pkpre-K*Pzzk*K';

    % Particle filter
    for i = 1 : N
        xpartminus(i) = 0.5 * xpart(i) + 25 * xpart(i) / (1 + xpart(i)^2) + 8 * cos(1.2*(k-1)) + sqrt(Q) * randn;
        ypart = xpartminus(i)^2 / 20;
        vhat = y - ypart;%观测和预测的差
        q(i) = (1 / sqrt(R) / sqrt(2*pi)) * exp(-vhat^2 / 2 / R);
    end
    % Normalize the likelihood of each a priori estimate.
    qsum = sum(q);
    for i = 1 : N
        q(i) = q(i) / qsum;%归一化权重
    end
    % Resample.
    for i = 1 : N
        u = rand; % uniform random number between 0 and 1
        qtempsum = 0;
        for j = 1 : N
            qtempsum = qtempsum + q(j);
            if qtempsum >= u
                xpart(i) = xpartminus(j);
                break;
            end
        end
    end
    % The particle filter estimate is the mean of the particles.
    xhatPart = mean(xpart);
    % Plot the estimated pdf's at a specific time.
    if k == 20
        % Particle filter pdf
        pdf = zeros(81,1);
        for m = -40 : 40
            for i = 1 : N
                if (m <= xpart(i)) && (xpart(i) < m+1)
                    pdf(m+41) = pdf(m+41) + 1;
                end
            end
        end
        figure;
        m = -40 : 40;
        plot(m, pdf / N, 'r');
        hold;
        disp(['min, max xpart(i) at k = 20: ', num2str(min(xpart)), ', ', num2str(max(xpart))]);
        % Kalman filter pdf
        pdf = (1 / sqrt(P) / sqrt(2*pi)) .* exp(-(m - xhat).^2 / 2 / P);
        plot(m, pdf, 'b');
        legend('PF', 'EKF');xlabel('x'),ylabel('概率分布密度函数');
    end
    % Save data in arrays for later plotting
    xArr = [xArr x];
    yArr = [yArr y];
    xhatArr = [xhatArr xhat];
    PArr = [PArr P];
    xhatPartArr = [xhatPartArr xhatPart];
    xhatUArr = [xhatUArr xhatU];
end

t = 0 : tf;

figure;
plot(t, xArr,'k.',t,xhatPartArr,'o-');
xlabel('采样点'); ylabel('状态值');
legend('真实状态', 'PF估计状态'); 

figure;
plot(t, xArr, 'b.', t, xhatArr, 'k-', t, xhatArr-2*sqrt(PArr), 'r:', t, xhatArr+2*sqrt(PArr), 'r:');
axis([0 tf -40 40]);
set(gca,'FontSize',12); set(gcf,'Color','White'); 
xlabel('采样点'); ylabel('状态值');
legend('真实状态', 'EKF估计值', '95%置信区间'); 

figure;
plot(t, xArr, 'b.', t, xhatPartArr, 'ko-',t,xhatUArr,'ks-',t,xhatArr,'r*-');
set(gca,'FontSize',12); set(gcf,'Color','White'); 
xlabel('采样点'); ylabel('状态值');
legend('真实状态', 'PF估计值','UKF估计值','EKF估计值'); 

xhatRMS = sqrt((norm(xArr - xhatArr))^2 / tf);
xhatPartRMS = sqrt((norm(xArr - xhatPartArr))^2 / tf);
disp(['Kalman filter RMS error = ', num2str(xhatRMS)]);
disp(['Particle filter RMS error = ', num2str(xhatPartRMS)]);
