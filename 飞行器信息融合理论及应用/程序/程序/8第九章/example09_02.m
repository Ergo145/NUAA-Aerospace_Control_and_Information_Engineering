%%%%%%%%%%%%%%%
%松组合基本程序，9状态，假设惯性器件的误差为白噪声
%Q参考惯导仿真白噪声，R参考GPS仿真白噪声
%数据输入为gps（[lat lon h ve vn vu]），delta_theta([gyro-b])，delta_V([accr-b])
%b系和s系的关系：初始姿态角theta=0*deg_rad;gama=0*deg_rad;fai=330*deg_rad;
%初始位置long=116*deg_rad;lati=40*deg_rad;high=0;%经度、纬度、高度初始值

%%%%%%%%%%%%%%%
clear all
close all
load dataa
%------------常值设定---------------
global T
T = 0.01;
deg_rad=pi/180;   %由度转化成弧度
rad_deg=180/pi;   %由弧度转化成度
%-----------GPS输出----------------
gps_lat = gps_lat*deg_rad;
gps_lon = gps_lon*deg_rad;
gps = [gps_lat gps_lon gps_h vel_gps];
%-----------惯导初始化----------------
%%载体初始参数设置
long=116*deg_rad;lati=40*deg_rad;high=0;%经度、纬度、高度初始值
vN=[0;0;0];
theta=0*deg_rad;gama=0*deg_rad;fai=330*deg_rad;
posiN=[long*rad_deg;lati*rad_deg;high];
atti=[theta;gama;fai]*rad_deg;

%%地球项
Re=6378245;  %地球半径
e=1/298.257;   %地球的椭圆率
[Rm,Rn] = wradicurv(lati);
pose0(1) = (Rn+high)*cos(lati)*cos(long);%地球坐标系初始位置
pose0(2) = (Rn+high)*cos(lati)*sin(long);
pose0(3) = (Rn*(1-e^2)+high)*sin(lati);

%重力加速度项
g0 = 9.7803267714;
g = wgravity(lati,high);
gN=[0;0;-g];%重力加速度

%地球自转角速度项
wie=7.292115147e-5;
wieN=[0;wie*cos(lati);wie*sin(lati)];%地球自转角速度（N系）
wenN=[-vN(2)/(Rm+high);vN(1)/(Rn+high);vN(1)/(Rn+high)*tan(lati)];%载体运动产生的角速度
winN=wieN+wenN;

%四元数初始值theta俯仰角gama横滚角
q = weulr2qua([theta gama fai]);

%姿态矩阵Cbn,坐标系N(地理系)-->B(机体系)
Cbn=[q(1)^2+q(2)^2-q(3)^2-q(4)^2 2*(q(2)*q(3)+q(1)*q(4)) 2*(q(2)*q(4)-q(1)*q(3));
    2*(q(2)*q(3)-q(1)*q(4)) q(1)^2-q(2)^2+q(3)^2-q(4)^2 2*(q(3)*q(4)+q(1)*q(2));
    2*(q(2)*q(4)+q(1)*q(3)) 2*(q(3)*q(4)-q(1)*q(2)) q(1)^2-q(2)^2-q(3)^2+q(4)^2];

%%指北捷联惯导力学编排
N=length(delta_theta(:,1));
Trace_Data=zeros(N,13);
I=eye(4);

%------------卡尔曼滤波初始化---------------
X = zeros(9,1);        % 状态量初值
H = zeros(6,9);        % 量测矩阵 Z=[Zr;Zv];
H(1:3,7:9) = -eye(3);H(4:6,4:6) = -eye(3);

% P初值
a = 10/Re;
Plon_pos = a^2;  Plat_pos = a^2;  Pup_pos = 10^2;           %位置误差
Peast_vel = 0.5^2;  Pnorth_vel = 0.5^2;  Pup_vel = 0.5^2;   %速度误差
Ppsi_x = 0.1^2;  Ppsi_y = 0.1^2; Ppsi_z = 0.1^2;         %失准角

P = zeros(9,9);
P(1,1) = Ppsi_x; P(2,2) = Ppsi_y; P(3,3) = Ppsi_z;
P(4,4) = Peast_vel; P(5,5) = Pnorth_vel; P(6,6) = Pup_vel;
P(7,7) = Plat_pos; P(8,8) = Plon_pos; P(9,9) = Pup_pos;
P_est(:,1) = [P(1,1),P(2,2),P(3,3),P(4,4),P(5,5),P(6,6),P(7,7),P(8,8),P(9,9)]';   %均方差估计
% R量测误差方差阵
R =diag([a^2, a^2, 10^2, 0.5^2, 0.5^2, 0.5^2]);

% Q系统误差方差阵
G = zeros(9,6);
G(1:6,1:6) = eye(6);
W = diag([(0.5*deg_rad)^2 (0.5*deg_rad)^2  (0.5*deg_rad)^2 (6.5*10^-6*g0)^2 (6.5*10^-6*g0)^2 (6.5*10^-6*g0)^2 ]) ;%W相当于Q,之后求Qd.
% W = diag([(1*deg_rad)^2 (1*deg_rad)^2  (1*deg_rad)^2 (16*10^-3*g0)^2 (16*10^-3*g0)^2 (16*10^-3*g0)^2 ]) ;%W相当于Q,之后求Qd.
%---------------------------------------------
update = 0;  count = 0;
j = 1;
%*************************进行kalman滤波******************************
w = waitbar(0,' Time Loop ');
late=lati;lone=long;highe=high;Ve=vN;%???
for i=1:N
    %      wibB(:,i) = Wib_b(i,:)';
    %      fB(:,i) = fib_b(i,:)';
    wibB(:,i) = delta_theta(i,:)';
    fB(:,i) = delta_V(i,:)';
    cW = wgenmtr(2*wieN+wenN);
    %载体运动加速度
    fN=Cbn'*fB(:,i);
    delta_vN=fN-cW*Ve+gN;
    
    %位置更新率
    delta_posiN=[Ve(1)/((Rn+highe)*cos(late));Ve(2)/(Rm+highe);Ve(3)];
    
    %载体速度、位置更新
    vN=Ve+delta_vN*T;
    long=lone+delta_posiN(1)*T;
    lati=late+delta_posiN(2)*T;
    high=highe+delta_posiN(3)*T;%高度通道发散
    
    %四元数更新(角增量法,四阶)
    wnbB=wibB(:,i)-Cbn*winN;
    delta_Q0=sqrt((wnbB(1))^2+(wnbB(2))^2+(wnbB(3))^2)*T;
    delta_Q1=[   0,     -wnbB(1),  -wnbB(2),    -wnbB(3);
        wnbB(1),     0,        wnbB(3),   -wnbB(2);
        wnbB(2),   -wnbB(3),      0,       wnbB(1);
        wnbB(3),    wnbB(2),   -wnbB(1),       0    ]*T;
    q=((1-delta_Q0^2/8+delta_Q0^4/384)*I+(1/2-delta_Q0^2/48)*delta_Q1)*q;
    q=q/norm(q);
    
    %姿态矩阵Cbn,坐标系N(地理系)-->B(机体系)
    Cbn=[q(1)^2+q(2)^2-q(3)^2-q(4)^2 2*(q(2)*q(3)+q(1)*q(4)) 2*(q(2)*q(4)-q(1)*q(3));
        2*(q(2)*q(3)-q(1)*q(4)) q(1)^2-q(2)^2+q(3)^2-q(4)^2 2*(q(3)*q(4)+q(1)*q(2));
        2*(q(2)*q(4)+q(1)*q(3)) 2*(q(3)*q(4)-q(1)*q(2)) q(1)^2-q(2)^2-q(3)^2+q(4)^2];
    
    %******************************卡尔曼滤波********************************
    CF = tranmj(wenN,wieN,fN,Rm,Rn,late,Ve,highe);%late?lati?Ve?vN?high?highe
    F(1:9,1:9) = CF;
    
    A = zeros(18,18);
    A(1:9,1:9) = -1*F;
    A(1:9,10:18) = G*W*G';
    A(10:18,10:18) = F';
    A = A*T;
    B = expm(A);    %离散化
    
    PHI_trans = B(10:18,10:18);
    PHI = PHI_trans';      %离散的状态转移矩阵
    Q = PHI*B(1:9,10:18); %量测噪声序列协方差阵
    
    P_pre = PHI*P*PHI' + Q;       %一步预测均方差
    
    %量测更新周期0.1s大于惯导解算周期0.01s
    count = count + 1;
    if count >= 10,    % 量测输出周期0.1s
        update = 1;
        count = 0;
    end
    if update == 1;
        Z(:,j)=gps(j+1,:)'-[lati;long;high;vN];
        K = P_pre*H'/(H*P_pre*H' + R);       %滤波增益
        X = K*Z(:,j);    %状态估计
        P = (eye(9) - K*H)*P_pre*(eye(9)-K*H)'+K*R*K';    %估计均方差
        update = 0;
    else
        X = zeros(9,1);
        P = P_pre;
    end
    
    %**********滤波结果对导航解算的校正***********
    if i==10*j
        %位置的校正
        late = lati-X(7);
        lone = long-X(8);
        highe = high-X(9);
        %速度校正
        Ve = vN - X(4:6);
        %姿态矩阵校正n-p
        Cpn = [    1     X(3)    -X(2);
            -X(3)    1       X(1);
            X(2)   -X(1)     1  ];
        %n系到b系
        Cbn = Cbn*Cpn;
        %校正后的四元数
        qr = sign(q(1));
        q = dcm2q(Cbn,qr);
        q=q/norm(q);
        X_est(:,j) = X;%状态估计
        P_est(:,j) = diag(P);%均方差估计
        j=j+1;
    else
        lone = long;     late = lati;     highe = high;
        Ve = vN;
    end
    
    posiN=[lone*180/pi;late*180/pi;highe];
    pose(1) = (Rn+highe)*cos(late)*cos(lone);
    pose(2) = (Rn+highe)*cos(late)*sin(lone);
    pose(3) = (Rn*(1-e^2)+highe)*sin(late);   % 轨迹原点在地球坐标系中的坐标  切平面坐标系原点p34
    DCMep = wllh2dcm(late,lone);  %地球坐标系到切平面坐标系
    posn = DCMep*(pose'-pose0');
    
    Cbn=[q(1)^2+q(2)^2-q(3)^2-q(4)^2 2*(q(2)*q(3)+q(1)*q(4)) 2*(q(2)*q(4)-q(1)*q(3));
        2*(q(2)*q(3)-q(1)*q(4)) q(1)^2-q(2)^2+q(3)^2-q(4)^2 2*(q(3)*q(4)+q(1)*q(2));
        2*(q(2)*q(4)+q(1)*q(3)) 2*(q(3)*q(4)-q(1)*q(2)) q(1)^2-q(2)^2-q(3)^2+q(4)^2];
    atte = wdcm2eulr(Cbn)*180/pi;
    
    %地球参数更新
    [Rm,Rn] = wradicurv(late);
    g = wgravity(late,highe);
    gN=[0;0;-g];
    wieN=[0;wie*cos(late);wie*sin(late)];
    wenN=[-Ve(2)/(Rm+highe);Ve(1)/(Rn+highe);Ve(1)/(Rn+highe)*tan(late)];
    winN=wieN+wenN;
    %惯导解算数据结果
    Trace_Data(i,:)=[i/100,posn',posiN',Ve',atte'];
    
    waitbar(i/N)
end
close(w)

%解算误差
ev=Trace_Data(:,8:10)-vel_prof_L(2:N+1,:);  %速度
ea=Trace_Data(:,11:13)-[pitch(2:N+1)' roll(2:N+1)' yaw(2:N+1)'];    %姿态角
ere=Trace_Data(:,5:7)-[lon(2:N+1)*180/pi lat(2:N+1)*180/pi h(2:N+1)];   %经纬高
ern=Trace_Data(:,2:4)-pos_prof_L(2:N+1,:);  %切平面xyz
for j=1:N
    if ea(j,3)>180
        ea(j,3)=ea(j,3)-360;
    elseif ea(j,3)<-180
        ea(j,3)=ea(j,3)+360;
    end
end
for i=1:N
    elat = Trace_Data(i,6);
    eh = Trace_Data(i,7);
    [Rm,Rn] = wradicurv(elat);
    tran = [0 1/(Rm+eh) 0;1/((Rn+eh)*cos(elat)) 0 0;0 0 1];
    ereb_n(i,:)=(tran\[ere(i,2)*deg_rad ere(i,1)*deg_rad ere(i,3)]')';
end

t = (1:N)*T;N1= 1:70:N;t1 = N1*T;
figure
%plot(t,ereb_n);hold on;
h2=plot(t1,ereb_n(N1,1),'k-',t1,ereb_n(N1,2),'kx-',t1,ereb_n(N1,3),'ks-')
grid on
xlabel('时间（s）'),ylabel('位置误差（m）'),legend(h2,'经度误差','纬度误差','高度误差');
figure
%plot(t,ea);hold on
h2=plot(t1,ea(N1,1),'k-',t1,ea(N1,2),'kx-',t1,ea(N1,3),'ks-')
grid on
xlabel('时间（s）'),ylabel('姿态角误差（°）'),legend(h2,'俯仰角误差','滚转角误差','偏航角误差');
figure
plot(t,ere(:,1:2))
grid on
xlabel('时间（s）'),ylabel('经纬度误差（°）'),legend('经度误差','纬度误差')
figure
plot(t,ere(:,3))
grid on
xlabel('时间（s）'),ylabel('高度误差（m）')
figure
%plot(t,ev);hold on
h2=plot(t1,ev(N1,1),'k-',t1,ev(N1,2),'ks-',t1,ev(N1,3),'k*-')
grid on
xlabel('时间（s）'),ylabel('速度误差（m/s）'),legend(h2,'东向速度误差','北向速度误差','天向速度误差');
% figure
% plot(t,ern)
% grid on
% title('松组合切平面坐标系误差')
% ylabel('米(m)')