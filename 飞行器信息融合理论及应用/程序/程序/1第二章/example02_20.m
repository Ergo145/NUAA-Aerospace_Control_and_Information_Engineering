% exmaple2-13
% CA码发生自相关、互相关和频谱计算

clear all
close all

n = 5;
length = 2^n-1;
g1 = ones(1,n); 

for i=1:length 
    code(i)=g1(n);  
    g_updated=[mod(g1(3)+g1(5),2)]; 
    g1=[g_updated g1(1:n-1)];  
end
%cacode=repmat(code,1,2); 
code(find(code==1))=-1;
code(find(code==0))=1;
cacode1 = code; % CA code for SVN 1

g2 = ones(1,n); 
for i=1:length
    code(i)=g2(n);  
    g_updated=[mod(g2(1)+g2(2)+g2(3)+g2(5),2)]; 
    g2=[g_updated g2(1:n-1)];  
end
%cacode=repmat(code,1,2); 
code(find(code==1))=-1;
code(find(code==0))=1;
cacode2 = code; % CA code for SVN 1

for i=1:length
    cacode_temp=circshift(cacode1',i-1)';
    rxx1(i)=sum(cacode1.*cacode_temp)/length;
    
    cacode_temp=circshift(cacode2',i-1)';
    rxx2(i)=sum(cacode2.*cacode_temp)/length;
    rxy(i)=sum(cacode1.*cacode_temp)/length;
end

figure(1)
plot(1:length,rxx1)
xlabel('\fontname{Times New Roman}Lag')
ylabel('\it\fontname{Times New Roman}R_x_x\rm(\tau)')
%legend('\fontname{Times New Roman}\beta_2=2','\fontname{Times New Roman}\beta_2=10')

figure(2)
plot(1:length,rxx2)
xlabel('\fontname{Times New Roman}Lag')
ylabel('\it\fontname{Times New Roman}R_x_x\rm(\tau)')

figure(3)
plot(1:length,rxy)
xlabel('\fontname{Times New Roman}Lag')
ylabel('\it\fontname{Times New Roman}R_x_y\rm(\tau)')

figure(4)
stairs(1:length,cacode1(1:length));
axis([0 31 -1.5 1.5])
xlabel('Lag'),ylabel('Pseudorandom code')

N=length;
tc=1/N;
f0=1;
sn=[];
for n=-100:1:100
    xnr=0;
    xni=0;
    for i=1:N
        xnr=xnr+cacode1(i)*cos(2*pi*f0*n*i*tc);
        xni=xni+cacode1(i)*sin(2*pi*f0*n*i*tc);
    end
    temp=tc*sinc(n*f0*tc)^2*(xnr^2+xni^2)/N;
    sn=[sn;temp];
end
snlog=10*log10(sn);
figure(5)
%plot(-100:100,snlog);
stem(-100:100,sn);
xlabel('\fontname{Times New Roman}\itf')
ylabel('\it\fontname{Times New Roman}S_x_x\rm(\itf\rm)')