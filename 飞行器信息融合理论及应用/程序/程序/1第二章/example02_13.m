% exmaple2-13
% CA码发生自相关、互相关和频谱计算

clear all
close all

g1=ones(1,10); 
g2=ones(1,10); 
number1 = 7; % SVN 1
number2 = 3; % SVN 2

g2mod=[2,3,4,5,1,2,1,2,3,2,3,5,6,7,8,9,1,2,3,4,5,6,1,4,5,6,7,8,1,2,3,4;...
    6,7,8,9,9,10,8,9,10,3,4,6,7,8,9,10,4,5,6,7,8,9,3,6,7,8,9,10,6,7,8,9]; 

length = 1023; % length of CA code in a chip
for i=1:length  
    g2i=mod(g2(g2mod(1,number1))+g2(g2mod(2,number1)),2); 
    code(i)=mod(g2i+g1(10),2);  
    g1_updated=[mod(g1(3)+g1(10),2)]; 
    g1=[g1_updated g1(1:9)];  
    g2_updated=[mod(g2(2)+g2(3)+g2(6)+g2(8)+g2(9)+g2(10),2)]; 
    g2=[g2_updated g2(1:9)]; 
end
cacode=repmat(code,1,2); 
cacode(find(cacode==1))=-1;
cacode(find(cacode==0))=1;
cacode1 = cacode; % CA code for SVN 1
code(find(code==1))=-1;
code(find(code==0))=1;
cacode12 = code;

g1=ones(1,10); 
g2=ones(1,10); 
for i=1:length  
    g2i=mod(g2(g2mod(1,number2))+g2(g2mod(2,number2)),2); 
    code(i)=mod(g2i+g1(10),2);  
    g1_updated=[mod(g1(3)+g1(10),2)]; 
    g1=[g1_updated g1(1:9)];  
    g2_updated=[mod(g2(2)+g2(3)+g2(6)+g2(8)+g2(9)+g2(10),2)]; 
    g2=[g2_updated g2(1:9)]; 
end
cacode=repmat(code,1,2); 
cacode(find(cacode==1))=-1;
cacode(find(cacode==0))=1;
cacode2 = cacode; % CA code for SVN 2
code(find(code==1))=-1;
code(find(code==0))=1;
cacode22 = code;

figure(1)
autocorr(cacode1,length-1); % autocorrelation of the CA code of SVN 1;

figure(2)
crosscorr(cacode1,cacode2,length-1); % crosscorrelation of the CA codes of the two SVNs

figure(3)
spectrum_ca1 = fft(cacode1,2*length);  
spectrum1 = real(ifft(conj(spectrum_ca1).*spectrum_ca1))/2/length; 
spectrum = ifftshift(spectrum1);  
plot(spectrum)  
set(gca,'xtick',[0:200:2*length-1]); 
axis([0 2050 -0.1 1.1]);
xlabel('Lag'); 
ylabel('R(\tau)');

figure(4)
stairs(1:length,cacode(1:length));
axis([0 1030 -1.5 1.5])
xlabel('Lag'),ylabel('CA code')

figure(5)
stairs(1:50,cacode(1:50));
ylim([-1.5 1.5])
xlabel('Lag'),ylabel('CA code')

for i=1:length
    cacode_temp=circshift(cacode12',i-1)';
    rxx1(i)=sum(cacode12.*cacode_temp)/length;
    
    cacode_temp=circshift(cacode22',i-1)';
    rxx2(i)=sum(cacode22.*cacode_temp)/length;
    rxy(i)=sum(cacode12.*cacode_temp)/length;
end

figure(6)
plot(1:length,rxx1)
xlabel('\fontname{Times New Roman}Lag')
ylabel('\it\fontname{Times New Roman}R_x_x\rm(\tau)')
%legend('\fontname{Times New Roman}\beta_2=2','\fontname{Times New Roman}\beta_2=10')

figure(7)
plot(1:length,rxx2)
xlabel('\fontname{Times New Roman}Lag')
ylabel('\it\fontname{Times New Roman}R_x_x\rm(\tau)')

figure(8)
plot(1:length,rxy)
xlabel('\fontname{Times New Roman}Lag')
ylabel('\it\fontname{Times New Roman}R_x_y\rm(\tau)')
