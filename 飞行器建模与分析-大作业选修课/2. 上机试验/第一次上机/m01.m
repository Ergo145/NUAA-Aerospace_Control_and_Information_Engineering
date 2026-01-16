clc;clear
% %% 绘制结果
% %绘制y1 and y2
% figure;
% plot(t, y(:,1), 'r-', t, y(:,2), 'b-', 'LineWidth', 2);
% xlabel('t');
% ylabel('y1 and y2');
% legend('y_1', 'y_2');
% grid on;
% %绘制y1
% figure;
% subplot(2,1,1);
% plot(t,y(:,1),'-','LineWidth',2);
% xlabel('t');
% ylabel('y1');
% grid on;
% %绘制y2
% subplot(2,1,2);
% plot(t,y(:,2),'-','LineWidth',2);
% xlabel('t');
% ylabel('y2');
% grid on;