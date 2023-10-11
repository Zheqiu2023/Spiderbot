%% 四元数插值
clear;
clc;
close all;
%% 建立机器人模型
%          theta            d   a       alpha
L1 = Link([0                0   0.637    pi/2]);
L2 = Link([0.1517*pi        0   0.618    0]);
L3 = Link([-0.4828*pi       0   2.14     0]);
L1.qlim = [-pi/6,pi/6];
L2.qlim = [(-pi*5)/18,(pi*5)/18];
L3.qlim = [(-pi*11)/36,(pi*11)/36];
spiderbot = SerialLink([L1 L2 L3],'name','leg');

%% 定义姿态
% p0 = transl(2.269,0,-1.562) * trotx(90) * trotz(-59.6);% 基准坐标系与末端坐标系变换矩阵
p1 = transl(2.036, 1.002, -1.562);% 根据给定起始点，得到起始点位姿
p2 = transl(2.894, 0.5, -1.35);% 中间点
p3 = transl(3.128, 0, -1.138);
p4 = transl(2.894, -0.5, -1.35);% 中间点
p5 = transl(2.036, -1.002, -1.562);% 根据给定终止点，得到终止点位姿

%% 姿态之间使用四元数球面线性插值（slerp）
q1 = trinterp(p1,p2,tpoly(0,1,30)/1);
q2 = trinterp(p2,p3,tpoly(0,1,30)/1);
q3 = trinterp(p3,p4,tpoly(0,1,30)/1);
q4 = trinterp(p4,p5,tpoly(0,1,30)/1);
Q2 = zeros(4,4,size(q1,3)+size(q2,3)+size(q3,3)+size(q4,3));
for i=1:size(q1,3)
      Q2(:,:,i)=q1(:,:,i);
end
for i=1:size(q2,3)
      Q2(:,:,30+i)=q2(:,:,i);
end
for i=1:size(q3,3)
      Q2(:,:,60+i)=q3(:,:,i);
end
for i=1:size(q4,3)
      Q2(:,:,90+i)=q4(:,:,i);
end

%% 逆解进行轨迹重现       
axis([-4 4 -4 4 -4 4]);
title("姿态插值");
xlabel('x/米','FontSize',12);
ylabel('y/米','FontSize',12);
zlabel('z/米','FontSize',12); 
hold on;
grid on;
mask = [1,1,1,0,0,0];
for i = 1:4:size(Q2,3)
% trplot(Q2(:,:,i));
P=Q2(:,:,i);
x=spiderbot.ikine(P,'mask',mask);
spiderbot.plot(x);% 动画演示
drawnow limitrate;
end
