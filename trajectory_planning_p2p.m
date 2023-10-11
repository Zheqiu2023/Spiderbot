%% 点到点规划
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

%% 求解逆运动学 轨迹规划
%T0 = transl(2.269,0,-1.562) * trotx(90) * trotz(-59.6);% 基准坐标系与末端坐标系变换矩阵
T1 = transl(2.036, -1.002, -1.562);% 根据给定起始点，得到起始点位姿
T2 = transl(2.036, 1.002, -1.562);% 根据给定终止点，得到终止点位姿

mask = [1,1,1,0,0,0];
q1 = spiderbot.ikine(T1,'mask',mask);% 根据起始点位姿，得到起始点关节角
q2 = spiderbot.ikine(T2,'mask',mask);% 根据终止点位姿，得到终止点关节角

% 五次多项式轨迹，得到关节角度，角速度，角加速度，80为采样点个数，修改采样点个数可调整速度
[q, qd, qdd] = jtraj(q1,q2,tpoly(0,1,80)/1);

subplot(2,2,1);plot(qd(:,1));grid on;xlabel('时间');ylabel('速度x');
subplot(2,2,2);plot(qd(:,2));grid on;xlabel('时间');ylabel('速度y');
subplot(2,2,3);plot(qd(:,3));grid on;xlabel('时间');ylabel('速度z');
subplot(2,2,4);T = double(spiderbot.fkine(q));% 根据插值，得到末端执行器位姿

%% 绘图
plot3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)));% 输出末端轨迹
spiderbot.plot(q,'workspace',[-4 4 -4 4 -4 4]);% 动画演示