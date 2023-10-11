%% 特定轨迹规划
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
%p0 = transl(2.269,0,-1.562) * trotx(90) * trotz(-59.6);% 基准坐标系与末端坐标系变换矩阵
p1 = transl(2.036, 1.002, -1.562);% 根据给定起始点，得到起始点位姿
p2 = transl(2.894, 0.5, -1.35);% 中间点
p3 = transl(3.128, 0, -1.138);
p4 = transl(2.894, -0.5, -1.35);% 中间点
p5 = transl(2.036, -1.002, -1.562);% 根据给定终止点，得到终止点位姿

mask = [1,1,1,0,0,0];
j1 = spiderbot.ikine(p1,'mask',mask);% 根据起始点位姿，得到起始点关节角
j2 = spiderbot.ikine(p2,'mask',mask);
j3 = spiderbot.ikine(p3,'mask',mask);
j4 = spiderbot.ikine(p4,'mask',mask);
j5 = spiderbot.ikine(p5,'mask',mask);% 根据终止点位姿，得到终止点关节角
                
%% 方法1————关节空间轨迹(速度可以连续，但中间点速度需自己设置)
% 五次多项式轨迹，得到轨迹上各点关节角度，角速度，角加速度，30为采样点个数，修改采样点个数可调整速度
[q1, qd1, qdd1] = jtraj(j1,j2,20,[0,0,0],[-0.2,0.1,0.1]);
[q2, qd2, qdd2] = jtraj(j2,j3,20,[-0.2,0.1,0.1],[-0.2,0,0]);
[q3, qd3, qdd3] = jtraj(j3,j4,20,[-0.2,0,0],[-0.2,-0.1,-0.1]);
[q4, qd4, qdd4] = jtraj(j4,j5,20,[-0.2,-0.1,-0.1],[0,0,0]);
q = [q1;q2;q3;q4]
qd = [qd1;qd2;qd3;qd4];
% 正运动学得到末端执行器轨迹
T1 = double(spiderbot.fkine(q1));% 正运动学解出末端位姿，T1为所有转换矩阵
T2 = double(spiderbot.fkine(q2));
T3 = double(spiderbot.fkine(q3));
T4 = double(spiderbot.fkine(q4));
% 绘图
plot3(squeeze(T1(1,4,:)),squeeze(T1(2,4,:)),squeeze(T1(3,4,:)));% 输出末端轨迹
hold on;
plot3(squeeze(T2(1,4,:)),squeeze(T2(2,4,:)),squeeze(T2(3,4,:)));
hold on;
plot3(squeeze(T3(1,4,:)),squeeze(T3(2,4,:)),squeeze(T3(3,4,:)));
hold on;
plot3(squeeze(T4(1,4,:)),squeeze(T4(2,4,:)),squeeze(T4(3,4,:)));
hold on;

spiderbot.plot(q,'workspace',[-4 4 -4 4 -4 4]);% 动画演示
% 速度和加速度曲线
subplot(3,2,1);plot(qd3(:,1));grid on;xlabel('时间');ylabel('速度x');
subplot(3,2,2);plot(qdd3(:,1));grid on;xlabel('时间');ylabel('加速度x');
subplot(3,2,3);plot(qd3(:,2));grid on;xlabel('时间');ylabel('速度y');
subplot(3,2,4);plot(qdd3(:,2));grid on;xlabel('时间');ylabel('加速度y');
subplot(3,2,5);plot(qd3(:,3));grid on;xlabel('时间');ylabel('速度z');
subplot(3,2,6);plot(qdd3(:,3));grid on;xlabel('时间');ylabel('加速度z');
%% 方法2————笛卡尔空间轨迹(速度无法连续)
% % 匀加速、匀减速轨迹(直线轨迹)，得到末端轨迹
% tc1 = ctraj(p1,p2,20);
% tc2 = ctraj(p2,p3,20);
% tc3 = ctraj(p3,p4,20);
% tc4 = ctraj(p4,p5,20);
% 
% mask = [1,1,1,0,0,0];
% q1 = double(spiderbot.ikine((tc1),'mask',mask));
% q2 = double(spiderbot.ikine((tc2),'mask',mask));
% q3 = double(spiderbot.ikine((tc3),'mask',mask));
% q4 = double(spiderbot.ikine((tc4),'mask',mask));
% q = [q1;q2;q3;q4];
% % 绘图
% subplot(1,2,1);
% plot3(squeeze(tc1(1,4,:)),squeeze(tc1(2,4,:)),squeeze(tc1(3,4,:)));% 输出末端轨迹
% hold on;
% plot3(squeeze(tc2(1,4,:)),squeeze(tc2(2,4,:)),squeeze(tc2(3,4,:)));
% hold on;
% plot3(squeeze(tc3(1,4,:)),squeeze(tc3(2,4,:)),squeeze(tc3(3,4,:)));
% hold on;
% plot3(squeeze(tc4(1,4,:)),squeeze(tc4(2,4,:)),squeeze(tc4(3,4,:)));
% hold on;
% subplot(1,2,2);
% spiderbot.plot(q,'workspace',[-4 4 -4 4 -4 4]);%动画演示

%% 方法3————多段轨迹(速度连续)
% % 路径生成--关节角的组合
% path12 = [j1;j2];
% path23 = [j2;j3];
% path34 = [j3;j4];
% path45 = [j4;j5];
% finalpath = [path12;path23;path34;path45];
% 
% qdmax = [1 1 1];
% q = mstraj(finalpath,qdmax,[],[],0.1,0);    % traj为轨迹上各点对应的关节角
% qd = diff(q)   % 对关节角微分求3速度
% tran = double(spiderbot.fkine(q));  % 正运动学解出末端位姿，tran为所有转换矩阵
% 
% plot3(squeeze(tran(1,4,:)),squeeze(tran(2,4,:)),squeeze(tran(3,4,:)));
% spiderbot.plot(q,'workspace',[-4 4 -4 4 -4 4]);