%% 可达空间
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

%% 求解可达空间
N = 20000;
A = unifrnd(-pi/6,pi/6,[1,N]);    % 基节角度限位
B = unifrnd((-pi*5)/18,(pi*5)/18,[1,N]);  % 大腿角度限位
C = unifrnd((-pi*11)/36,(pi*11)/36,[1,N]);    % 小腿角度限位
G = cell(N, 3);  % 建立元胞数组
for n = 1:1:N     % for循环的执行次数
    G{n} = [A(n) B(n) C(n)];
end                                   % 产生20000组随机点
H1 = cell2mat(G);                       % 将元胞数组转化为矩阵
T = double(spiderbot.fkine(H1));        % 机械臂正解

%% 绘图
scatter3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)),2); % 随机点图
spiderbot.plot([0  0.1517*pi  -0.4828*pi],'workspace',[-5 5 -5 5 -5 5],'tilesize',2);    % 机械臂图
spiderbot.teach([0  0.1517*pi  -0.4828*pi]);