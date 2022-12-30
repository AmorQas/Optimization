% 该文件用于生成随机环境数据并保存

% 生成随机的障碍物高度
Z1 = randi(1000,21,21);
Z2 = randi(1000,21,21);
Z2 = Z2 + 1000;
% 建立 X Y Z 坐标系
[X, Y] = meshgrid([1:21],[1:21]);
% 画图
subplot(211);
surf(X, Y, Z1);
title('BottomEnv');
subplot(212);
surf(X, Y, Z2);
title('TopEnv');
%保存
save('TopEnv','Z1');
save('BottomEnv','Z2');
