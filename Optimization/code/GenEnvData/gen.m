% ���ļ�������������������ݲ�����

% ����������ϰ���߶�
Z1 = randi(1000,21,21);
Z2 = randi(1000,21,21);
Z2 = Z2 + 1000;
% ���� X Y Z ����ϵ
[X, Y] = meshgrid([1:21],[1:21]);
% ��ͼ
subplot(211);
surf(X, Y, Z1);
title('BottomEnv');
subplot(212);
surf(X, Y, Z2);
title('TopEnv');
%����
save('TopEnv','Z1');
save('BottomEnv','Z2');
