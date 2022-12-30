#### **七、附录(源码)**

```matlab
% ./code/GenEnvData/gen.m
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
```

```matlab
% ./code/Simulation/main.m
%% 主函数入口，进行三维路径规划

%% 清空环境
clc
clear

%% 数据初始化

%下载数据
load  HeightData HeightData

%网格划分
LevelGrid=10;
PortGrid=21;

%起点终点网格点 
starty=10;starth=4;
endy=8;endh=6;
m=1;
%算法参数
PopNumber=10;         %种群个数
BestFitness=[];    %最佳个体

%初始信息素
pheromone=ones(21,21,21);

%% 初始搜索路径
[path,pheromone]=searchpath(PopNumber,LevelGrid,PortGrid,pheromone, ...
    HeightData,starty,starth,endy,endh); 
fitness=CacuFit(path);                          %适应度计算
[bestfitness,bestindex]=min(fitness);           %最佳适应度
bestpath=path(bestindex,:);                     %最佳路径
BestFitness=[BestFitness;bestfitness];          %适应度值记录
 
%% 信息素更新
rou=0.2;
cfit=100/bestfitness;
for i=2:PortGrid-1
    pheromone(i,bestpath(i*2-1),bestpath(i*2))= ...
        (1-rou)*pheromone(i,bestpath(i*2-1),bestpath(i*2))+rou*cfit;
end
    
%% 循环寻找最优路径
for kk=1:300
    %% 路径搜索
    [path,pheromone]=searchpath(PopNumber,LevelGrid,PortGrid,...
        pheromone,HeightData,starty,starth,endy,endh); 
    
    %% 适应度值计算更新
    fitness=CacuFit(path);                               
    [newbestfitness,newbestindex]=min(fitness);     
    if newbestfitness<bestfitness
        bestfitness=newbestfitness;
        bestpath=path(newbestindex,:);
    end
    BestFitness=[BestFitness;bestfitness];
    
    %% 更新信息素
    cfit=100/bestfitness;
    for i=2:PortGrid-1
        pheromone(i,bestpath(i*2-1),bestpath(i*2))=(1-rou)* ...
            pheromone(i,bestpath(i*2-1),bestpath(i*2))+rou*cfit;
    end
end

%% 最佳路径
for i=1:21
    a(i,1)=bestpath(i*2-1);
    a(i,2)=bestpath(i*2);
end
figure(1)
x=1:21;
y=1:21;
[x1,y1]=meshgrid(x,y);
mesh(x1,y1,HeightData)
axis([1,21,1,21,0,2000])
hold on
k=1:21;
plot3(k(1)',a(1,1)',a(1,2)'*200,'--o','LineWidth',2,...
                       'MarkerEdgeColor','k',...
                       'MarkerFaceColor','g',...
                       'MarkerSize',10)
plot3(k(21)',a(21,1)',a(21,2)'*200,'--o','LineWidth',2,...
                       'MarkerEdgeColor','k',...
                       'MarkerFaceColor','g',...
                       'MarkerSize',10)
                   text(k(1)',a(1,1)',a(1,2)'*200,'S');
text(k(21)',a(21,1)',a(21,2)'*200,'T');
xlabel('km','fontsize',12);
ylabel('km','fontsize',12);
zlabel('m','fontsize',12);
title('BottomEnv','fontsize',12)
set(gcf, 'Renderer', 'ZBuffer')
hold on
plot3(k',a(:,1)',a(:,2)'*200,'--o')

%% 适应度变化
figure(2)
plot(BestFitness)
title('最佳个体适应度变化趋势')
xlabel('迭代次数')
ylabel('适应度值')

```

```matlab
% ./code/Simulation/CacuFit.m
function fitness=CacuFit(path)
%% 该函数用于计算个体适应度值
%path       input     路径
%fitness    input     路径
[n,m]=size(path);
for i=1:n
    fitness(i)=0;
    for j=2:m/2
        %适应度值为长度加高度
        fitness(i)=fitness(i)+sqrt(1+(path(i,j*2-1)-path(i,(j-1)*2-1))^2 ...
            +(path(i,j*2)-path(i,(j-1)*2))^2)+abs(path(i,j*2));
    end
end
```

```matlab
% ./code/Simulation/CacuQfz.m
function qfz=CacuQfz(Nexty,Nexth,Nowy,Nowh,endy,endh,abscissa,HeightData)
% 该函数用于计算各点的启发值
%Nexty Nexth    input    下个点坐标
%Nowy Nowh      input    当前点坐标
%endy endh      input    终点坐标
%abscissa       input    横坐标
%HeightData     input    地图高度
%qfz            output   启发值

% 判断下个点是否可达
if HeightData(Nexty,abscissa)<Nexth*200
    S=1;
else
    S=0;
end

% 计算启发值
%D距离
D=50/(sqrt(1+(Nowh*0.2-Nexth*0.2)^2+(Nexty-Nowy)^2)+sqrt((21-abscissa)^2 ...
    +(endh*0.2-Nexth*0.2)^2+(endy-Nowy)^2));

%计算高度
M=30/abs(Nexth+1);
%计算启发值
qfz=S*M*D;
end
```

```matlab
% ./code/Simulation/searchpath.m
function [path,pheromone]=searchpath(PopNumber,LevelGrid,PortGrid,pheromone,HeightData,starty,starth,endy,endh)
%% 该函数用于蚂蚁蚁群算法的路径规划
%LevelGrid     input    横向划分格数
%PortGrid      input    纵向划分个数
%pheromone     input    信息素
%HeightData    input    地图高度
%starty starth input    开始点
%path          output   规划路径
%pheromone     output   信息素
%% 搜索参数
ycMax=2;   %蚂蚁横向最大变动
hcMax=2;   %蚂蚁纵向最大变动
decr=0.5;  %信息素衰减概率
%% 循环搜索路径
for ii=1:PopNumber
    path(ii,1:2)=[starty,starth];  %记录路径
    NowPoint=[starty,starth];      %当前坐标点 
    %% 计算点适应度值
    for abscissa=2:PortGrid-1
        %计算所有数据点对应的适应度值
        kk=1;
        for i=-ycMax:ycMax
            for j=-hcMax:hcMax
                NextPoint(kk,:)=[NowPoint(1)+i,NowPoint(2)+j];
                if (NextPoint(kk,1)<PortGrid)&&(NextPoint(kk,1)>0)&&(NextPoint(kk,2)<LevelGrid)&&(NextPoint(kk,2)>0)
                    qfz(kk)=CacuQfz(NextPoint(kk,1),NextPoint(kk,2),NowPoint(1),NowPoint(2),endy,endh,abscissa,HeightData);
                    qz(kk)=qfz(kk)*pheromone(abscissa,NextPoint(kk,1),NextPoint(kk,2));
                    kk=kk+1;
                else
                    qz(kk)=0;
                    kk=kk+1;
                end
            end
        end
        %选择下个点
        sumq=qz./sum(qz);
        pick=rand;
        while pick==0
            pick=rand;
        end
        for i=1:25
            pick=pick-sumq(i);
            if pick<=0
                index=i;
                break;
            end
        end
        oldpoint=NextPoint(index,:);
        %更新信息素
        pheromone(abscissa,oldpoint(1),oldpoint(2))=decr*pheromone(abscissa,oldpoint(1),oldpoint(2));
        %路径保存
        path(ii,abscissa*2-1:abscissa*2)=[oldpoint(1),oldpoint(2)];
        NowPoint=oldpoint; 
    end
    path(ii,41:42)=[endy,endh];
end
```

