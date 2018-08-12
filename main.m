clear
clc

[obX,obY] = polygon(4,5);                    %绘制障碍物并返回障碍物的顶点坐标
[robotSO,robotTO] = drawRobot(1,2.5,1,2.5);     %初始化搜索机器人与目标机器人的起点

%计算搜索区域整数点上的适应度
point_fitness = zeros(121,121);
for i = 0:120
    for j = 0:120
        point_fitness(i+1,j+1) = pFitness([i,j],robotTO,obX,obY);    %记录每一点的适应度
    end
end

%调用粒子群算法对目标机器人进行搜索
[walkPoint,num] = pso(robotSO,robotTO,obX,obY);
[n1,~] = size(robotSO);
a = {'r','g','b'};
for  i = 1 : n1
    for  j = 1 : num-1
        plot([walkPoint{i}(j,1),walkPoint{i}(j+1,1)],[walkPoint{i}(j,2),walkPoint{i}(j+1,2)],a{i},'LineWidth',1);
        hold on;
    end
end



axis([0,120,0,120]);                 %设置坐标轴的范围
