%粒子群算法
%robotSO为每个搜索机器人的初始点，robotTO为每个目标机器人的初始点
%obX为障碍物顶点的横坐标；obY为障碍物顶点的纵坐标
%hasWalk为机器人经过的点
%count为机器人经过的点的个数
function  [hasWalk,count] = pso(robotSO,robotTO,obX,obY)

[n1,~] = size(robotSO);                    %搜索机器人的个数
global flagTO;                       
flagTO = 0;                       %全局变量flagTO记录目标机器人是否被找到
p = robotSO + [0,2.5];                      %搜索机器人的当前位置
s = 1;                            %搜索机器人行走一次的步长
v = [0,s];                              %搜索机器人的初始速度
count = ones(1,n1);                    %搜索机器人经过点的个数
hasWalk = cell(1,n1);                       %搜索机器人已经走过的点
td = 5;                           %搜索机器人的检测距离
c1 = 1/18;
c2 = 1/9;
c3 = 1 - c1 -c2;

%初始化hasWalk
for i = 1:n1
    fit0 = pFitness([p(i,1),p(i,2)],robotTO,obX,obY);      %搜索机器人初始点的适应度
    hasWalk{i} = [p(i,1),p(i,2),fit0];                     %将初始点与初始点的适应度写入hasWalk
end

step = 1;                         %搜索机器人每行走一步step加1

%搜索机器人开始搜索
while 1
    if flagTO == 1
        break;
    end
    for i = 1:n1                              %遍历这n1个搜索机器人
        p0 = p;                               %保存未更新前的点
        [~,f] = aroundFitness([p(i,1),p(i,2)],s,robotTO,obX,obY);
        [~,fn] = size(f);                     %适应度值最小的相邻点有几个
        if fn ~= 1
            f = f(randi([1,fn]));             %从fn个相邻点中任选一个相邻点
        end
        %计算搜索机器人速度的局部最优值
        if f == 1
            v_p = [-2^(-1/2),2^(-1/2)];
        elseif f == 2
            v_p = [0,1];
        elseif f == 3
            v_p = [2^(-1/2),2^(-1/2)];
        elseif f == 4
            v_p = [1,0]*s;
        elseif f == 5
            v_p = [2^(-1/2),-2^(-1/2)];
        elseif f == 6
            v_p = [0,-1]*s;
        elseif f == 7
            v_p = [-2^(-1/2),-2^(-1/2)];
        else
            v_p = [-1,0];
        end
        %计算机器人速度的全局最优值
        %fit = zeros(1,count(i));                  %已走过的点的适应度，分配内存空间
        %for j = 1:count(i)
        %    fit(j) = pFitness([hasWalk{i}(j,1),hasWalk{i}(j,2)],robotTO,obX,obY); 
        %end
        %fit_max = max(fit);
        %row = find(fit==fit_max);
        [~,row] = max(hasWalk{i}(:,3));            %从hasWalk中寻找适应度最大的那一行
        v_g = ([hasWalk{i}(row,1),hasWalk{i}(row,2)]-[p(i,1),p(i,2)])/norm([hasWalk{i}(row,1),hasWalk{i}(row,2)]-[p(i,1),p(i,2)]);    %速度的全局最优解
        %{
        %未加全局最优解时的情况
        v1 = c1*v + c2*v_p;
        v1 = v1/norm(v1)*s;
        p1(1) = p(i,1) + v1(1);
        p1(2) = p(i,2) + v1(2);
        fit1 = pFitness([p1(1),p1(2)],robotTO,obX,obY);     %未加全局最优解时的适应度
        %加入全局最优解的情况
        v2 = c1*v + c2*v_p + c3*v_g;
        v2 = v2/norm(v2)*s;
        p2(1) = p(i,1) + v2(1);
        p2(2) = p(i,2) + v2(2);
        fit2 = pFitness([p2(1),p2(2)],robotTO,obX,obY);     %加入全局最优解时的适应度
        %选择适应度最大的情况
        if fit1 >= fit2
            p(i,1) = p1(1);
            p(i,2) = p1(2);
        else
            p(i,1) = p2(1);
            p(i,2) = p2(2);
        end
        %}
        
        v = c1*v + c2*v_p;
        v = v/norm(v)*s;
        p(i,1) = p(i,1) + v(1);
        p(i,2) = p(i,2) + v(2);
        
        fit = pFitness([p(i,1),p(i,2)],robotTO,obX,obY);    %计算此时的适应度
        if fit == -Inf                                      %如果下一个目标点不在搜索区域内或者在障碍物内
            while 1
                pr = [p0(1)+2^(-1/2)*(2*rand()-1) , p0(2)+2^(-1/2)*(2*rand()-1)];  %在未更新前的点的基础上任意给一个点
                pr = pr/norm(pr)*s;
                if  pFitness(pr,robotTO,obX,obY) ~= -Inf       %如果该点在搜索区域内且不在障碍物内
                    p(i,1) = pr(1);                        %更新下一个位置
                    p(i,2) = pr(2);
                    fit = pFitness(pr,robotTO,obX,obY);  %下一个位置的适应度
                    break;
                end
            end
        end
        hasWalk{i} = [hasWalk{i};p(i,1),p(i,2),fit];             %将更新后的机器人位置写入hasWalk元组
        count(i) = count(i) + 1;                                 %机器人经过点的个数加1
        step =step+1;
        disp(['计数：step= ',num2str(step)]);
        rgd = norm([p(i,1),p(i,2)]-[robotTO(1),robotTO(2)]); 
        if rgd <= td && flagTO==0                            %如果搜索机器人附近存在未被搜索到的目标机器人
            flagTO = 1;                                      %标记该目标机器人已经找到
        end
    end
end

%walkPoint = hasWalk;
