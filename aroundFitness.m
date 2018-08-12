%计算搜索机器人当前位置邻近8个位置的适应度
%p_n表示搜索机器人的当前位置，robotTO表示目标机器人的位置，obX,obY分别为障碍物顶点的横坐标与纵坐标，s为机器人行走一次的步长
%U_min为得到的最小适应度，f表示哪几个邻近点的适应度最小
function  [U_max,f] = aroundFitness(p_n,s,robotTO,obX,obY)

U = zeros(1,8);                     %记录相邻8个位置的适应度，分配内存空间
%初始化around
around = [p_n(1)-2^(-1/2)*s , p_n(2)+2^(-1/2)*s           %机器人相邻的8个位置
          p_n(1)            , p_n(2)+s
          p_n(1)+2^(-1/2)*s , p_n(2)+2^(-1/2)*s
          p_n(1)+s          , p_n(2)
          p_n(1)+2^(-1/2)*s , p_n(2)-2^(-1/2)*s
          p_n(1)            , p_n(2)-s
          p_n(1)-2^(-1/2)*s , p_n(2)-2^(-1/2)*s
          p_n(1)-s          , p_n(2)           ];

%计算相邻8个位置的适应度
for i = 1:8
    U(i) = pFitness([around(i,1),around(i,2)],robotTO,obX,obY);
end

U_max = max(U);
f = find(U==U_max);
