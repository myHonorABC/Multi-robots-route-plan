%计算某一个点的适应度
%p为搜索区域的某一点，robotTO为目标机器人的位置，obX、obY分别为障碍物顶点的横坐标与纵坐标
%U_p为得到的p点的适应度
function  U_p = pFitness(p,robotTO,obX,obY)

bound = 120;                    %搜索区域的边界
td = 1;                         %探测到目标点的最远距离
od = 1;                         %障碍物产生影响的距离
ka = 1;                         
kr = 1;                         %ka,kr,wa,wr均为正实数
wa = 1;
wr =1;

%计算Ua
pgd = norm(p-[robotTO(1),robotTO(2)]);     %点p与目标点的距离
if pgd <= td                               %如果检测到目标点
    Ua = 500;
else                            %如果没有检测到目标点
    Ua = 1/2 * ka * pgd^(-2);
end

%计算Ur
pod = min(min(ro_distance(p,obX,obY)));    %p点与障碍物的最近距离
if pod <= od                               %如果p点在障碍物的影响范围内
    Ur = 1/2 * kr * (1/pod - 1/od) * pgd^2;
else
    Ur = 0;
end

%判断该点是否在障碍物内
[ob_m,~] = size(obX);          %获得障碍物的个数
for j = 1:ob_m
	in_ob = inpolygon(p(1),p(2),obX(j,:),obY(j,:));    %判断该点是否在第j个障碍物内
	if in_ob == 1              %如果在障碍物内
        break                  %结束循环
	end
end

%计算U_p
if   p(1)>0 && p(1)<bound && p(2)>0 && p(2)<bound && in_ob~=1   %如果p点在搜索区域内且不在障碍物内
    U_p = wa*Ua - wr*Ur;
else                     %如果p点不在搜索区域内或者在障碍物内
    U_p = -Inf;
end
