%绘制搜索机器人与目标机器人
%n1,n2分别为搜索机器人与目标机器人的个数；r1,r2分别为搜索机器人与目标机器人的半径
%c1,c2分别为搜索机器人的中心点坐标集合和目标机器人的中心点坐标集合
function  [c1,c2] = drawRobot(n1,r1,n2,r2)

c1 = zeros(n1,2);
c2 = zeros(n2,2);
for i = 1:n1                                  %绘制搜索机器人
    theta = 0:0.1:2*pi;
    circleX = 5*i + r1*cos(theta);  
    circleY = 5 + r1*sin(theta);  
    plot(circleX,circleY,'-','linewidth',1);  
    fill(circleX,circleY,'b')
    c1(i,1) = 5*i;
    c1(i,2) = 5;
    hold on;
end
for i = 1:n2                                  %绘制目标机器人
    theta = 0:0.1:2*pi;
    circleX = (120 - 5 * i) + r2 * cos(theta);  
    circleY = 115 + r2 * sin(theta);    
    plot(circleX,circleY,'-','linewidth',1);  
    fill(circleX,circleY,'r')
    c2(i,1) = 120 - 5*i;
    c2(i,2) = 115;
    hold on;
end
axis equal  
