function [Ro,L_best,L_ave]=ACATSP(C,D)
% 主要符号说明
% C n个城市的坐标，n×2的矩阵
% NC_max 最大迭代次数
NC_max=500;
% m 蚂蚁个数
m=100;
% Alpha 表征信息素重要程度的参数
Alpha=1.5;
% Beta 表征启发式因子重要程度的参数
Beta=2;
% Rho 信息素蒸发系数
Rho=0.9;
% Q 信息素增加强度系数
Q=10;
% R_best 各代最佳路线
% L_best 各代最佳路线的长度
%%=========================================================================
%%第一步：变量初始化

n=size(C,1);%n表示问题的规模（城市个数）

Eta=1./D;        

Tau=ones(n,n);    

Tabu=zeros(m,n);   

NC=1;   


R_best=zeros(NC_max,n);      

L_best=inf.*ones(NC_max,1);   

L_ave=zeros(NC_max,1);     


while NC<=NC_max        %停止条件之一：达到最大迭代次数，停止

%%第二步：将m只蚂蚁放到n个城市上

Randpos=[];   %随即存取

   for i=1:(ceil(m/n))
% Randpos=[1~31 + 1~31]   将每只蚂蚁放到随机的城市  
% Randpos 中随机选择n个数，代表蚂蚁的初始城市
   Randpos=[Randpos,randperm(n)];
   end

Tabu(:,1)=(Randpos(1,1:m))';    % 第一次迭代每只蚂蚁的禁忌表


%%第三步：m只蚂蚁按概率函数选择下一座城市，完成各自的周游

for j=2:n     %所在城市不计算

for i=1:m     

visited=Tabu(i,1:(j-1)); 

J=zeros(1,(n-j+1));      

P=J;                      

Jc=1;

for k=1:n

if length(find(visited==k))==0   %开始时置0

J(Jc)=k;

Jc=Jc+1;                        
  end

end

%下面计算待选城市的概率分布

for k=1:length(J)

P(k)=(Tau(visited(end),J(k))^Alpha)*(Eta(visited(end),J(k))^Beta);

end

P=P/(sum(P));

%按概率原则选取下一个城市

Pcum=cumsum(P);    

Select=find(Pcum>=rand); 

if Select(1) <= length(J)
    to_visit = J(Select(1));
else
    % 处理超出数组索引范围的情况，例如给出一个默认值或者抛出错误
end

% to_visit=J(Select(1)); %原来的只能找8个目标点
 
Tabu(i,j)=to_visit;

end

end

if NC>=2

Tabu(1,:)=R_best(NC-1,:);

end


%%第四步：记录本次迭代最佳路线

L=zeros(m,1);     %开始距离为0，m*1的列向量

for i=1:m

R=Tabu(i,:);

for j=1:(n-1)

L(i)=L(i)+D(R(j),R(j+1));  

end

L(i)=L(i)+D(R(1),R(n));    

end

L_best(NC)=min(L);           %最佳距离取最小

pos=find(L==L_best(NC));

R_best(NC,:)=Tabu(pos(1),:); 

L_ave(NC)=mean(L);           

NC=NC+1;                      



%%第五步：更新信息素

Delta_Tau=zeros(n,n);       

for i=1:m

for j=1:(n-1)

Delta_Tau(Tabu(i,j),Tabu(i,j+1))=Delta_Tau(Tabu(i,j),Tabu(i,j+1))+Q/L(i);           

%此次循环在路径（i，j）上的信息素增量

end

Delta_Tau(Tabu(i,n),Tabu(i,1))=Delta_Tau(Tabu(i,n),Tabu(i,1))+Q/L(i);

%此次循环在整个路径上的信息素增量

end

Tau=(1-Rho).*Tau+Delta_Tau; %考虑信息素挥发，更新后的信息素

%%第六步：禁忌表清零

Tabu=zeros(m,n);             %%直到最大迭代次数

end

%%第七步：输出结果

Pos=find(L_best==min(L_best)); 

Shortest_Route=R_best(Pos(1),:);

Shortest_Length=L_best(Pos(1)) ;

 Ro=Shortest_Route;




