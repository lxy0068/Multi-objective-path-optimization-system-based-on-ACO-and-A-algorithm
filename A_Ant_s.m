clc ;
clear;
figure 
% 基于蚁群+A多目标点路径规划
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                地图建模
MAX0=MAX();
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%只能设置正方形矩阵，行和列相等，否则旋转时会出现错误

%%% 通道设置为 0 ；障碍点设置为 1 ；起始点设置为 2 ；目标点设置为 -1 。
MAX=rot90(MAX0,3);      %%%设置0,1摆放的图像与存入的数组不一样，需要先逆时针旋转90*3=270度给数组，最后输出来的图像就是自己编排的图像
MAX_X=size(MAX,2);                                %%%  获取列数，即x轴长度
MAX_Y=size(MAX,1);                                %%%  获取行数，即y轴长度
MAX_VAL=10;                              %%%   返回由数字组成的字符表达式的数字值，就是函数用于将数值字符串转换为数值


x_val = 1;
y_val = 1;
axis([1 MAX_X+1, 1 MAX_Y+1])                %%%  设置x，y轴上下限
set(gca,'xtick',1:1:MAX_X+1,'ytick',1:1:MAX_Y+1,'GridLineStyle','-',... 
    'xGrid','on','yGrid','on')
grid on;                                   %%%  在画图的时候添加网格线
hold on;                                   %%%  当前轴及图像保持而不被刷新，准备接受此后将绘制的图形，多图共存
n=0;%Number of Obstacles                   %%%  障碍的数量


k=1;          %%%% 将所有障碍物放在关闭列表中；障碍点的值为1;并且显示障碍点
CLOSED=[];
for j=1:MAX_X
    for i=1:MAX_Y
        if (MAX(i,j)==1)
          %%plot(i+.5,j+.5,'ks','MarkerFaceColor','b'); 原来是红点圆表示
          fill([i,i+1,i+1,i],[j,j,j+1,j+1],'k');  %%%改成 用黑方块来表示障碍物
          CLOSED(k,1)=i;  %%% 将障碍点保存到CLOSE数组中
          CLOSED(k,2)=j; 
          k=k+1;
        end
    end
end
Area_MAX(1,1)=MAX_X;
Area_MAX(1,2)=MAX_Y;
Obs_Closed=CLOSED;
num_Closed=size(CLOSED,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                       设置起始点和多个目标点
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%   选择起始位置
h=msgbox('请使用鼠标左键选择车辆初始位置');                    %%%原文 Please Select the Vehicle initial position using the Left Mouse button
uiwait(h,5);
if ishandle(h) == 1
    delete(h);
end
xlabel('请选择车辆初始位置 ','Color','red');                %%% 原文 Please Select the Vehicle initial position
but=0;
while (but ~= 1) %Repeat until the Left button is not clicked %%%重复，直到没有单击“向左”按钮
    [xval,yval,but]=ginput(1);
    xval=floor(xval);
    yval=floor(yval);
end
xStart=xval;%Starting Position
yStart=yval;%Starting Position

Start(1,1)=xStart;
Start(1,2)=yStart;

                                               %%%   起始点位置的值设置为1；目标点为0，障碍点为-1，其余空白点为2
plot(xStart+.5,yStart+.5,'b^');
text(xStart+1,yStart+1.5,'S','fontsize',12')

pause(2);
h=msgbox('请使用鼠标左键选择多目标地点,右键选择最后一个目标地点');
  xlabel('请使用鼠标左键选择多目标地点,右键选择最后一个目标地点','Color','blue');
uiwait(h,10);
if ishandle(h) == 1
    delete(h);
end
while but == 1
    [xval,yval,but] = ginput(1);
    xval=floor(xval);
    yval=floor(yval);
    MAX(xval,yval)=-1;%Put on the closed list as well
    plot(xval+.5,yval+.5,'bo');
   
 end%End of While loop
 

gg=1;%Dummy counter
for i=1:MAX_X
    for j=1:MAX_Y
      if i <= size(MAX, 1) && j <= size(MAX, 2)
        if(MAX(i,j) == -1)
            Goal(gg,1)=i; 
            Goal(gg,2)=j; 
            text(i+1,j+1.5,num2str(gg,'T(%d)'),'fontsize',12')
            gg=gg+1;
        end
      end
    end
end
Goal_COUNT=size(Goal,1);

xlabel('起始点位置标记为 △ ，目标点位置标记为 o ','Color','red'); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                     计算起始点目标点 两两组合的路径节点 及 长度
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

X=[Start;Goal]; %将  起始点和多个目标点  组合在一起
X_num=Goal_COUNT+1;
% A_X=X_num*Goal_COUNT;%%%排列组合，A_X 组 起始点目标点路径规划
Distanse_xx=zeros(X_num,X_num);  %%% 用来保存每组路径长度

Pa_X=cell(X_num,X_num);          %%% 用来保存每组路径节点 
Num_open=0;
for i=1:1:X_num
    for j=1:1:X_num
       if i~=j
        St(1,1) = X(i,1);   St(1,2) = X(i,2);
        Ta(1,1) = X(j,1);   Ta(1,2) = X(j,2);
        [path_X,distance_x,OPEN_num]=Astar_s(Obs_Closed,St,Ta,MAX_X,MAX_Y);
        Distanse_xx(i,j)=distance_x;
        Num_open=Num_open+OPEN_num;
        Pa_X{i,j}=path_X;
       end
    end   
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                     计算 起始点 到 各个目标点 的最优 顺序
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[lu,L_best,L_ave] = ACATSP(X,Distanse_xx);
num_inx=size(lu,2);
inj=0;

for i=1:num_inx
    if lu(1,i)==1
        inx_lu=i;
    end
end

for j=inx_lu:1:num_inx
    inj=inj+1;
    inx_X(inj)=lu(1,j);
    
end

  for j=1:1:inx_lu
    inj=inj+1;
    inx_X(inj)=lu(1,j);
  end
Sequence=[0 0];
num_sun = size(inx_X,2);
for i=1:(num_sun-1)
    Sequence(i,1) = inx_X(i) ;
    Sequence(i,2) = inx_X(i+1) ;
end
 n=size(inx_X,2);
  for i=1:n
      inx=inx_X(1,i);
      Tartx(i,1)=X(inx,1);
      Tartx(i,2)=X(inx,2); 
  end
Tartx

num_Se=size(Sequence,1);
Path_goal=cell(num_Se,1);
Line_path=[ ];
for i=1:num_Se
    pathth=Pa_X{Sequence(i,1),Sequence(i,2)}; 
    Path_goal{i,1} = [pathth;Tartx(i+1,:)];
    Line_path=[Line_path;pathth];
end
Line_path=[Line_path;Start];

plot( Line_path(:,1)+.5, Line_path(:,2)+.5,'b','linewidth',2); 
 
sl=size(Line_path,1);
S_line=0;
for iii=1:(sl-1)
    S=distance(Line_path(iii,1),Line_path(iii,2),Line_path(iii+1,1),Line_path(iii+1,2));
    S_line=S_line+S;
end
Num_open
S_line

%% C Coordinate 节点坐标，由一个N×2的矩阵存储

%% R Route 路线
%=========================================================================
figure
Ro=lu;
C=X;
subplot(1,2,1)     

N=length(Ro);
scatter(C(:,1),C(:,2));

hold on
axis([1 MAX_X+1, 1 MAX_Y+1])
plot([C(Ro(1),1),C(Ro(N),1)],[C(Ro(1),2),C(Ro(N),2)],'g')
grid on
hold on

for ii=2:N

   plot([C(Ro(ii-1),1),C(Ro(ii),1)],[C(Ro(ii-1),2),C(Ro(ii),2)],'g')
   
   hold on

end

title('基于蚁群算法的多目标点优化排序 ')

%%=========================================================================
subplot(1,2,2)                  

plot(L_best)
grid on
hold on                         

plot(L_ave,'r')

title('平均距离和最短距离')     %标题



 
 
 