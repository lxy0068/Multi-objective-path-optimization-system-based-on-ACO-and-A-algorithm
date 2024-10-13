function [Path,distanceX,OPEN_num]= Astar_s(Obs_Closed,St,Ta,MAX_X,MAX_Y)

CLOSED = Obs_Closed;
xStart = St(1,1);      yStart = St(1,2); 
xTarget = Ta(1,1);    yTarget = Ta(1,2);

OPEN=[];
%CLOSED LIST STRUCTURE %%% ��յ��б�ṹ
Num_obs=size(Obs_Closed,1);
CLOSED_COUNT=size(CLOSED,1);   %%% CLOSED�����������ϰ���ĸ��� 
Nobs=CLOSED_COUNT;

xNode=xStart;      %%% =xStart
yNode=yStart;      %%% =yStart
OPEN_COUNT=1;    %%% OPEN_COUNT �����б��������־
path_cost=0;
goal_distance=distance(xNode,yNode,xTarget,yTarget);                                                                                    %%%  ***����distance�������������������֮��ĵѿ�������
% Sgoal=goal_distance;
OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,path_cost,goal_distance,goal_distance);  %%%   ���뵽�����б�
                            %%%        OPEN����һ�е�Ԫ�أ�=��1��xNode,yNode,xNode,yNode,path_cost,goal_distance,goal_distanc����
OPEN(OPEN_COUNT,1)=0;      %%%   OPEN(1,1)=0
CLOSED_COUNT=CLOSED_COUNT+1;  %%%   CLOSED �洢���ϰ������һ����Ԫ
CLOSED(CLOSED_COUNT,1)=xNode; %%%   ��һ���洢��ʼ��� ����
CLOSED(CLOSED_COUNT,2)=yNode;
NoPath=1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM ��ʼ�㷨
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while((xNode ~= xTarget || yNode ~= yTarget) && NoPath == 1)       %%%  �жϵ�ǰ���Ƿ����Ŀ���

 exp_array=expand_array01(xNode,yNode,path_cost,xTarget,yTarget,CLOSED,MAX_X,MAX_Y);  %%% ���ڹر��б���ӽڵ㣬��x,y,gn,hn,fn��,�����Ǹ���
 exp_count=size(exp_array,1);  %%%  ��ѡ����ӽڵ����
 
 for i=1:exp_count         %%% ��exp_array�ڵ�Ԫ����ӵ� �����б� ����
    flag=0;                %%% ��exp_array�ڵĵ�ı�־λ��Ϊ0
    for j=1:OPEN_COUNT         %%% OPEN_COUNT ��1��ʼ���Լ�
        if(exp_array(i,1) == OPEN(j,2) && exp_array(i,2) == OPEN(j,3) )    %%%�жϿ�ѡ�ӽڵ��Ƿ���OPEN[]�еĵ���ͬ
            OPEN(j,8)=min(OPEN(j,8),exp_array(i,5));                       %%%�����ͬ���Ƚ�����fn��ֵ�Ĵ�С������fnС������㸳ֵ��OPEN(j,8)
            if OPEN(j,8)== exp_array(i,5)                                  %%% ��ʾ����һ���Ƚ��� exp_array(i,5)С�����exp_array(i,�����е�ֵ����OPEN
                %UPDATE PARENTS,gn,hn
                OPEN(j,4)=xNode;
                OPEN(j,5)=yNode;
                OPEN(j,6)=exp_array(i,3);
                OPEN(j,7)=exp_array(i,4);
            end;%End of minimum fn check
            flag=1;                    %%%����ϣУţ���ͬ�ģ���磽1
        end;%End of node check
%         if flag == 1
%             break;
    end;%End of j for
    if flag == 0
        OPEN_COUNT = OPEN_COUNT+1; 
        OPEN(OPEN_COUNT,:)=insert_open(exp_array(i,1),exp_array(i,2),xNode,yNode,exp_array(i,3),exp_array(i,4),exp_array(i,5));
     end;%End of insert new element into the OPEN list
 end;%End of i for
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %END OF WHILE LOOP
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %Find out the node with the smallest fn  �ҳ�fn��С�Ľڵ�
  index_min_node = min_fn(OPEN,OPEN_COUNT,xTarget,yTarget);   %%%ѡ��fn��С��һ�У����������� index_min_node
  if (index_min_node ~= -1)    
   %Set xNode and yNode to the node with minimum fn ��xNode��yNode����Ϊ��Сfn�Ľڵ�
   xNode=OPEN(index_min_node,2);
   yNode=OPEN(index_min_node,3);
   path_cost=OPEN(index_min_node,6);% Update the cost of reaching the parent node ���µ��︸�ڵ�ĳɱ�  gn
  
%    p_xNode=OPEN(index_min_node,4);
%    p_yNode=OPEN(index_min_node,5);
   %Move the Node to list CLOSED   ���ڵ��ƶ����б�CLOSED  
  CLOSED_COUNT=CLOSED_COUNT+1;
  CLOSED(CLOSED_COUNT,1)=xNode;
  CLOSED(CLOSED_COUNT,2)=yNode;
  OPEN(index_min_node,1)=0;
%   CLOSED  %%%****���CLOSE[]������ѧϰ�˽�A*�㷨���������****  ///����Ҫ֪�����̿�ע�͵�///
%   OPEN    %%%****���OPEN[]������ѧϰ�˽�A*�㷨���������****  ///����Ҫ֪�����̿�ע�͵�///
  else
      %No path exists to the Target!!
      NoPath=0;%Exits the loop!
  end;%End of index_min_node check
end;%End of While Loop


i=size(CLOSED,1);    %%%CLOSE����ĳ���
Optimal_path=[];     %%%·������
xval=CLOSED(i,1);    %%%��CLOSE���һ��������������һ����ΪĿ���
yval=CLOSED(i,2);
i=1;
Optimal_path(i,1)=xval; %%%��Ŀ�������긳�� ·������� ��һ��
Optimal_path(i,2)=yval;
            

if ( (xval == xTarget) && (yval == yTarget))  %%%���CLOSE���һ���Ƿ�ΪĿ���
 
   %Traverse OPEN and determine the parent nodes ����OPEN��ȷ�����ڵ�
   Target_ind=node_index(OPEN,xval,yval);
   parent_x=OPEN(Target_ind,4); %node_index returns the index of the node  node_index���ؽڵ������
   parent_y=OPEN(Target_ind,5);%%% ����ǰ��ĸ��ڵ������
   
   
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 while( parent_x ~= xStart || parent_y ~= yStart)   %%% �жϸ��ڵ��Ƿ�Ϊ��ʼ��
           i=i+1; 
           Optimal_path(i,1) = parent_x;             %%% ���� �򽫸��ڵ��͸�·������
           Optimal_path(i,2) = parent_y;
           inode=node_index(OPEN,parent_x,parent_y); 
           parent_x= OPEN(inode,4);
           parent_y= OPEN(inode,5);
           
 end;
  
  j = size(Optimal_path,1) + 1;
  Optimal_path(j,1) = xStart;           
  Optimal_path(j,2) = yStart; %%%����ʼ��ӽ�ȥ
  %plot(Optimal_path(:,1)+.5,Optimal_path(:,2)+.5,'linewidth',2); %5%����
  i1=1;
  patn=[ ];
  for i2=j:-1:1
      patn(i1,1)=Optimal_path(i2,1);
      patn(i1,2)=Optimal_path(i2,2);
      i1=i1+1;
  end
  
  
 %%%%%%%%% ����ֱ��·���ĳ���
    S_line=0;
    for i=1:(j-1)
    s1 = distance(Optimal_path(i,1),Optimal_path(i,2),Optimal_path(i+1,1),Optimal_path(i+1,2));
    S_line=s1 + S_line;
    end
   Path=patn;
   distanceX=S_line;
   OPEN_num=size(OPEN,1);
   
%  angle_du=0;
%  for i=1:1:(j-2)  %%%% ��·�����õĽǶ�
%      du=angle6(  NewOptimal_path(i,1),NewOptimal_path(i,2),NewOptimal_path(i+1,1) ,NewOptimal_path(i+1,2),NewOptimal_path(i+2,1) ,NewOptimal_path(i+2,2));
%      angle_du=angle_du+du;
%      
%  end
%  angle_du
%  S
%  T= angle_du/45 + S + zhuan_num

%   p=plot(Optimal_path(j,1)+.5,Optimal_path(j,2)+.5,'bo'); %%
%  j=j-1;
%  for i=j:-1:1
%   pause(.25);
%   set(p,'XData',Optimal_path(i,1)+.5,'YData',Optimal_path(i,2)+.5);
%  drawnow ;
%  end;
 
% plot(Newopt(:,1)+.5,Newopt(:,2)+.5,'linewidth',2); %5%����
 
 
else
 pause(1);
 h=msgbox('Sorry, No path exists to the Target!','warn');
 uiwait(h,5);
end

















