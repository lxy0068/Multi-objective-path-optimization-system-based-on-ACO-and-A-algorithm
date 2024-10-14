function i_min = min_fn(OPEN,OPEN_COUNT,xTarget,yTarget)
%Function to return the Node with minimum fn
% This function takes the list OPEN as its input and returns the index of the
% node that has the least cost
%
%   Copyright 2009-2010 The MathWorks, Inc.

 temp_array=[];
 k=1;
 flag=0;
 goal_index=0;
 for j=1:OPEN_COUNT
     if (OPEN(j,1)==1)
         temp_array(k,:)=[OPEN(j,:) j];             %#ok<*AGROW>
         if (OPEN(j,2)==xTarget && OPEN(j,3)==yTarget)
             flag=1;
             goal_index=j;                         %Store the index of the goal node �洢Ŀ��ڵ������
         end;
         k=k+1;
     end;
 end;                        %Get all nodes that are on the list open ��ȡ�б��е����нڵ�
 if flag == 1               % one of the successors is the goal node so send this node ����һ���������Ŀ��ڵ㣬��˷��ʹ˽ڵ�
     i_min=goal_index;
 end
 %Send the index of the smallest node ������С�ڵ������
 if size(temp_array ~= 0)
%      temp_array  %%%****���temp_array������ѧϰ�˽�A*�㷨���������****  ///����Ҫ֪�����̿�ע�͵�///
     [min_fn,temp_min]=min(temp_array(:,8));%Index of the smallest node in temp array ��ʱ��������С�ڵ������
                                         %%%��Сֵ����min_fn����Сֵ���ڵ���������temp_min��    ����  A=[1 2 3;4 5 6;7 8 9;0 0 0] [a,b]=min(A(:,3)) a=0 b=4;
  i_min=temp_array(temp_min,9);          %Index of the smallest node in the OPEN array OPEN��������С�ڵ������
 else
     i_min=-1;%The temp_array is empty i.e No more paths are available.  temp_array�ǿյģ���û�и����·������
 end;