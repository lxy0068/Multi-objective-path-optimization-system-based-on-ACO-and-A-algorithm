function     dire=direction(OPEN,node_x,node_y,s_x,s_y)
% node_x,node_y ��ǰ�����꣬s_x,s_y���ӽڵ㣻
i_1= node_index(OPEN,node_x,node_y); %
x_p=OPEN(i_1,4);%��ǰ��ĸ��ڵ�
y_p=OPEN(i_1,5);
du=angle6(x_p,y_p,node_x,node_y,s_x,s_y);
dire=du;

  