function     dire=direction(OPEN,node_x,node_y,s_x,s_y)
% node_x,node_y 当前点坐标，s_x,s_y，子节点；
i_1= node_index(OPEN,node_x,node_y); %
x_p=OPEN(i_1,4);%当前点的父节点
y_p=OPEN(i_1,5);
du=angle6(x_p,y_p,node_x,node_y,s_x,s_y);
dire=du;

  