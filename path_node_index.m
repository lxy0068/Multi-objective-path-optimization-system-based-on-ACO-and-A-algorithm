function n_index = path_node_index(path,xval,yval)
   

    i=1;
    while(path(i,1) ~= xval || path(i,2) ~= yval )  %%%从OPEN第一组数据开始遍历，确定当前点在哪一行
        i=i+1;
    end;
    n_index=i;     %%%将当前点的OPEN数组的行数 赋值出去
end