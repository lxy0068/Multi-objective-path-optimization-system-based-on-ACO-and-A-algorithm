function n_index = path_node_index(path,xval,yval)
   

    i=1;
    while(path(i,1) ~= xval || path(i,2) ~= yval )  %%%��OPEN��һ�����ݿ�ʼ������ȷ����ǰ������һ��
        i=i+1;
    end;
    n_index=i;     %%%����ǰ���OPEN��������� ��ֵ��ȥ
end