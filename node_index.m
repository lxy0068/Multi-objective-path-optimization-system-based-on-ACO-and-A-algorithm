function n_index = node_index(OPEN,xval,yval)
    %This function returns the index of the location of a node in the list
    %OPEN
    %
    %   Copyright 2009-2010 The MathWorks, Inc.
    i=1;
    while(OPEN(i,2) ~= xval || OPEN(i,3) ~= yval )  %%%��OPEN��һ�����ݿ�ʼ������ȷ����ǰ������һ��
        i=i+1;
    end;
    n_index=i;     %%%����ǰ���OPEN��������� ��ֵ��ȥ
end