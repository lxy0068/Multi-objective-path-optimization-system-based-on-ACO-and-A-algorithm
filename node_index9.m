function n_index = node_index9(OPEN,xval,yval)
    %This function returns the index of the location of a node in the list
    %OPEN
    %
    %   Copyright 2009-2010 The MathWorks, Inc.
    i=1;
    while(OPEN(i,4) ~= xval || OPEN(i,5) ~= yval )  %%%��OPEN��һ�����ݿ�ʼ������ȷ����ǰ������һ��
        i=i+1;
    end;
    n_index=i;     %%%����ǰ���OPEN��������� ��ֵ��ȥ
end