function  n_index = Optimal_index(Optimal,xval,yval)
    %This function returns the index of the location of a node in the list
    %OPEN
    %
    %   Copyright 2009-2010 The MathWorks, Inc.
    i=1;
    while(Optimal(i,1) ~= xval || Optimal(i,2) ~= yval )  %%%��OPEN��һ�����ݿ�ʼ������ȷ����ǰ������һ��
        i=i+1;
    end;
    n_index=i;     %%%����ǰ���OPEN��������� ��ֵ��ȥ
end