function dist = distance(x1,y1,x2,y2)
%This function calculates the distance between any two cartesian 
%coordinates.
%�˺����������������ѿ�������֮���ŷ����þ��롣
%   Copyright 2009-2010 The MathWorks, Inc.
dist=sqrt((x1-x2)^2 + (y1-y2)^2);
%  ���� �����پ��� h(n) = abs(nx - goalx) + abs(ny - goaly)
%       �б�ѩ����� h(n) = max[abs(nx - goalx),abs(ny - goaly)