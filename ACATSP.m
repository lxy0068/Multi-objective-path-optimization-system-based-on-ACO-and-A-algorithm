function [Ro,L_best,L_ave]=ACATSP(C,D)
% ��Ҫ����˵��
% C n�����е����꣬n��2�ľ���
% NC_max ����������
NC_max=500;
% m ���ϸ���
m=100;
% Alpha ������Ϣ����Ҫ�̶ȵĲ���
Alpha=1.5;
% Beta ��������ʽ������Ҫ�̶ȵĲ���
Beta=2;
% Rho ��Ϣ������ϵ��
Rho=0.9;
% Q ��Ϣ������ǿ��ϵ��
Q=10;
% R_best �������·��
% L_best �������·�ߵĳ���
%%=========================================================================
%%��һ����������ʼ��

n=size(C,1);%n��ʾ����Ĺ�ģ�����и�����

Eta=1./D;        

Tau=ones(n,n);    

Tabu=zeros(m,n);   

NC=1;   


R_best=zeros(NC_max,n);      

L_best=inf.*ones(NC_max,1);   

L_ave=zeros(NC_max,1);     


while NC<=NC_max        %ֹͣ����֮һ���ﵽ������������ֹͣ

%%�ڶ�������mֻ���Ϸŵ�n��������

Randpos=[];   %�漴��ȡ

   for i=1:(ceil(m/n))
% Randpos=[1~31 + 1~31]   ��ÿֻ���Ϸŵ�����ĳ���  
% Randpos �����ѡ��n�������������ϵĳ�ʼ����
   Randpos=[Randpos,randperm(n)];
   end

Tabu(:,1)=(Randpos(1,1:m))';    % ��һ�ε���ÿֻ���ϵĽ��ɱ�


%%��������mֻ���ϰ����ʺ���ѡ����һ�����У���ɸ��Ե�����

for j=2:n     %���ڳ��в�����

for i=1:m     

visited=Tabu(i,1:(j-1)); 

J=zeros(1,(n-j+1));      

P=J;                      

Jc=1;

for k=1:n

if length(find(visited==k))==0   %��ʼʱ��0

J(Jc)=k;

Jc=Jc+1;                        
  end

end

%��������ѡ���еĸ��ʷֲ�

for k=1:length(J)

P(k)=(Tau(visited(end),J(k))^Alpha)*(Eta(visited(end),J(k))^Beta);

end

P=P/(sum(P));

%������ԭ��ѡȡ��һ������

Pcum=cumsum(P);    

Select=find(Pcum>=rand); 

if Select(1) <= length(J)
    to_visit = J(Select(1));
else
    % ����������������Χ��������������һ��Ĭ��ֵ�����׳�����
end

% to_visit=J(Select(1)); %ԭ����ֻ����8��Ŀ���
 
Tabu(i,j)=to_visit;

end

end

if NC>=2

Tabu(1,:)=R_best(NC-1,:);

end


%%���Ĳ�����¼���ε������·��

L=zeros(m,1);     %��ʼ����Ϊ0��m*1��������

for i=1:m

R=Tabu(i,:);

for j=1:(n-1)

L(i)=L(i)+D(R(j),R(j+1));  

end

L(i)=L(i)+D(R(1),R(n));    

end

L_best(NC)=min(L);           %��Ѿ���ȡ��С

pos=find(L==L_best(NC));

R_best(NC,:)=Tabu(pos(1),:); 

L_ave(NC)=mean(L);           

NC=NC+1;                      



%%���岽��������Ϣ��

Delta_Tau=zeros(n,n);       

for i=1:m

for j=1:(n-1)

Delta_Tau(Tabu(i,j),Tabu(i,j+1))=Delta_Tau(Tabu(i,j),Tabu(i,j+1))+Q/L(i);           

%�˴�ѭ����·����i��j���ϵ���Ϣ������

end

Delta_Tau(Tabu(i,n),Tabu(i,1))=Delta_Tau(Tabu(i,n),Tabu(i,1))+Q/L(i);

%�˴�ѭ��������·���ϵ���Ϣ������

end

Tau=(1-Rho).*Tau+Delta_Tau; %������Ϣ�ػӷ������º����Ϣ��

%%�����������ɱ�����

Tabu=zeros(m,n);             %%ֱ������������

end

%%���߲���������

Pos=find(L_best==min(L_best)); 

Shortest_Route=R_best(Pos(1),:);

Shortest_Length=L_best(Pos(1)) ;

 Ro=Shortest_Route;




