clc ;
clear;
figure 
% ������Ⱥ+A��Ŀ���·���滮
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                ��ͼ��ģ
MAX0=MAX();
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%ֻ�����������ξ����к�����ȣ�������תʱ����ִ���

%%% ͨ������Ϊ 0 ���ϰ�������Ϊ 1 ����ʼ������Ϊ 2 ��Ŀ�������Ϊ -1 ��
MAX=rot90(MAX0,3);      %%%����0,1�ڷŵ�ͼ�����������鲻һ������Ҫ����ʱ����ת90*3=270�ȸ����飬����������ͼ������Լ����ŵ�ͼ��
MAX_X=size(MAX,2);                                %%%  ��ȡ��������x�᳤��
MAX_Y=size(MAX,1);                                %%%  ��ȡ��������y�᳤��
MAX_VAL=10;                              %%%   ������������ɵ��ַ����ʽ������ֵ�����Ǻ������ڽ���ֵ�ַ���ת��Ϊ��ֵ


x_val = 1;
y_val = 1;
axis([1 MAX_X+1, 1 MAX_Y+1])                %%%  ����x��y��������
set(gca,'xtick',1:1:MAX_X+1,'ytick',1:1:MAX_Y+1,'GridLineStyle','-',... 
    'xGrid','on','yGrid','on')
grid on;                                   %%%  �ڻ�ͼ��ʱ�����������
hold on;                                   %%%  ��ǰ�ἰͼ�񱣳ֶ�����ˢ�£�׼�����ܴ˺󽫻��Ƶ�ͼ�Σ���ͼ����
n=0;%Number of Obstacles                   %%%  �ϰ�������


k=1;          %%%% �������ϰ�����ڹر��б��У��ϰ����ֵΪ1;������ʾ�ϰ���
CLOSED=[];
for j=1:MAX_X
    for i=1:MAX_Y
        if (MAX(i,j)==1)
          %%plot(i+.5,j+.5,'ks','MarkerFaceColor','b'); ԭ���Ǻ��Բ��ʾ
          fill([i,i+1,i+1,i],[j,j,j+1,j+1],'k');  %%%�ĳ� �úڷ�������ʾ�ϰ���
          CLOSED(k,1)=i;  %%% ���ϰ��㱣�浽CLOSE������
          CLOSED(k,2)=j; 
          k=k+1;
        end
    end
end
Area_MAX(1,1)=MAX_X;
Area_MAX(1,2)=MAX_Y;
Obs_Closed=CLOSED;
num_Closed=size(CLOSED,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                       ������ʼ��Ͷ��Ŀ���
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%   ѡ����ʼλ��
h=msgbox('��ʹ��������ѡ������ʼλ��');                    %%%ԭ�� Please Select the Vehicle initial position using the Left Mouse button
uiwait(h,5);
if ishandle(h) == 1
    delete(h);
end
xlabel('��ѡ������ʼλ�� ','Color','red');                %%% ԭ�� Please Select the Vehicle initial position
but=0;
while (but ~= 1) %Repeat until the Left button is not clicked %%%�ظ���ֱ��û�е��������󡱰�ť
    [xval,yval,but]=ginput(1);
    xval=floor(xval);
    yval=floor(yval);
end
xStart=xval;%Starting Position
yStart=yval;%Starting Position

Start(1,1)=xStart;
Start(1,2)=yStart;

                                               %%%   ��ʼ��λ�õ�ֵ����Ϊ1��Ŀ���Ϊ0���ϰ���Ϊ-1������հ׵�Ϊ2
plot(xStart+.5,yStart+.5,'b^');
text(xStart+1,yStart+1.5,'S','fontsize',12')

pause(2);
h=msgbox('��ʹ��������ѡ���Ŀ��ص�,�Ҽ�ѡ�����һ��Ŀ��ص�');
  xlabel('��ʹ��������ѡ���Ŀ��ص�,�Ҽ�ѡ�����һ��Ŀ��ص�','Color','blue');
uiwait(h,10);
if ishandle(h) == 1
    delete(h);
end
while but == 1
    [xval,yval,but] = ginput(1);
    xval=floor(xval);
    yval=floor(yval);
    MAX(xval,yval)=-1;%Put on the closed list as well
    plot(xval+.5,yval+.5,'bo');
   
 end%End of While loop
 

gg=1;%Dummy counter
for i=1:MAX_X
    for j=1:MAX_Y
      if i <= size(MAX, 1) && j <= size(MAX, 2)
        if(MAX(i,j) == -1)
            Goal(gg,1)=i; 
            Goal(gg,2)=j; 
            text(i+1,j+1.5,num2str(gg,'T(%d)'),'fontsize',12')
            gg=gg+1;
        end
      end
    end
end
Goal_COUNT=size(Goal,1);

xlabel('��ʼ��λ�ñ��Ϊ �� ��Ŀ���λ�ñ��Ϊ o ','Color','red'); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                     ������ʼ��Ŀ��� ������ϵ�·���ڵ� �� ����
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

X=[Start;Goal]; %��  ��ʼ��Ͷ��Ŀ���  �����һ��
X_num=Goal_COUNT+1;
% A_X=X_num*Goal_COUNT;%%%������ϣ�A_X �� ��ʼ��Ŀ���·���滮
Distanse_xx=zeros(X_num,X_num);  %%% ��������ÿ��·������

Pa_X=cell(X_num,X_num);          %%% ��������ÿ��·���ڵ� 
Num_open=0;
for i=1:1:X_num
    for j=1:1:X_num
       if i~=j
        St(1,1) = X(i,1);   St(1,2) = X(i,2);
        Ta(1,1) = X(j,1);   Ta(1,2) = X(j,2);
        [path_X,distance_x,OPEN_num]=Astar_s(Obs_Closed,St,Ta,MAX_X,MAX_Y);
        Distanse_xx(i,j)=distance_x;
        Num_open=Num_open+OPEN_num;
        Pa_X{i,j}=path_X;
       end
    end   
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                     ���� ��ʼ�� �� ����Ŀ��� ������ ˳��
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[lu,L_best,L_ave] = ACATSP(X,Distanse_xx);
num_inx=size(lu,2);
inj=0;

for i=1:num_inx
    if lu(1,i)==1
        inx_lu=i;
    end
end

for j=inx_lu:1:num_inx
    inj=inj+1;
    inx_X(inj)=lu(1,j);
    
end

  for j=1:1:inx_lu
    inj=inj+1;
    inx_X(inj)=lu(1,j);
  end
Sequence=[0 0];
num_sun = size(inx_X,2);
for i=1:(num_sun-1)
    Sequence(i,1) = inx_X(i) ;
    Sequence(i,2) = inx_X(i+1) ;
end
 n=size(inx_X,2);
  for i=1:n
      inx=inx_X(1,i);
      Tartx(i,1)=X(inx,1);
      Tartx(i,2)=X(inx,2); 
  end
Tartx

num_Se=size(Sequence,1);
Path_goal=cell(num_Se,1);
Line_path=[ ];
for i=1:num_Se
    pathth=Pa_X{Sequence(i,1),Sequence(i,2)}; 
    Path_goal{i,1} = [pathth;Tartx(i+1,:)];
    Line_path=[Line_path;pathth];
end
Line_path=[Line_path;Start];

plot( Line_path(:,1)+.5, Line_path(:,2)+.5,'b','linewidth',2); 
 
sl=size(Line_path,1);
S_line=0;
for iii=1:(sl-1)
    S=distance(Line_path(iii,1),Line_path(iii,2),Line_path(iii+1,1),Line_path(iii+1,2));
    S_line=S_line+S;
end
Num_open
S_line

%% C Coordinate �ڵ����꣬��һ��N��2�ľ���洢

%% R Route ·��
%=========================================================================
figure
Ro=lu;
C=X;
subplot(1,2,1)     

N=length(Ro);
scatter(C(:,1),C(:,2));

hold on
axis([1 MAX_X+1, 1 MAX_Y+1])
plot([C(Ro(1),1),C(Ro(N),1)],[C(Ro(1),2),C(Ro(N),2)],'g')
grid on
hold on

for ii=2:N

   plot([C(Ro(ii-1),1),C(Ro(ii),1)],[C(Ro(ii-1),2),C(Ro(ii),2)],'g')
   
   hold on

end

title('������Ⱥ�㷨�Ķ�Ŀ����Ż����� ')

%%=========================================================================
subplot(1,2,2)                  

plot(L_best)
grid on
hold on                         

plot(L_ave,'r')

title('ƽ���������̾���')     %����



 
 
 