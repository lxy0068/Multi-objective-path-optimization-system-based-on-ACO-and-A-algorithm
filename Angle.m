function  angle=Angle(x,y)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% ��Ϊ1����Ϊ2����Ϊ3����Ϊ4���JΪ5���KΪ6���LΪ7���IΪ8    %%%
%%%       �������н�Ϊ0�㣬45�㣬-45�㣬90�㣬-90��          %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

angle=[];
z=0;
 m=[x y];                
if (x>0 )          
     n=[1 0];             
     if (y > 0 )    
      i=1;z=5;
     elseif (y < 0)
      i=-1;z=6;
     elseif (y==0)
      i=1;z=2;
     end;
elseif (x<0)
      n=[-1 0];
      if (y>0 )
      i=-1;z=8;
      elseif (y < 0)
      i=1;z=7;
      elseif (y==0)
      i=-1; z=4;  
      end;
elseif (x==0)
    n=[1 0];
    if (y>0)
     i=1;z=1;
    else
     i=-1;z=3;
    end;
    
 end;
    
    c=dot(n,m)/norm(n,2)/norm(m,2);%%%��cosֵ
    angle(1,1)=rad2deg(i*acos(c)) ; %%%��Ƕ�
    angle(1,2)=z;
    %angle=angle_a*pi/180;%%%�󻡶�
   