function  angle=Angle(x,y)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% ↑为1，→为2，↓为3，←为4；↗为5，↘为6，↙为7，↖为8    %%%
%%%       两向量夹角为0°，45°，-45°，90°，-90°          %%%
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
    
    c=dot(n,m)/norm(n,2)/norm(m,2);%%%求cos值
    angle(1,1)=rad2deg(i*acos(c)) ; %%%求角度
    angle(1,2)=z;
    %angle=angle_a*pi/180;%%%求弧度
   