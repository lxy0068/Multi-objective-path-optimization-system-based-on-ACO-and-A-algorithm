function ang=sn_angle(x_1,y_1,x_2,y_2)
x=x_2-x_1 ;
y=y_2-y_1;
n=[1 0];
m=[x y];
c=dot(n,m)/norm(n,2)/norm(m,2) ;
if  y<0
    i=-1 ;
else
    i=1 ;
end
du=i*rad2deg(acos(c)) ;
 
ang=du/180*pi ;