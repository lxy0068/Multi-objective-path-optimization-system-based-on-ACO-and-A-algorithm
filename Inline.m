function inline=Inline(x1,y1,x3,y3,CLOSED,Num_obs)
 k = (y3-y1)/(x3-x1); b =y1-k*x1; %%两点之间直线方程 y=k*x+b
            x_min=min(x1,x3);  x_max=max(x1,x3);
            y_min=min(y1,y3);  y_max=max(y1,y3);
            f=0;
            for j=1:1:Num_obs
                x_obn=CLOSED(j,1); y_obn=CLOSED(j,2); %%判断障碍点是否在行驶区域内
                if (x_obn>=x_min && x_obn<=x_max && y_obn>=y_min && y_obn<=y_max)
                  yline=x_obn*k+b; D=0.9;
                  d=abs(y_obn-yline)*cos(atan(k));
                  if d<=D
                      f=1;
                  end
                end
            end
            inline=f;