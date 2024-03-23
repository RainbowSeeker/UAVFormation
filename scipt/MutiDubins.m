%基于Dubins算法的协同集结策略
%输入量为起点(n*3)、终点(n*3)【n为飞行器数量，列包含点位的X坐标、Y坐标和航向角】、速度(m/s)(n*1/1*n)、最大滚转角(deg)
%输出量为协同航迹矩阵(n*5*5)【n为飞行器数量，列包含点位X坐标、Y坐标、航向角、转弯半径和路段长度】
%5个航点分别为起点、起始圆切点、终止圆切点、（中间点）、终点【如果是参考飞行器的航点则中间点就是终点】
function result =  MutiDubins(Start, Goal, Velocity, PhiMaximum) 
    rad2deg = 57.295779513082320876798154814105;
    %初始化
    obj=struct('xs',0, ...%起点x坐标
                  'ys',0, ...%起点y坐标
                  'psi_s',0, ...%起始航向角
                  'xf',0, ...%终点x坐标
                  'yf',0, ...%终点y坐标
                  'psi_f',0, ...%终止航向角
                  'v',0, ...%初始速度
                  'r',0, ...%转弯半径
                  'pos',zeros(2,2), ...%起始圆圆心坐标
                  'pof',zeros(2,2), ...%终止圆圆心坐标
                  'xts',zeros(2,2), ...%起始圆切点x坐标
                  'yts',zeros(2,2), ...%起始圆切点y坐标
                  'xtf',zeros(2,2), ...%终止圆切点x坐标
                  'ytf',zeros(2,2), ...%终止圆切点y坐标
                  'cs',zeros(2,2),...%起始圆弧段
                  'cf',zeros(2,2), ...%终止圆弧段
                  'lt',zeros(2,2), ...%切线段
                  'l',zeros(2,2), ...%路径长度
                  'index_dubins',zeros(1,2), ...%最短Dubins路径索引
                  'l_ad',0, ...%延长直线段长度
                  'precision_flag',0, ...%精度
                  'xm',0, ...%中间点x坐标
                  'ym',0 ...%中间点y坐标
                   );
    
    object = [];
    for i=1:size(Start,1)
        object = [object obj];
        object(i).xs=Start(i,1);
        object(i).ys=Start(i,2);
        object(i).psi_s=Start(i,3);
        object(i).xf=Goal(i,1);
        object(i).yf=Goal(i,2);
        object(i).psi_f=Goal(i,3);
        object(i).v=Velocity(i);
    end
    
    %参考路径求解
    target=-1;
    l_ref=-1;
    for i=1:length(object)
        [object(i).xts,object(i).yts,object(i).xtf,object(i).ytf,object(i).cs,object(i).cf,object(i).lt,object(i).l,object(i).pos,object(i).pof,object(i).r,object(i).index_dubins(1),object(i).index_dubins(2)] ...
        =Dubins(object(i).xs,object(i).ys,object(i).psi_s,object(i).xf,object(i).yf,object(i).psi_f,object(i).v);
        if(object(i).l(object(i).index_dubins(1),object(i).index_dubins(2))>l_ref)
            l_ref=object(i).l(object(i).index_dubins(1),object(i).index_dubins(2));
            target=i;
        end
    end
    %路径调整
                   %1    2   3  4 5   6    7     8
    precision_base=[1000,100,10,1,0.1,0.01,0.001,0.0001];%精度库
    precision=7;%精度库索引
    for i=1:length(object)
        if(i~=target)
            search_floor=0;%搜索下限
            search=0;%当前搜索对象
            stop_flag=1;%求解完成
            for j=1:precision
                while(stop_flag)
                    search=search+precision_base(j);
                    object(i).xm=object(i).xf-search*sin(object(i).psi_f/rad2deg);
                    object(i).ym=object(i).yf-search*cos(object(i).psi_f/rad2deg);
                    [object(i).xts,object(i).yts,object(i).xtf,object(i).ytf,object(i).cs,object(i).cf,object(i).lt,object(i).l,object(i).pos,object(i).pof,~,object(i).index_dubins(1),object(i).index_dubins(2)] ...
                        =Dubins(object(i).xs,object(i).ys,object(i).psi_s,object(i).xm,object(i).ym,object(i).psi_f,object(i).v);
                    goal=object(i).l(object(i).index_dubins(1),object(i).index_dubins(2))+search-l_ref;
                    if(goal>0)
                        object(i).l_ad=search_floor;
                        object(i).precision_flag=precision_base(j);
                        search=search_floor;
                        break;
                    elseif(goal<0)
                        search_floor=search;%下限前移
                    else
                        object(i).l_ad=search;
                        object(i).precision_flag=0;%精度准确
                        stop_flag=0;
                    end
                end
            end
        else
            object(i).xm=object(i).xf;
            object(i).ym=object(i).yf;
        end
    end
    
    %结果输出
    result=zeros(length(object),5,5);
    for i=1:size(result,1)
        if(i==target)
            result(i,1,:)=[object(i).xs,object(i).ys,object(i).psi_s,object(i).r,object(i).cs(object(i).index_dubins(1),object(i).index_dubins(2))];
            result(i,2,:)=[object(i).xts(object(i).index_dubins(1),object(i).index_dubins(2)),object(i).yts(object(i).index_dubins(1),object(i).index_dubins(2)),deltapsi(object(i).xtf(object(i).index_dubins(1),object(i).index_dubins(2)),object(i).ytf(object(i).index_dubins(1),object(i).index_dubins(2)),object(i).xts(object(i).index_dubins(1),object(i).index_dubins(2)),object(i).yts(object(i).index_dubins(1),object(i).index_dubins(2)))*rad2deg,0,object(i).lt(object(i).index_dubins(1),object(i).index_dubins(2))];
            result(i,3,:)=[object(i).xtf(object(i).index_dubins(1),object(i).index_dubins(2)),object(i).ytf(object(i).index_dubins(1),object(i).index_dubins(2)),deltapsi(object(i).xtf(object(i).index_dubins(1),object(i).index_dubins(2)),object(i).ytf(object(i).index_dubins(1),object(i).index_dubins(2)),object(i).xts(object(i).index_dubins(1),object(i).index_dubins(2)),object(i).yts(object(i).index_dubins(1),object(i).index_dubins(2)))*rad2deg,object(i).r,object(i).cf(object(i).index_dubins(1),object(i).index_dubins(2))];
            result(i,4,:)=[object(i).xm,object(i).ym,object(i).psi_f,0,0];
            result(i,5,:)=[object(i).xf,object(i).yf,object(i).psi_f,0,object(i).l(object(i).index_dubins(1),object(i).index_dubins(2))];
        else
            result(i,1,:)=[object(i).xs,object(i).ys,object(i).psi_s,object(i).r,object(i).cs(object(i).index_dubins(1),object(i).index_dubins(2))];
            result(i,2,:)=[object(i).xts(object(i).index_dubins(1),object(i).index_dubins(2)),object(i).yts(object(i).index_dubins(1),object(i).index_dubins(2)),deltapsi(object(i).xtf(object(i).index_dubins(1),object(i).index_dubins(2)),object(i).ytf(object(i).index_dubins(1),object(i).index_dubins(2)),object(i).xts(object(i).index_dubins(1),object(i).index_dubins(2)),object(i).yts(object(i).index_dubins(1),object(i).index_dubins(2)))*rad2deg,0,object(i).lt(object(i).index_dubins(1),object(i).index_dubins(2))];
            result(i,3,:)=[object(i).xtf(object(i).index_dubins(1),object(i).index_dubins(2)),object(i).ytf(object(i).index_dubins(1),object(i).index_dubins(2)),deltapsi(object(i).xtf(object(i).index_dubins(1),object(i).index_dubins(2)),object(i).ytf(object(i).index_dubins(1),object(i).index_dubins(2)),object(i).xts(object(i).index_dubins(1),object(i).index_dubins(2)),object(i).yts(object(i).index_dubins(1),object(i).index_dubins(2)))*rad2deg,object(i).r,object(i).cf(object(i).index_dubins(1),object(i).index_dubins(2))];
            result(i,4,:)=[object(i).xm,object(i).ym,object(i).psi_f,0,object(i).l_ad];
            result(i,5,:)=[object(i).xf,object(i).yf,object(i).psi_f,0,object(i).l(object(i).index_dubins(1),object(i).index_dubins(2))+object(i).l_ad];
        end
    end
    
    %atan求以X轴正方向逆时针到某向量的夹角
    function result=atan3(x,y,x0,y0)%rad
        deltax=x-x0;
        deltay=y-y0;
        if(deltax>0&&deltay>0)
            result=atan(deltay/deltax);
        elseif (deltax>0&&deltay<0)
            result=2*pi+atan(deltay/deltax);
        elseif (deltax<0)
            result=pi+atan(deltay/deltax);
        elseif (deltax==0)
            if (deltay>0)
                result=pi/2;
            else
                result=pi*3/2;
            end
        else
            if(deltax>0)
                result=0;
            else
                result=pi;
            end
        end
    end

    %atan求以y轴正方向顺时针到某向量的夹角/航向角
    function delta_psi=deltapsi(x,y,x0,y0)%rad
        deltax=x-x0;
        deltay=y-y0;
        if(deltax>0&&deltay>0)
            delta_psi=atan(deltax/deltay);
        elseif (deltax<0&&deltay>0)
            delta_psi=2*pi+atan(deltax/deltay);
        elseif (deltay<0)
            delta_psi=pi+atan(deltax/deltay);
        elseif (deltax==0)
            if (deltay>0)
                delta_psi=0;
            else
                delta_psi=pi;
            end
        else
            if(deltax>0)
                delta_psi=pi/2;
            else
                delta_psi=pi*3/2;
            end
        end
    end

    %单机Dubins
    function [xts,yts,xtf,ytf,cs,cf,lt,l,pos,pof,r,m,n]=Dubins(xs,ys,psi_s,xf,yf,psi_f,v)
        %基本参数
        g=9.81;
        r=v*v/g/tan(PhiMaximum/rad2deg);
        
        %起始圆弧段
        pos=zeros(2,2);%xos1,yos1;xos2,yos2
        pos(1,1)=xs+r*cos(psi_s/rad2deg);
        pos(1,2)=ys-r*sin(psi_s/rad2deg);
        pos(2,1)=xs-r*cos(psi_s/rad2deg);
        pos(2,2)=ys+r*sin(psi_s/rad2deg);
        direction_s=[1,-1];%右为正
        
        %终止圆弧段
        pof=zeros(2,2);%xof1,yof1;xof2,yof2
        pof(1,1)=xf+r*cos(psi_f/rad2deg);
        pof(1,2)=yf-r*sin(psi_f/rad2deg);
        pof(2,1)=xf-r*cos(psi_f/rad2deg);
        pof(2,2)=yf+r*sin(psi_f/rad2deg);
        direction_f=[1,-1];%右为正
        
        %直线段
        alpha=zeros(2,2);%rad
        beta=zeros(2,2);%rad
        d=zeros(2,2);%圆心距
        theta=zeros(2,2);%rad
        xts=zeros(2,2);%起始圆切点横坐标
        yts=zeros(2,2);%起始圆切点纵坐标
        xtf=zeros(2,2);%终止圆切点横坐标
        ytf=zeros(2,2);%终止圆切点纵坐标
        cs=zeros(2,2);%起始圆弧长度
        cf=zeros(2,2);%终止圆弧长度
        lt=zeros(2,2);%切线长度
        l=zeros(2,2);%路径长度
        
        for i1=1:2
            for j1=1:2
                alpha(i1,j1)=atan3(pof(j1),pof(j1+2),pos(i1),pos(i1+2));
                d(i1,j1)=sqrt((pof(j1+2)-pos(i1+2))*(pof(j1+2)-pos(i1+2))+(pof(j1)-pos(i1))*(pof(j1)-pos(i1)));
                if(direction_s(i1)*direction_f(j1)>0)  
                    beta(i1,j1)=0;
                    theta(i1,j1)=alpha(i1,j1);
                    if(direction_s(i1)==1&&direction_f(j1)==1)
                        xts(i1,j1)=pos(i1)-r*sin(theta(i1,j1));
                        yts(i1,j1)=pos(i1+2)+r*cos(theta(i1,j1));
                        xtf(i1,j1)=pof(j1)-r*sin(theta(i1,j1));
                        ytf(i1,j1)=pof(j1+2)+r*cos(theta(i1,j1));
                    else
                        xts(i1,j1)=pos(i1)+r*sin(theta(i1,j1));
                        yts(i1,j1)=pos(i1+2)-r*cos(theta(i1,j1));
                        xtf(i1,j1)=pof(j1)+r*sin(theta(i1,j1));
                        ytf(i1,j1)=pof(j1+2)-r*cos(theta(i1,j1));
                    end
                else
                    beta(i1,j1)=acos(r*2/d(i1,j1));
                    if(direction_s(i1)==1&&direction_f(j1)==-1)
                        theta(i1,j1)=alpha(i1,j1)+beta(i1,j1);
                        xts(i1,j1)=pos(i1)+r*cos(theta(i1,j1));
                        yts(i1,j1)=pos(i1+2)+r*sin(theta(i1,j1));
                        xtf(i1,j1)=pof(j1)-r*cos(theta(i1,j1));
                        ytf(i1,j1)=pof(j1+2)-r*sin(theta(i1,j1));
                    else
                        theta(i1,j1)=alpha(i1,j1)-beta(i1,j1);
                        xts(i1,j1)=pos(i1)+r*cos(theta(i1,j1));
                        yts(i1,j1)=pos(i1+2)+r*sin(theta(i1,j1));
                        xtf(i1,j1)=pof(j1)-r*cos(theta(i1,j1));
                        ytf(i1,j1)=pof(j1+2)-r*sin(theta(i1,j1));
                     end
                end  
                cs(i1,j1)=r*2*asin(sqrt((xs-xts(i1,j1))*(xs-xts(i1,j1))+(ys-yts(i1,j1))*(ys-yts(i1,j1)))/2/r);
                cf(i1,j1)=r*2*asin(sqrt((xf-xtf(i1,j1))*(xf-xtf(i1,j1))+(yf-ytf(i1,j1))*(yf-ytf(i1,j1)))/2/r);
                lt(i1,j1)=sqrt((xtf(i1,j1)-xts(i1,j1))*(xtf(i1,j1)-xts(i1,j1))+(ytf(i1,j1)-yts(i1,j1))*(ytf(i1,j1)-yts(i1,j1)));
                l(i1,j1)=cs(i1,j1)+cf(i1,j1)+lt(i1,j1);
            end
        end
        %校验dubins路径
        for i1=1:2
            for j1=1:2
                if(imag(l(i1,j1))~=0)
                    l(i1,j1)=-1;%校验路径长度是否有虚部
                end
            end
        end
        l_best=inf;
        for i1=1:4
            if(l(i1)>=0&&l(i1)<l_best)
                l_best=l(i1);
            end
        end
        [m,n]=find(l==l_best);
        if(length(m)~=1||length(n)~=1)%路径长度相同时取首个
            m=m(1);
            n=n(1);
        end
    end
end