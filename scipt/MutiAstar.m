%基于A*算法的协同集结策略
%输入量为作战空间长度(m)、作战空间宽度(m)、作战空间左上角坐标(1*2)、起点(n*2)、终点(n*2)【n为飞行器数量，列包含点位的X坐标和Y坐标】、速度(m/s)、最大滚转角(deg)
%输出量为协同航迹(航点+航向)(m*3*n)【m为航点数量最大值，n为飞行器数量，列包含点位X坐标、Y坐标和航向角】
%优点：有避障和防撞策略，路径相对自然；缺点：不能满足起点和终点航向角约束，对于不在网格形心的起点和终点暂无对应策略
function result = MutiAstar(Length, Wide, Origin, Start, Goal, Velocity, PhiMaximum)
    %计算最小转弯半径
    rad2deg = 57.295779513082320876798154814105;
    g=9.80;%m/s^2
    v=Velocity;%m/s
    phi_max=PhiMaximum;%deg
    r=v*v/g/tan(phi_max/rad2deg);%m
    
    %地图初始化
    L=Length;%作战空间长度,m/s
    W=Wide;%作战空间宽度,m/s
    O=Origin;%作战空间左上顶点定位
    unit=5*r;%栅格边长,m
    map=zeros(ceil(W/unit),ceil(L/unit));%生成作战空间
    
    %数据初始化
    object=struct('start',[],           ...%起点
                  'goal',[],            ...%终点
                  'path',[],            ...%路径点
                  'real_path',[],       ...%真实路径点
                  'real_path_smooth',[],...%真实光滑路径点
                  'length',0,           ...%路径代价
                  'strategy',[],        ...%策略表
                  'diagonal',0,         ...%对角线移动次数
                  'off_diagonal',0,     ...%非对角线移动次数
                  'choice',0);             %策略选择
    for i=1:size(Start,1)
        object(i).start=Start(i,:);
        object(i).goal=Goal(i,:);
    end

    %单机A*
    for i=1:length(object)
        object(i).path = astar(map, object(i).start, object(i).goal);
        [object(i).length,object(i).diagonal,object(i).off_diagonal]=getDistandCounts(object(i).path,1,size(object(i).path,1));
    end
    
    %参考路径求解
    l_ref=-1;
    for i=1:length(object)
        if object(i).length>l_ref
            l_ref=object(i).length;
            target=i;
        end
    end
    
    %协同路径生成
    for i=1:length(object)
        if i~=target && object(i).length~=l_ref
            %获取策略表
            strategy = getStrategy(object(i).start, object(i).goal, l_ref);
            object(i).strategy=strategy;
            [accFlag,object(i).path] = mutiAstar(map, object(i).start, object(i).goal, object(target).diagonal, object(target).off_diagonal);
            currentStrategy=1;%当前选择策略一
            if accFlag==1%如果可达
                if i<target
                    if collisionDetect(object(target).path,object(i).path)
                        accFlag=0;%若和协同航迹相撞则不可达
                    else
                        j=1;
                        while j<i
                            if collisionDetect(object(j).path,object(i).path)
                                accFlag=0;%若与其他僚机相撞也不可达
                                break;
                            end
                            j=j+1;
                        end
                    end
                elseif i>target
                    j=1;
                    while j<i
                        if collisionDetect(object(j).path,object(i).path)
                            accFlag=0;
                            break;
                        end
                        j=j+1;
                    end
                end 
            end
            while ~accFlag%如果不可达则切换策略
                currentStrategy=currentStrategy+1;
                [accFlag,object(i).path] = mutiAstar(map, object(i).start, object(i).goal, strategy(currentStrategy,2), strategy(currentStrategy,1));
                if accFlag==1
                    if i<target
                        if collisionDetect(object(target).path,object(i).path)
                            accFlag=0;
                            continue;
                        end
                        j=1;
                        while j<i
                            if collisionDetect(object(j).path,object(i).path)
                                accFlag=0;
                                break;
                            end
                            j=j+1;
                        end
                    elseif i>target
                        j=1;
                        while j<i
                            if collisionDetect(object(j).path,object(i).path)
                                accFlag=0;
                                break;
                            end
                            j=j+1;
                        end
                    end
                end
            end
            [object(i).length,object(i).diagonal,object(i).off_diagonal]=getDistandCounts(object(i).path,1,size(object(i).path,1));
            object(i).choice=currentStrategy;
        end
    end
    
    %坐标转换与路径圆滑
    for i=1:length(object)
        object(i).real_path=trans2Position(object(i).path, O, unit);
        object(i).real_path_smooth=pathSmooth(object(i).real_path, r);
    end

    %结果输出
    pointMax=0;
    for i=1:length(object)
        if size(object(i).real_path_smooth,1)>pointMax
            pointMax=size(object(i).real_path_smooth,1);
        end
    end
    result=inf(pointMax,3,length(object));
    for i=1:size(result,3)
        for j=1:size(object(i).real_path_smooth,1)
            result(j,:,i)=object(i).real_path_smooth(j,:);
        end
    end
    
    %网格坐标转换实际坐标
    function result = trans2Position(position, origin, unit)
        result=zeros(size(position,1),2);
        for i1=1:size(position,1)
            result(i1,:)=[origin(1)+(position(i1,2)-0.5)*unit,origin(2)-(position(i1,1)-0.5)*unit];
        end
    end
    
    %A*+贪婪算法
    function [acFlag,path] = mutiAstar(grid, start, goal, diagonal, off_diagonal)
        stopFlag = 1;%停止迭代标识符
        counter = 0;%迭代次数
        counterMax = 1000;%最大迭代次数
        acFlag = 1;%可达标志
        path=[];%创建路径表
        while(stopFlag)
            reference = [diagonal,off_diagonal];%[对角线移动参考次数,非对角线移动参考次数]
            counter = counter + 1;%迭代累加
            %迭代跳出
            if counter>counterMax
                acFlag=0;
                break;
            end
            %初始化
            epsilon = 0.4;%贪婪策略变量
            closedList=[];%关闭列表，包含已搜索的节点
            gScore = inf(size(grid));%从起始点到当前点的路径代价
            gScore(start(1), start(2)) = 0;%起点的gScore为0
            hScore = inf(size(grid));%当前点到终止点的路径代价估值
            for i1=1:size(grid, 1)
                for j1=1:size(grid, 2)
                    hScore(i1,j1)=heuristic(goal,i1,j1);
                end
            end
            fScore = gScore + hScore;%总路径代价估值
            father = inf(size(grid,1),size(grid,2));
            father(start(1), start(2)) = 0;%起点的父节点为0
            current = start;%当前点置于起点
            %开始
            while ~isequal(reference, zeros(1,2))||isequal(current, goal)
                if isequal(current, goal)&&isequal(reference, zeros(1,2))
                    path = backtrack(current, closedList, father);
                    stopFlag = 0;
                    break;
                else
                    %将当前节点移入closedList
                    closedList = [closedList;current];
                    %更新可用行动集
                    actionList = [];
                    %生成当前节点的邻居
                    neighbors = get2Neighbors(current);
                    for i1=1:8
                        neighbor = neighbors(i1,:);
                        %邻居节点未越界且不是障碍物
                        if neighbor(1)>=1&&neighbor(1)<=size(grid,2)&&neighbor(2)>=1&&neighbor(2)<=size(grid,1)&&grid(neighbor(1),neighbor(2))~=1
                            %不在closedList中且还有对应移动次数
                            if ~any(closedList(:,1)==neighbor(1)&closedList(:,2)==neighbor(2))&&reference(neighbor(3))>0
                                %加入可用行动集
                                actionList = [actionList;neighbor];
                                %计算从当前节点到邻居的gScore
                                tentativeGScore = gScore(current(1), current(2)) + dist(current, neighbor);
                                gScore(neighbor(1), neighbor(2)) = tentativeGScore;
                                fScore(neighbor(1), neighbor(2)) = gScore(neighbor(1), neighbor(2)) + heuristic(goal, neighbor(1), neighbor(2));
                                father(neighbor(1), neighbor(2)) = sub2ind([size(father,1),size(father,2)],current(1),current(2));
                            end
                        end
                    end
                    %贪婪选择
                    if isempty(actionList)
                        break;
                    end
                    chance = rand;
                    if chance>epsilon%向fScore最小的一格移动
                        fBest=inf;
                        for i1 = 1:size(actionList,1)
                            if fScore(actionList(i1,1),actionList(i1,2)) < fBest
                                fBest = fScore(actionList(i1,1),actionList(i1,2));
                                target1 = i1;
                            end
                        end
                        current(1) = actionList(target1,1);
                        current(2) = actionList(target1,2);
                        reference(actionList(target1,3))=reference(actionList(target1,3))-1;%对应移动次数减一
                    else%随机移动
                        randomIdx = randi(size(actionList,1));
                        current(1) = actionList(randomIdx,1);
                        current(2) = actionList(randomIdx,2);
                        reference(actionList(randomIdx,3))=reference(actionList(randomIdx,3))-1;%对应移动次数减一
                    end
                end
            end 
        end
    end
    
    %单机A*
    function path = astar(grid, start, goal)
        %初始化
        openList=[];%开放列表，包含待搜索的节点
        closedList=[];%关闭列表，包含已搜索的节点
        gScore = inf(size(grid));%从起始点到当前点的路径代价
        gScore(start(1), start(2)) = 0;%起点的gScore为0
        hScore = inf(size(grid));%当前点到终止点的路径代价估值
        for i1=1:size(grid, 1)
            for j1=1:size(grid, 2)
                hScore(i1,j1)=heuristic(goal,i1,j1);
            end
        end
        fScore = gScore + hScore;%总路径代价估值
        father = inf(size(grid));%父节点列表
        father(start(1), start(2)) = 0;%起点的父节点为0
        
        openList=[openList;start];%将起点加入开放列表
        while ~isempty(openList)
            fBest=inf;
            for i1 = 1:size(openList,1)
                if fScore(openList(i1,1),openList(i1,2)) < fBest
                    fBest = fScore(openList(i1,1),openList(i1,2));
                    current = openList(i1,:);
                    index = i1;
                end
            end
            if ~isequal(current, goal)
                %将当前节点移出openList并移到closedList
                openList(index,:) = [];
                closedList = [closedList;current];
                %生成当前节点的邻居
                neighbors = getNeighbors(current);
                for i1=1:8
                    neighbor = neighbors(i1,:);
                    %邻居节点未越界且不是障碍物
                    if neighbor(1)>=1&&neighbor(1)<=size(grid,2)&&neighbor(2)>=1&&neighbor(2)<=size(grid,1)&&grid(neighbor(1),neighbor(2))~=1
                        %不在在closedList中
                        if ~any(closedList(:,1)==neighbor(1)&closedList(:,2)==neighbor(2))
                            %计算从当前节点到邻居的gScore
                            tentativeGScore = gScore(current(1), current(2)) + dist(current, neighbor);
                            if isempty(openList) || ~any(openList(:,1) == neighbor(1) & openList(:,2) == neighbor(2)) || tentativeGScore < gScore(neighbor(1), neighbor(2))
                                gScore(neighbor(1), neighbor(2)) = tentativeGScore;
                                fScore(neighbor(1), neighbor(2)) = gScore(neighbor(1), neighbor(2)) + heuristic(goal, neighbor(1), neighbor(2));
                                father(neighbor(1), neighbor(2)) = sub2ind([size(father,1),size(father,2)],current(1),current(2));
                                
                                % 如果邻居不在openList中，则添加进去
                                if ~any(openList(:,1) == neighbor(1) & openList(:,2) == neighbor(2))
                                    openList = [openList; neighbor];
                                end
                            end
                        end
                    end
                end
            else
                path = backtrack(current, closedList, father);
                break;
            end
        end
    end
    
    %启发函数：欧几里得距离
    function h = heuristic(goal, x, y)
        h = sqrt((goal(1) - x)^2 + (goal(2) - y)^2);
    end
    
    %获取节点的邻居
    function neighbors = getNeighbors(node)
        dx=[0,1,1,1,0,-1,-1,-1];
        dy=[1,1,0,-1,-1,-1,0,1];
        neighbors=zeros(8,2);
        for i1=1:8
            neighbors(i1,:) = node + [dx(i1),dy(i1)];
        end
    end
    
    %获取节点的邻居(带方向标识符)
    function neighbors = get2Neighbors(node)
        node=[node,0];
        dx=[0,1,1,1,0,-1,-1,-1];
        dy=[1,1,0,-1,-1,-1,0,1];
        isdiagonal=[2,1,2,1,2,1,2,1];%方位标识符
        neighbors=zeros(8,3);
        for i1=1:8
            neighbors(i1,:) = node + [dx(i1),dy(i1),isdiagonal(i1)];
        end
    end
    
    %计算两个节点之间的代价
    function d = dist(node1,node2)
        if node1(1)==node2(1)||node1(2)==node2(2)
            d=1;
        else
            d=sqrt(2);
        end
    end
    
    %回溯路径
    function path = backtrack(current, closedList, father)
        path = current;
        closedList=[closedList;current];
            while ~isempty(closedList)
                [idx, ~] = find(closedList(:,1) == current(1) & closedList(:,2) == current(2), 1);
                
                % 如果找到了父节点，则将其加入路径，并更新当前节点为父节点
                if father(closedList(idx, 1),closedList(idx, 2))~=0
                    [row,col] = ind2sub([size(father,1),size(father,2)],father(closedList(idx, 1),closedList(idx, 2)));
                    parent = [row,col];
                    path = [path;parent];
                    current = parent;
                else
                    % 如果没有父节点了，说明已经回溯到了起点
                    break;
                end
            end
            % 反转路径，使其从起点到终点
        path = flipud(path);
    end
    
    %碰撞检测
    function result = collisionDetect(pathRef,path)
        standard=0.05;
        for i1=1:size(path,1)
            for j1=1:size(pathRef,1)
                %如果两路径有点相同且不是终点
                if path(i1,1)==pathRef(j1,1)&&path(i1,2)==pathRef(j1,2)&&i1~=size(path,1)
                    %获取可能碰撞点到起点的距离
                    [length,~,~]=getDistandCounts(path,1,i1);
                    [lengthRef,~,~]=getDistandCounts(pathRef,1,j1);
                    %如果距离相等则为碰撞点
                    if abs(length-lengthRef)<standard
                        result=1;
                        return;
                    end
                end
            end
        end
        result=0;
    end
    
    %求两路径点间距离和移动次数
    function [length,diagonal,off_diagonal]=getDistandCounts(path,index1,index2)%index1<index2
        current=path(index1,:);
        length=0;
        diagonal=0;
        off_diagonal=0;
        for i1=index1+1:index2
            if current(1)==path(i1,1)||current(2)==path(i1,2)
                off_diagonal=off_diagonal+1;
                length=length+1;
            else
                diagonal=diagonal+1;
                length=length+sqrt(2);
            end
            current=path(i1,:);
        end
    end
    
    %获取策略表
    function result = getStrategy(start, goal, lengthRef)
        %初始化
        offDiagonal=0;
        standard=1;
        result=[];
    
        %求距离
        dx=abs(start(1)-goal(1));
        dy=abs(start(2)-goal(2));
        distance=dx+dy;
        
        %获取策略表
        while 1
            offDiagonal=offDiagonal+1;
            diagonal=0;
            if distance>offDiagonal
                diagonal=ceil((distance-offDiagonal)/sqrt(2));
            end
            if offDiagonal>lengthRef
                break;
            end
            while 1
                lengthCurrent=offDiagonal+sqrt(2)*diagonal;
                if abs(lengthCurrent-lengthRef)<=standard
                    resultCurrent=[offDiagonal,diagonal,lengthCurrent,abs(lengthCurrent-lengthRef)];
                    result=[result;resultCurrent];
                end
                if lengthCurrent-lengthRef>0
                    break;
                end
                diagonal=diagonal+1;
             end
        end
    
        %从小到大冒泡排序
        for i1=1:size(result,1)
            for j1=1:size(result,1)-i1
                if result(j1,4)>result(j1+1,4)
                    temp=result(j1,:);
                    result(j1,:)=result(j1+1,:);
                    result(j1+1,:)=temp;
                end
            end
        end
    end
    
    %平滑航点
    function result = pathSmooth(path, r)
        result=[path(1,:),round(deltapsi(path(2,1),path(2,2),path(1,1),path(1,2))*rad2deg)];
        for i1=1:size(path,1)-2
            alpha=round(deltapsi(path(i1+1,1),path(i1+1,2),path(i1,1),path(i1,2))*rad2deg);
            beta=round(deltapsi(path(i1+2,1),path(i1+2,2),path(i1+1,1),path(i1+1,2))*rad2deg);
            delta=alpha-beta;
            if delta<-180
                delta=delta+360;
            elseif delta>180
                delta=delta-360;
            end
            if isequal(abs(delta),90)
                p1=path(i1+1,:)-[r*sin(alpha/rad2deg),r*cos(alpha/rad2deg)];
                p2=path(i1+1,:)+[r*sin(beta/rad2deg),r*cos(beta/rad2deg)];
                result=[result;p1,alpha;p2,beta];
            elseif isequal(abs(delta),45)
                p1=path(i1+1,:)-[r*tan(22.5/rad2deg)*sin(alpha/rad2deg),r*tan(22.5/rad2deg)*cos(alpha/rad2deg)];
                p2=path(i1+1,:)+[r*tan(22.5/rad2deg)*sin(beta/rad2deg),r*tan(22.5/rad2deg)*cos(beta/rad2deg)];
                result=[result;p1,alpha;p2,beta];
            elseif isequal(abs(delta),135)
                p1=path(i1+1,:)-[r*tan(67.5/rad2deg)*sin(alpha/rad2deg),r*tan(67.5/rad2deg)*cos(alpha/rad2deg)];
                p2=path(i1+1,:)+[r*tan(67.5/rad2deg)*sin(beta/rad2deg),r*tan(67.5/rad2deg)*cos(beta/rad2deg)];
                result=[result;p1,alpha;p2,beta];
            end
        end
        result=[result;path(size(path,1),:),round(deltapsi(path(size(path,1),1),path(size(path,1),2),path(size(path,1)-1,1),path(size(path,1)-1,2))*rad2deg)];
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
        elseif (deltay==0)
            if(deltax>0)
                delta_psi=pi/2;
            else
                delta_psi=pi*3/2;
            end
        end
    end
end