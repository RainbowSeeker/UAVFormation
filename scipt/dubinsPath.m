function path = dubinsPath(start, goal)
    % 参数：
    % start：起始点，格式为 [x, y, z, theta]，其中 (x, y, z) 是起始位置坐标，theta 是起始航向角（弧度）。
    % goal：目标点，格式同 start。
    % radius：转弯半径。
    
    dubConnObj = uavDubinsConnection;
    dubConnObj.AirSpeed = 15;
%     dubConnObj.MaxRollAngle = 0;

    [pathSegObj, ~] = connect(dubConnObj, start, goal);
    show(pathSegObj{1})
    flight_angel = start(4);
    now_pose = start(1:2);
    for i = 1:length(pathSegObj{1}.MotionTypes)
        switch pathSegObj{1}.MotionTypes(i)
            case 'R'
                flight_angel = wrapTo2Pi(flight_angel + pathSegObj{1}.MotionLengths(i) / pathSegObj{1}.MinTurningRadius);
                path(1) = now_pose(:) + 
        end
    end
end



