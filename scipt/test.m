tic
% 定义 Dubins 路径段的起始点和相关参数
startPose = [0, 0, 0];   % 起始点的位姿 [x, y, theta]
goalPose = [1 1 pi];

% 计算 Dubins 路径段的相关信息
dubConnObj = dubinsConnection;

segment = connect(dubConnObj,startPose, goalPose);

numPoints = 5;   % 路径点的数量
length = segment{1}.Length;
seg = 0:length/(numPoints - 1):length;
poses = interpolate(segment{1});

% 显示路径点
figure;
plot(points(:, 1), points(:, 2), 'r-', 'LineWidth', 2);
xlabel('X');
ylabel('Y');
title('Dubins 路径');
grid on;
axis equal;

toc