
dubConnObj = dubinsConnection;
[pathSegObj, ~] = connect(dubConnObj, [0 0 0], [2 2 pi/2]);
show(pathSegObj{1})
tic
path = interpolate(pathSegObj{1});
toc


