%% brief: run this script to init Model
start_time = 0;
end_time = 1000;

%% init path
filepath = which(mfilename);
filefolder = fileparts(filepath);
rootpath = fullfile(filefolder,'.');

%% set build path
buildpath = fullfile(rootpath, 'build');
if(~exist(buildpath, 'dir'))
   mkdir(buildpath); 
end
set_param(0, 'CodeGenFolder', buildpath);
set_param(0, 'CacheFolder', buildpath);

%% add path
addpath(genpath('build'));

%% load bus
load_bus(rootpath);

%% init models
run('plant_init.m');
run('fms_model_init.m');
run('control_init.m');

sl_refresh_customizations;