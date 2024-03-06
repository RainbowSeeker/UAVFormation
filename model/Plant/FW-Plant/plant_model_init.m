%% load configuration
% load('aero_data.mat');

%% Constant Variable (for internal use)
PLANT_CONST.dt = 0.002;

% aero_data.con_v = 4;
% aero_data.con_h = 0.05;
% aero_data.con_vh = 1.184;
aero_data.con_v = 1;
aero_data.con_h = 1;
aero_data.con_vh = 1;

model_A = [0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1;0 0 0 -1/aero_data.con_v 0 0;0 0 0 0 -1/aero_data.con_v 0;0 0 -1/aero_data.con_vh 0 0 -1/aero_data.con_vh];
model_B = [0 0 0;0 0 0;0 0 0;1/aero_data.con_v 0 0;0 1/aero_data.con_v 0;0 0 1/aero_data.con_h];
model_C = eye(6);