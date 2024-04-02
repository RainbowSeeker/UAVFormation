%% Constant Variable
FMS_CONST.dt = 0.05;

%% Fromation Paramaters
FORMATION_PARAM_VALUE.UAV_ID   = 1;
FORMATION_PARAM_VALUE.ADJ_MARTIX = [0 0 0;1 0 0;1 0 0];                % Adjacency matrix --> A
FORMATION_PARAM_VALUE.REL_X_MATRIX = [0 20 20;-20 0 0;-20 0 0];     % Postion X Relation matrix --> Rx
FORMATION_PARAM_VALUE.REL_Y_MATRIX = [0 20 -20;-20 0 -40;20 40 0];     % Postion Y Relation matrix --> Ry
FORMATION_PARAM_VALUE.REL_Z_MATRIX = [0 0 0;0 0 0;0 0 0];              % Postion Z Relation matrix --> Rz
FORMATION_PARAM_VALUE.NUM_UAV  = size(FORMATION_PARAM_VALUE.ADJ_MARTIX, 1);  % Num of uav at now.
FORMATION_PARAM_VALUE.FORM_POINT = [00 1000 0;
                              50 1000 0;
                              -50 1000 0];
FORMATION_PARAM_VALUE.FORM_RADIUS = single(100);
FORMATION_PARAM_VALUE.ASSEMBLE_KT = single(1);

FORMATION_PARAM = Simulink.Parameter(FORMATION_PARAM_VALUE);
FORMATION_PARAM.CoderInfo.StorageClass = 'ExportedGlobal';
%% Paramaters
FMS_PARAM_VALUE.FW_AIRSPD_TRIM = single(25);
FMS_PARAM_VALUE.FW_HEIGHT_TRIM = single(1000);

FMS_PARAM_VALUE.THROTTLE_DZ = single(0.15);
FMS_PARAM_VALUE.YAW_DZ = single(0.15);
FMS_PARAM_VALUE.ROLL_DZ = single(0.1);
FMS_PARAM_VALUE.PITCH_DZ = single(0.1);
FMS_PARAM_VALUE.AIRSPD_P = single(0.5);
FMS_PARAM_VALUE.Z_P = single(1);
FMS_PARAM_VALUE.VEL_Z_LIM = single(2.5);
FMS_PARAM_VALUE.YAW_P = single(2.5);
FMS_PARAM_VALUE.YAW_RATE_LIM = single(pi/3);
FMS_PARAM_VALUE.ROLL_PITCH_LIM = single(pi/6);
FMS_PARAM_VALUE.L1 = single(50.0);
FMS_PARAM_VALUE.ACCEPT_R = single(100.0);
FMS_PARAM_VALUE.LOITER_R = single(100.0);

FMS_PARAM_VALUE.Y_P = single(0.95);
FMS_PARAM_VALUE.ACC_Y_LIM = single(8);

FMS_PARAM = Simulink.Parameter(FMS_PARAM_VALUE);
FMS_PARAM.CoderInfo.StorageClass = 'ExportedGlobal';