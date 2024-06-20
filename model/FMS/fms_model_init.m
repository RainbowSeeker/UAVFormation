%% Constant Variable
FMS_CONST.dt = 0.02;

load("config.mat")
%% Fromation Paramaters
FORMATION_PARAM_VALUE.UAV_ID   = uint32(1);
FORMATION_PARAM_VALUE.ADJ_MARTIX = [0 0 0;1 0 0;1 0 0];                % Adjacency matrix --> A
FORMATION_PARAM_VALUE.REL_X_MATRIX = [0 20 20;-20 0 0;-20 0 0];     % Postion X Relation matrix --> Rx
FORMATION_PARAM_VALUE.REL_Y_MATRIX = [0 20 -20;-20 0 -40;20 40 0];     % Postion Y Relation matrix --> Ry
FORMATION_PARAM_VALUE.REL_Z_MATRIX = [0 20 40;-20 0 20;-40 -20 0];              % Postion Z Relation matrix --> Rz
FORMATION_PARAM_VALUE.NUM_UAV  = size(FORMATION_PARAM_VALUE.ADJ_MARTIX, 1);  % Num of uav at now.
FORMATION_PARAM_VALUE.FORM_POINT = [00 1000 0;
                              20 1000 0;
                              -20 1000 0];
FORMATION_PARAM_VALUE.DISBAND_POINT = [00 0 0;
                              20 0 0;
                              -20 0 0];
FORMATION_PARAM_VALUE.FORM_RADIUS = single(100);
FORMATION_PARAM_VALUE.ASSEMBLE_KT = single(1);
FORMATION_PARAM_VALUE.LATERAL_DAMP = single(0.2);
FORMATION_PARAM_VALUE.LATERAL_PERIOD = single(1);
FORMATION_PARAM_VALUE.FORM_POS_KP = single(0.3);
FORMATION_PARAM_VALUE.FORM_POS_KD = single(0.05);
FORMATION_PARAM_VALUE.FORM_VEL_KP = single(0.5);

FORMATION_PARAM = Simulink.Parameter(FORMATION_PARAM_VALUE);
FORMATION_PARAM.CoderInfo.StorageClass = 'ExportedGlobal';
%% Paramaters
FMS_PARAM_VALUE.FW_AIRSPD_TRIM = single(25);
FMS_PARAM_VALUE.FW_HEIGHT_TRIM = single(1000);
FMS_PARAM_VALUE.FW_RADIUS_RATIO = single(1.2);
FMS_PARAM_VALUE.AIRSPD_P = single(1);
FMS_PARAM_VALUE.Z_P= single(1);
FMS_PARAM_VALUE.L1 = single(50.0);
FMS_PARAM_VALUE.ACCEPT_R = single(100.0);
FMS_PARAM_VALUE.LOITER_R = single(100.0);
FMS_PARAM_VALUE.ACC_Y_LIM = single(8);

FMS_PARAM = Simulink.Parameter(FMS_PARAM_VALUE);
FMS_PARAM.CoderInfo.StorageClass = 'ExportedGlobal';