%% Constant Variable
FMS_CONST.dt = 0.005;

%% Paramaters
FMS_PARAM_VALUE.THROTTLE_DZ = single(0.15);
FMS_PARAM_VALUE.YAW_DZ = single(0.15);
FMS_PARAM_VALUE.ROLL_DZ = single(0.1);
FMS_PARAM_VALUE.PITCH_DZ = single(0.1);
FMS_PARAM_VALUE.XY_P = single(0.95);
FMS_PARAM_VALUE.Z_P = single(1);
FMS_PARAM_VALUE.VEL_Z_LIM = single(2.5);
FMS_PARAM_VALUE.YAW_P = single(2.5);
FMS_PARAM_VALUE.YAW_RATE_LIM = single(pi/3);
FMS_PARAM_VALUE.ROLL_PITCH_LIM = single(pi/6);
FMS_PARAM_VALUE.L1 = single(30.0);
FMS_PARAM_VALUE.CRUISE_SPEED = single(13);
FMS_PARAM_VALUE.ACCEPT_R = single(10.0);
FMS_PARAM_VALUE.LOITER_R = single(50.0);

FMS_PARAM_VALUE.Y_P = single(0.95);
FMS_PARAM_VALUE.ACC_Y_LIM = single(8);
FMS_PARAM_VALUE.ROLL_LIM = single(pi/4);
FMS_PARAM_VALUE.PITCH_LIM = single(pi/4);

FMS_PARAM = Simulink.Parameter(FMS_PARAM_VALUE);
FMS_PARAM.CoderInfo.StorageClass = 'ExportedGlobal';