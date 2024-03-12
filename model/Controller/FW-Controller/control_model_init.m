%% Model Constant
CONTROL_CONST.dt = 0.002;   % model execution period
CONTROL_CONST.g  = single(9.80665);

%% Fromation Paramaters
FROMATION_PARAM.UAV_ID   = 1;
FROMATION_PARAM.ADJ_MARTIX = [1 1 1;1 1 1;1 1 1];                % Adjacency matrix --> A
FROMATION_PARAM.REL_X_MATRIX = [0 -40 -20;40 0 20;20 -20 0];     % Postion X Relation matrix --> Rx
FROMATION_PARAM.REL_Y_MATRIX = [0 20 40;-20 0 20;-40 -20 0];     % Postion Y Relation matrix --> Ry
FROMATION_PARAM.REL_Z_MATRIX = [0 0 0;0 0 0;0 0 0];              % Postion Z Relation matrix --> Rz
FROMATION_PARAM.NUM_UAV  = size(FROMATION_PARAM.ADJ_MARTIX, 1);  % Num of uav at now.
FROMATION_PARAM.FORM_X_KP = single(1);
FROMATION_PARAM.FORM_Y_KP = single(1);
FROMATION_PARAM.FORM_Z_KP = single(0);

%% Control Paramaters
CONTROL_PARAM_VALUE.ROLL_P          = single(7);
CONTROL_PARAM_VALUE.PITCH_P         = single(7);
CONTROL_PARAM_VALUE.ROLL_RATE_P     = single(0.1);
CONTROL_PARAM_VALUE.PITCH_RATE_P    = single(0.2);
CONTROL_PARAM_VALUE.YAW_RATE_P      = single(0.15);
CONTROL_PARAM_VALUE.ROLL_RATE_I     = single(0.1);
CONTROL_PARAM_VALUE.PITCH_RATE_I    = single(0.1);
CONTROL_PARAM_VALUE.YAW_RATE_I      = single(0.2);
CONTROL_PARAM_VALUE.RATE_I_MIN      = single(-0.1);
CONTROL_PARAM_VALUE.RATE_I_MAX      = single(0.1);

CONTROL_PARAM_VALUE.TRIM_ROLL       = single(deg2rad(utrim(2)));    % Roll trim
CONTROL_PARAM_VALUE.TRIM_PITCH      = single(deg2rad(utrim(1)));    % Pitch trim
CONTROL_PARAM_VALUE.TRIM_YAW        = single(deg2rad(utrim(3)));    % Yaw trim
CONTROL_PARAM_VALUE.FW_PSP_OFF      = single(deg2rad(2.7480));  % Pitch setpoint offset (pitch at level flight)

CONTROL_PARAM_VALUE.FW_AIRSPD_MIN   = single(10.0); % Minimum Airspeed (CAS)
CONTROL_PARAM_VALUE.FW_AIRSPD_MAX   = single(30.0); % Maximum Airspeed (CAS)
CONTROL_PARAM_VALUE.FW_AIRSPD_TRIM  = single(25.0); % Trim (Cruise) Airspeed
CONTROL_PARAM_VALUE.FW_AIRSPD_STALL = single(7.0);  % Stall Airspeed (CAS)
CONTROL_PARAM_VALUE.FW_ARSP_MODE    = int32(0);     % Airspeed mode: 0 Use airspeed in controller
CONTROL_PARAM_VALUE.FW_ARSP_SCALE_EN= int32(1);     % Enable airspeed scaling
CONTROL_PARAM_VALUE.FW_T_SPD_STD    = single(0.2);  % Airspeed measurement standard deviation for airspeed filter
CONTROL_PARAM_VALUE.FW_T_SPD_PRC_STD= single(0.2);  % Process noise standard deviation for the airspeed rate in the airspeed filter
CONTROL_PARAM_VALUE.FW_T_TAS_TC     = single(5.0);  % True airspeed error time constant
CONTROL_PARAM_VALUE.FW_T_I_GAIN_PIT = single(0.1);  % Integrator gain pitch
CONTROL_PARAM_VALUE.FW_T_I_GAIN_THR = single(0.05); % Integrator gain throttle
CONTROL_PARAM_VALUE.FW_T_THR_DAMP   = single(0.1);  % Throttle damping factor
CONTROL_PARAM_VALUE.FW_T_SPDWEIGHT  = single(1.0);  % Speed <--> Altitude priority
CONTROL_PARAM_VALUE.FW_T_CLMB_MAX   = single(5.0);  % Maximum climb rate:This is the maximum climb rate that the aircraft can achieve with the throttle set to THR_MAX and the airspeed set to the trim value. 
CONTROL_PARAM_VALUE.FW_T_SINK_MIN   = single(2.0);  % Minimum descent rate
CONTROL_PARAM_VALUE.FW_T_SINK_MAX   = single(5.0);  % Maximum descent rate
CONTROL_PARAM_VALUE.FW_T_CLMB_R_SP  = single(3.0);  % Default target climbrate
CONTROL_PARAM_VALUE.FW_T_SINK_R_SP  = single(2.0);  % Default target sinkrate
CONTROL_PARAM_VALUE.FW_P_LIM_MAX    = single(deg2rad(30.0)); % Maximum pitch angle
CONTROL_PARAM_VALUE.FW_P_LIM_MIN    = single(deg2rad(-30.0));% Minimum pitch angle
CONTROL_PARAM_VALUE.FW_R_LIM        = single(deg2rad(50.0)); % Maximum roll angle
CONTROL_PARAM_VALUE.FW_T_VERT_ACC   = single(7.0);  % Maximum vertical acceleration
CONTROL_PARAM_VALUE.FW_T_ALT_TC     = single(5.0);  % Altitude error time constant
CONTROL_PARAM_VALUE.FW_T_HRATE_FF   = single(0.3);  % Height rate feed forward
CONTROL_PARAM_VALUE.FW_T_STE_R_TC   = single(0.4);  % Specific total energy rate first order filter time constant
CONTROL_PARAM_VALUE.FW_T_RLL2THR    = single(15.0); % Roll -> Throttle feedforward 

%% Rate Control
CONTROL_PARAM_VALUE.FW_RR_FF        = single(0.5);  % Roll rate feed forward
CONTROL_PARAM_VALUE.FW_PR_FF        = single(0.5);  % Pitch rate feed forward
CONTROL_PARAM_VALUE.FW_YR_FF        = single(0.3);  % Yaw rate feed forward
CONTROL_PARAM_VALUE.FW_R_RMAX       = single(deg2rad(70));  % Maximum roll rate setpoint
CONTROL_PARAM_VALUE.FW_P_RMAX       = single(deg2rad(60));  % Maximum up / down pitch rate setpoint
CONTROL_PARAM_VALUE.FW_Y_RMAX       = single(deg2rad(50));  % Maximum yaw rate setpoint
%% Throttle
CONTROL_PARAM_VALUE.FW_THR_MAX      = single(1.0);  % Throttle limit max
CONTROL_PARAM_VALUE.FW_THR_MIN      = single(0.0);  % Throttle limit min
CONTROL_PARAM_VALUE.FW_THR_TRIM     = single(utrim(4)/100);  % Trim throttle

CONTROL_PARAM_VALUE.FW_T_SEB_R_FF   = single(1.0);  % Specific total energy balance rate feedforward gain
CONTROL_PARAM_VALUE.FW_T_I_GAIN_PIT = single(0.1);  % Integrator gain pitch
CONTROL_PARAM_VALUE.FW_T_PTCH_DAMP  = single(0.1);  % Pitch damping factor
CONTROL_PARAM_VALUE.FW_T_I_GAIN_THR = single(0.05); % Integrator gain throttle
CONTROL_PARAM_VALUE.FW_T_THR_DAMP   = single(0.1);  % Throttle damping factor

% CONTROL_PARAM_VALUE.FW_ROLL_EFFC    = single(1.0);
% CONTROL_PARAM_VALUE.FW_PITCH_EFFC   = single(1.0);
% CONTROL_PARAM_VALUE.FW_YAW_EFFC     = single(1.0);
% Export to firmware
CONTROL_PARAM = Simulink.Parameter(CONTROL_PARAM_VALUE);
CONTROL_PARAM.CoderInfo.StorageClass = 'ExportedGlobal';