CTRL_CONST.dt = 0.005;

CTRL_PARAM.UAV_ID   = 1;
CTRL_PARAM.ADJ_MARTIX = [1 1 1;1 1 1;1 1 1];                % Adjacency matrix --> A
CTRL_PARAM.REL_X_MATRIX = [0 -40 -20;40 0 20;20 -20 0];     % Postion X Relation matrix --> Rx
CTRL_PARAM.REL_Y_MATRIX = [0 20 40;-20 0 20;-40 -20 0];     % Postion Y Relation matrix --> Ry
CTRL_PARAM.REL_Z_MATRIX = [0 0 0;0 0 0;0 0 0];              % Postion Z Relation matrix --> Rz
CTRL_PARAM.NUM_UAV  = size(CTRL_PARAM.ADJ_MARTIX, 1);       % Num of uav at now.


%% constrain
CTRL_PARAM.W_MIN = single(1.0);     % deg / s 
CTRL_PARAM.AIRSPD_MIN   = single(10.0); % Minimum Airspeed (CAS)
CTRL_PARAM.AIRSPD_MAX   = single(20.0); % Maximum Airspeed (CAS)
CTRL_PARAM.AIRSPD_TRIM  = single(15.0); % Trim (Cruise) Airspeed

%% control param
CTRL_PARAM.FORM_X_KP = single(1);
CTRL_PARAM.FORM_Y_KP = single(1);
CTRL_PARAM.FORM_VH_KP = single(1);