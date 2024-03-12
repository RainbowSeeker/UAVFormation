%% load configuration
% load('aero_data.mat');

%% Constant Variable (for internal use)
PLANT_CONST.dt = 0.002;

[xtrim,ytrim,utrim,dxtrim,along,blong,clong,dlong] = trimuavA(25,1000,0);