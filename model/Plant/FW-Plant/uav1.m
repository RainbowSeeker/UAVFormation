function [sys,x0,str,ts] = uav(t,x,u,flag)
% function [sys,x0,str,ts] = uav(t,x,u,flag)
% Simulate the flight dynamics of uav
%
% Airmud   2021.4.1
switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes

sizes = simsizes;

sizes.NumContStates  = 12;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 13;
sizes.NumInputs      = 4;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
rad2deg=57.295779513082320876798154814105;

psi_hmr=0.0;
if (psi_hmr> 180)   psi_hmr = psi_hmr-360.0; end
if (psi_hmr<-180)   psi_hmr = psi_hmr+360.0; end

Vt =25; alpha=(2.7480)/rad2deg; beta =0.0/rad2deg;theta0=(2.7480)/rad2deg;

x0  = [Vt; alpha; beta; 1000; 100; 1000; 0; 0;0;0;theta0;psi_hmr/rad2deg;  ];
% str is always an empty matrix
%
str = [];
%
% initialize the array of sample times
%
ts  = [0 0];

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u)
% state
% --------------
% Vt,alpha,beta,  phi,theta, psi,  P,Q,R,  PN,PE,H
%
% Definition
% ----------
% [Vt,alpha,beta]  ----- airspeed(m/s), angle of attack(rad), angle of sideslip(rad)
% [PN,PE, H]      ----- position of north(rad),position of east(rad),alt(m)
% [P,Q,R]          ----- roll rate(rad/s),pitch rate(rad/s),yaw rate(rad/s)
% [phi,theta,psi]  ----- roll angle,pitch angle,heading angle(rad)

Vt      = x(1);   PN = x(4);    P = x(7);   phi   = x(10);       
alpha   = x(2);  PE = x(5);   Q = x(8); theta = x(11);        
beta    = x(3);    H = x(6);   R = x(9);   psi   = x(12);      

ele = u(1);      % elevator deflection angle (deg)
ail = u(2);      % aileron  deflection angle (deg)
rud = u(3);      % rudder   deflection angle (deg)
eng = u(4);      
% ------------------------------------------------% 
SA   = 1.3536;          % [机翼面积]|[平方米]
b    = 3.2;             % [翼展]|[米]
cbar   = 0.423;           % [平均气动弦长]|[米]
Jx  =1.71;
Jy  =5.54;
Jz  =4.15;
Jxz =0;
mass = 17;
rad2deg=57.295779513082320876798154814105;
g=9.81;

alpha_deg = alpha*rad2deg;
beta_deg = beta*rad2deg;

% ---- 计算转换阵，从来流系到机体系----
salpha = sin(alpha);	   sbeta = sin(beta);
calpha = cos(alpha);	   cbeta = cos(beta);	  

% ---- 迎角侧滑计算机体三轴速度----
U = Vt*calpha*cbeta;
V = Vt*sbeta;
W = Vt*salpha*cbeta;

% ---- 计算转换阵，从地面系到机体系----
sphi = sin(phi);	   stheta = sin(theta);	    spsi = sin(psi);
cphi = cos(phi);	   ctheta = cos(theta);	    cpsi = cos(psi);

% ---- 计算推力----
Pow =eng/100*(mass*g/4.0);
% ---- 计算空气密度和动压----
[ru,mach] = UAV_density(H,Vt);   % [air density] [mach number]
qs = SA*(ru*Vt*Vt/2);         % [Dynamic pressure](kg/m^2)
% ---- 计算气动力----
CD=UAV_CD(alpha_deg);
D = qs*CD;

CY_beta = UAV_CY(alpha_deg);
Y = qs*(CY_beta*beta_deg );  

[CL0,CL_ele]=UAV_CL(alpha_deg);
L = qs*(CL0 + CL_ele*ele);
    
% ---- 计算线运动方程----
Fx = Pow - D*calpha*cbeta - Y*calpha*sbeta + L*salpha;
Fy =     - D*sbeta        + Y*cbeta;
Fz =     - D*salpha*cbeta - Y*salpha*sbeta - L*calpha;
Ax = Fx/mass;
Ay = Fy/mass;
Az = Fz/mass;
dU = R*V - Q*W + Ax - g*stheta;
dV =-R*U + P*W + Ay + g*sphi*ctheta;
dW = Q*U - P*V + Az + g*cphi*ctheta;
dVt=(U*dU+V*dV+W*dW)/Vt;
dbeta=(dV*Vt-V*dVt)/(Vt*Vt*cbeta);
dalpha=(U*dW-W*dU)/(U*U+W*W);

% ---- 计算气动力矩----
[CR_beta,CR_ail,CR_rud,CR_P,CR_R]= UAV_CR(alpha_deg,beta_deg);
Lbar = qs*b*(CR_beta*beta_deg + CR_ail*ail + CR_rud*rud + (CR_P*P + CR_R*R)*b/Vt/2);

[CM0,CM_ele,CM_Q,CM_dalpha]=UAV_CM(alpha_deg); 
M = qs*cbar*(CM0+ CM_ele*ele + (CM_Q*Q)*cbar/Vt/2);  % + CM_dalpha*dalpha

[CN_beta,CN_ail,CN_rud,CN_P,CN_R]=UAV_CN(alpha_deg,beta_deg);
N = qs*b*(CN_beta*beta_deg + CN_ail*ail + CN_rud*rud + (CN_P*P + CN_R*R)*b/Vt/2);
    
% ---- 计算角运动方程----
T  = Jx*Jz-Jxz*Jxz;
c1 = ((Jy-Jz)*Jz-Jxz*Jxz)/T;
c2 = (Jx-Jy+Jz)*Jxz/T;
c3 = Jz/T;
c4 = Jxz/T;
c5 = (Jz-Jx)/Jy;
c6 = Jxz/Jy;
c7 = 1/Jy;
c8 = (Jx*(Jx-Jy)+Jxz*Jxz)/T;
c9 = Jx/T;
dP = (c1*R+c2*P)*Q + c3*Lbar + c4*N;
dQ = c5*P*R - c6*(P*P-R*R) + c7*M;
dR = (c8*P-c2*R)*Q + c4*Lbar + c9*N;

% ---- 计算欧拉角微分方程----
dPhi = P + (stheta/ctheta)*(Q*sphi+R*cphi);
dTheta = Q*cphi - R*sphi;
dPsi = (Q*sphi+R*cphi)/ctheta;

% ---- 计算地面系分速度----
dPN = U*ctheta*cpsi + V*(-cphi*spsi+sphi*stheta*cpsi) + W*(sphi*spsi+cphi*stheta*cpsi);
dPE = U*ctheta*spsi + V*(cphi*cpsi+sphi*stheta*spsi)  + W*(-sphi*cpsi+cphi*stheta*spsi);
dH  = U*stheta      - V*sphi*ctheta                   - W*cphi*ctheta;

sys = [dVt;dalpha;dbeta;dPN;dPE;dH;dP;dQ;dR;dPhi;dTheta;dPsi;];

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate
%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)

Vt      = x(1);   PN = x(4);    P = x(7);   phi   = x(10);       
alpha   = x(2);  PE = x(5);   Q = x(8); theta = x(11);        
beta    = x(3);    H = x(6);   R = x(9);   psi   = x(12);     

rad2deg=57.295779513082320876798154814105;

psi_hmr=psi*rad2deg;
if (psi_hmr<   0.0) psi_hmr=psi_hmr+360.0; end
if (psi_hmr>=360.0) psi_hmr=psi_hmr-360.0; end
sys = [Vt;alpha*rad2deg;beta*rad2deg;PN;PE;H;P*rad2deg;Q*rad2deg;R*rad2deg;phi*rad2deg;theta*rad2deg;psi*rad2deg;(theta-alpha)*rad2deg];
% end mdlOutputs
%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate

%
% ============================================================================
% UAV_density
% Return air density and mach number
% ============================================================================
function [ru,mach]=UAV_density(alt,VT)
%[ru,d2ru,mach]=density(H)
%
%Calculate the atmospheric density (ru),its differentiation with altitude (d2ru)
%and mach number (mach)
%
%Inputs:
%   Q     dynamic pressure (N/m^2)
%   H     altitude (m)
%
%Outputs:
%   ru    atmospheric density at specified altitude (kg/m^3)
% d2ru    differentiation of atmospheric density with altitude (kg/m^4)
% mach    mach number
%
K  = 34.163195;
TL = 288.15;
PL = 101325;
RL = 1.225;
C1 = 0.001;
AL = 340.294;
ML = 1.7894E-05;
BT = 1.458E-06;

H = C1 * alt / (1 + C1 * alt / 6356.766);
if H<11
    T  = 288.15 - 6.5 * H;
    PP = (288.15/T)^(-K / 6.5);
elseif H<20
    T  = 216.65;
    PP = 0.22336*exp(- K*(H-11)/216.65);
elseif H<32
    T  = 216.65 + (H - 20);
    PP = 0.054032 * (216.65/T)^K;
elseif H<47
    T  = 228.65 + 2.8 * (H - 32);
    PP = 0.0085666 * (228.65/T)^(K/2.8);
elseif H<51
    T  = 270.65;
    PP = 0.0010945 *exp(-K*(H-47)/270.65);
elseif H<71
    T  = 270.65 - 2.8*(H - 51);
    PP = 0.00066063*(270.65/T)^(-K/2.8);
else
    T = 214.65 - 2 * (H - 71);
    PP = 3.9046e-05 * (214.65 / T)^(-K/2);
end

RR = PP / (T / 288.15);
MU = BT * T^1.5 / (T + 110.4);
TS = T / 288.15;
sonic  = AL *  sqrt(TS);
T  = TL * TS;
ru  = RL * RR;
P  = PL * PP;
RM = ru * sonic / MU;
QM = .7 * P;

mach =VT/sonic;

% end
%=============================================================================
%UAV_CD
%=============================================================================
function CD = UAV_CD(alpha_deg)
    IDX_alpha =[-4;-2;0;2;4;8;12;16;20];
    TBL_CD =[0.026;0.024;0.024;   0.028;0.036;0.061;0.102;0.141;0.173];
%         IDX_alpha =[-6.00;-4.00;-2.00;0.00;   2.00;4.00;6.00;  
%                 8.00;10.00;12.00;14.00;16.00;18.00;20.00];
%     TBL_Cx =[0.03931;0.03863;0.04155;   0.04847;0.05841;0.07079;0.08530;
%              0.10213;0.11742;0.16796;0.23854;0.29419;0.37927;0.47888];
    CD =interp1d(TBL_CD,IDX_alpha,alpha_deg);
%end function
%=============================================================================
%UAV_CL
%=============================================================================
function [CL0,CL_ele]=UAV_CL(alpha_deg)
    IDX_alpha =[-4;-2;0;  2;4;8;12;16;20];
    TBL_CL0 =[-0.219;-0.04;0.139;  0.299;0.455;0.766;1.083;1.409;1.743];
%     IDX_alpha =[-6.00;-4.00;-2.00;0.00;   2.00;4.00;6.00;
%                 8.00;10.00;12.00;14.00;16.00;18.00;20.00];
%     TBL_Cy0 =[0.00983;0.21652;0.41990;   0.61268;0.78538;0.95201;1.11654;
%               1.26195;1.39341;1.44498;1.44329;1.44519;1.28669;1.13923];
    
    CL0 =interp1d(TBL_CL0,IDX_alpha,alpha_deg);
    CL_ele =0.00636;
%end UAV_CL

%=============================================================================
%UAV_CY
%=============================================================================
function CY_beta=UAV_CY(alpha_deg)
    CY_beta =-0.00909;
%end UAV_CY

%=============================================================================
%UAV_CM
%=============================================================================
function [CM0,CM_ele,CM_Q,CM_dalpha]=UAV_CM(alpha_deg)
    IDX_alpha =[-4;-2;0;2;4;8;12;16;20];
    TBL_CM0=[0.1161;0.0777;0.0393;0.0009;-0.0375;-0.0759;-0.1527;-0.2295;-0.3063];
    CM0 =interp1d(TBL_CM0,IDX_alpha,alpha_deg);
    CM_ele =-0.02052;
    CM_Q =-9.3136;
    CM_dalpha =-4.0258;
%end UAV_CM

%=============================================================================
%UAV_CR
%=============================================================================
function [CR_beta,CR_ail,CR_rud,CR_P,CR_R]= UAV_CR(alpha_deg,beta_deg)
    CR_beta =-0.00600;
    CR_ail =-0.003618;
    CR_rud = 0.000144;
    CR_P =-0.52568;
    CR_R = 0.01832;
%end UAV_CR
%=============================================================================
%UAV_CN
%=============================================================================
function [CN_beta,CN_ail,CN_rud,CN_P,CN_R]=UAV_CN(alpha_deg,beta_deg)
    CN_beta=0.00235;
    CN_ail =0.000132;
    CN_rud =-0.00111;
    CN_P = 0.01792;
    CN_R =-0.15844;
%end UAV_CN
%=============================================================================
%interp1d
%=============================================================================
function  y = interp1d(A,idx,xi)

if xi<idx(1)
   r=1;
elseif xi<idx(end)
   r = max(find(idx <= xi));      
else
   r = length(idx)-1;
end

DA = (xi-idx(r))/(idx(r+1)-idx(r));
y = A(r) + (A(r+1)-A(r))*DA;
% END interp1d

