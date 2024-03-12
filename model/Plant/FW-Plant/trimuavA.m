function [xtrim,ytrim,utrim,dxtrim,along,blong,clong,dlong]=trimuavA(V,H,path)
% Calculate trim date
%         1      2          3     4      5   6   7   8   9   10   11   12
% x = [Vt, alpha, beta, PN, PE, alt,  P,  Q, R, phi, theta, psi,]
%       1    2     3     4      5     6    7   8  9   10   11   12
% Y = [Vt, alpha, beta, PN, PE, alt,  P,  Q, R, phi, theta, psi,theta-alpha]
%  u = [elevator, aileron, rudder, engine]

x0 = [V;3/57.3;0;  0;0;H; 0;0;0; 0;3/57.3+path/57.3;0; ]; 
IX  = [1;7;8;9; 10; 12];

u0 = [0;0;0;0.3];
IU  = [2;3];

y0 = [V;0;0; 0;0;H; 0;0;0; 0;0;0; path];
IY  = [1;7;8;9; 10; 12; 13];

dx0= zeros(12,1);  %12°¡1¡–æÿ’Û
IDX=[1;2;3;7;8;9;10;11;12];


options=optimset('MaxFunEvals',2e4,'TolX',1e-4,'TolFun',1e-4);

[xtrim,utrim,ytrim,dxtrim] =trim('uavCtrl',x0,u0,y0,IX,IU,IY,dx0,IDX);

[a,b,c,d]  = linmod('uavCtrl',xtrim,utrim);
%       1           2     3      4       5      6       7   8  9   10  11  12
%  y = [Vt, alpha, beta, phi, theta, psi,  P,  Q, R, PN, PE, alt, theta-alpha]
% Y = [Vt, alpha, beta, PN, PE, alt,      P,  Q, R,   phi, theta, psi,theta-alpha]
[along,blong,clong,dlong] = ssselect(a,b,c,d,[1;4],[1;2;8;11;6],[1;2;8;11;6]);
[alate,blate,clate,dlate] = ssselect(a,b,c,d,[2;3],[3;10;12;7;9;5], [3;10;12;7;9;5]);
save('data','xtrim','utrim','ytrim','dxtrim','a','b','c','d','along','blong','clong','dlong','alate','blate','clate','dlate');
% damp(along);
% damp(alate);
% [num,den]=ss2tf(along,blong,clong(4,:),dlong(4,:),1);
% printsys(num,den,'s');

