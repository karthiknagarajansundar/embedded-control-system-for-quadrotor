m = 0.027;% Mass
g = 9.81;% Gravitational acceleration
d = 0.046;% Distance from center of mass to rotor axis
lift = 1.9796e-9;% Lift constant
drag = 2.5383e-11;% Drag constant

Ts = 0.01; alpha = 0.05; gamma = alpha / ( Ts + alpha );

Jx = 1.1463e-5; Jy = 1.6993e-5; Jz = 2.9944e-5;
J = [Jx,0,0;0,Jy,0;0,0,Jz];

bx = (d*cos(pi/4))/Jx;
by = (d*cos(pi/4))/Jy;
bz = drag/(lift*Jz);

%CONTINUOUS SYSTEM
Ac = zeros(5,5); Ac(1,3) = 1; Ac(2,4)  = 1;
Bc = [0 0 0 0;
      0 0 0 0;
      -bx, -bx, bx, bx;
      -by, by, by, -by;
      -bz, bz, -bz, bz];
C = diag([0,0,1,1,0]); D=0;
%CONTINUOUS TO DISCRETE
sysc = ss(Ac,Bc,C,D); 
sysd = c2d(sysc,0.01);
Ad = sysd.A; 
Bd = sysd.B;
%TUNING PARAMETERS
Q=diag([500 500 1 1 1]); 
R=0.1*eye(4);
%GAIN MATRIX
[Klqr,S,e] = dlqr(Ad,Bd,Q,R);

Kref = pinv(eye(5)*inv(eye(5)-Ad+Bd*Klqr)*Bd);

 %N=zeros(5,4);
%CONT TO DISC
% [K_lqr,S,e] = lqrd(Ac,Bc,Q,R,N,0.01);