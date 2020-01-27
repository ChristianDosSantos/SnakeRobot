N = 3;
Ls = 0.14;
Ms = 1;
Js = 0.0016;
ct = 0.5;
cn = 3;
c1 = 0.45;
c2 = 3;
c3 = 0.5;
c4 = 20;
Ko = 0.036;
% Ko = 0.01;
Rm = 10.9;
% Rm = 1;
Bm = 1.34e-4;
% Bm = 0.1;
Tl = 1;
Lm = 2e-3;
% Lm = 0.5;
Jm = 3.2e-6;
% Jm = 0.01;
Alfa = 0.1;
% Ws = pi*70/180;
Ws = pi*10/180;
Delta = pi*40/180;
OffsetAngle = 0;
tmax = 100;
t = 0:0.001:tmax;
ORef = patheval(N-1,Delta,Ws,Alfa,OffsetAngle,t);
O1Ref = ORef(1,:);
O2Ref = ORef(2,:);
