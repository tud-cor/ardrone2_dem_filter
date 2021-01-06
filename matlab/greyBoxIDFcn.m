function [A,B,C,D,K,x0] = greyBoxIDFcn(cT1,cT2,ts,nu,nx,ny,g,l,m,ixx,iyy,izz,cA1,cA2,pwmEq,cQ1,cQ2)
% Determine derivative terms for thrust and torque w.r.t. PWM
cTDer      = 2*cT1*cA1^2*pwmEq + 2*cT1*cA1*cA2 + cT2*cA1;
cTPhiDer   = sqrt(2)/2*l*(2*cT1*cA1^2*pwmEq + 2*cT1*cA1*cA2+cT2*cA1);
cTThetaDer = sqrt(2)/2*l*(2*cT1*cA1^2*pwmEq + 2*cT1*cA1*cA2+cT2*cA1);
cTPsiDer   = 2*cQ1*cA1^2*pwmEq + 2*cQ1*cA1*cA2 + cQ2*cA1;

% Construct elements of B-matrix
bZDot     = 1/m*cTDer;
bPhiDot   = 1/ixx*cTPhiDer;
bThetaDot = 1/iyy*cTThetaDer;
bPsiDot   = 1/izz*cTPsiDer;

A = [0, 0, 0, 1, 0, 0, 0 , 0, 0, 0, 0, 0;
     0, 0, 0, 0, 1, 0, 0 , 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 1, 0 , 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0 , g, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, -g, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0 , 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0 , 0, 0, 1, 0, 0;
     0, 0, 0, 0, 0, 0, 0 , 0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 0, 0 , 0, 0, 0, 0, 1;
     0, 0, 0, 0, 0, 0, 0 , 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0 , 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0 , 0, 0, 0, 0, 0];
B = [0,          0,          0,         0;
     0,          0,          0,         0;
     0,          0,          0,         0;
     0,          0,          0,         0;
     0,          0,          0,         0;
     bZDot,      bZDot,      bZDot,     bZDot;
     0,          0,          0,         0;
     0,          0,          0,         0;
     0,          0,          0,         0;
     bPhiDot,    -bPhiDot,   -bPhiDot,  bPhiDot;
     -bThetaDot, -bThetaDot, bThetaDot, bThetaDot;
     bPsiDot,    -bPsiDot,   bPsiDot,   -bPsiDot];

% xSel = [2,5,7,10];
xSel = [3,6];
A = A(xSel,xSel);
B = B(xSel,:);

C = zeros(ny,nx);
C(1,1) = 1;
D = zeros(ny,nu);

K = zeros(nx,ny);
x0 = zeros(nx,1);
end
