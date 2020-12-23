%%Longtitudinal x
Xu = -0.25;
Mu = 1.1;
g = 32.2;
M0 = 0.02;
delayx = 20;
Ax = [0 1 0 0 0;
      0 Xu 0 -g 0;
      0 Mu 0 0 M0;
      0 0 1 0 0;
      0 0 0 0 -delayx];
Bx = [0; 0; 0; 0; delayx];
Cx = [0 0 0 1 0; 1 0 0 0 0 ];
Dx = [0; 0];

s = tf('s');
[a, b] = ss2tf(Ax, Bx, Cx, Dx, 1);
tf1 = minreal(tf(a(1,:),b)); %inner loop transfer function for theta
tf2 = tf(a(2,:),b) %outer loop transfer function for x

%%Lateral y
Yv = -0.2;
Lv = -0.5;
g = 32.2;
Lphi = 0.016;
delayy = 20;
Ay = [0 1 0 0 0;
      0 Yv 0 g 0;
      0 Lv 0 0 Lphi;
      0 0 1 0 0;
      0 0 0 0 -delayy];
By = [0; 0; 0; 0; delayy];
Cy = [0 0 0 1 0; 1 0 0 0 0];
[c, d] = ss2tf(Ax, Bx, Cx, Dx, 1);
sys = ss(Ax, Bx, Cx, 0);
tf1y = minreal(tf(c(1,:),d)) %tf of inner loop phi
tf2y = tf(c(2,:),d) %tf of outer loop y

pdy = tf1y*500*(s+3);
c2y = 0.25*(s+3);
pdOuterY = -minreal(c2y*feedback(pdy,1)*tf2y/tf1y);
%rltool(pdOuterY);
ky = 0.1746 %from rltool on pdOuterY;

%%Z
Az = [0 0; 1 0];
Bz = [1/200; 0];
Cz = [0 1];

%Plot 10.57 - RLOC W/ PD Theta
figure(1);
pd = tf1*500*(s+4);
rlocus(pd);
title("10.57 - RLOC W/ PD Theta");

%Plot 10.58 - BODE W/ PD Theta
figure(2);
bode(pd);
title("10.58 - BODE W/ PD Theta");

%Plot 10.59 - RLOC W/ PD X
figure(3); 
c2 = 0.4*(s+2.2);
pdx = -minreal(c2*feedback(pd,1)*tf2/tf1)
rlocus(pdx);
%rltool(pdx);
k = 0.12019; %from rltool on pdx;
title("10.59 - RLOC W/ PD X");

%Plot 10.60 - STEP W/ PD X
figure(4);
closedX = feedback(k*pdx,1);
step(closedX);
title("10.60 - STEP W/ PD X");

%Plot 10.61 - COMMAND W/ PD X
figure(5);
simOut = sim('quadcop');
command = simOut.get('trapezoid');
t = simOut.get('time');
plot(t.data, command.data);
title("10.61 - COMMAND W/ PD X");

%%Plot 10.63 Simulink Trajectory
simOut = sim('quadcop');
xTra = simOut.get('xTrajec');
yTra = simOut.get('yTrajec');
figure(6);
plot(xTra.data, yTra.data);
title("10.63- COMMAND W/ WHOLE DRONE");

%%LQR X
Kx = [-316230 -49020 6204 126690 2.66];
Nxx = [0 0 0 1 0]'; 
Nxbar = -3.1623*10^5;
M = Bx*Nxbar;
Lx = [17; 148; -44; -23; -170];

LAx = [Ax -Bx.*Kx; Lx.*[1 0 0 0 0] Ax-Bx.*Kx-Lx.*[1 0 0 0 0]];
LBx = [M; M];
LCx = [[1 0 0 0 0] [0 0 0 0 0]];

%Plot 10.65 - X LQR
figure(7)
tfx = minreal(tf(ss(LAx, LBx, LCx, 0))); %tf of LQRx
step(tfx);
title("10.65 - X LQR");

%%LQR Y
Ky = [316230 51641 7147 139480 2.5];
Ny = [0 0 0 1 0]'; 
Nybar = 3.16*10^5;
My = By*Nybar;
Ly = [38; 733; 261; 38; 52342];

LAy = [Ay -By*Ky; Ly.*[1 0 0 0 0] Ay-By.*Ky-Ly.*[1 0 0 0 0]];
LBy = [My; My];
LCy = [[1 0 0 0 0] [0 0 0 0 0]] ;

%LQR Yaw
Kzrot = [11246 316227];
Nzbar = 3.16*10^5;
Lz = [500; 31];
Mz = Bz*Nzbar;

LAz = [Az -Bz.*Kzrot; Lz.*[0 1] Az-Bz.*Kzrot-Lz.*[0 1]]; 
LBz = [Mz; Mz];
LCz = [0 1 0 0];

%Plot 10.65 - Rotational LQR
figure(8)
tfz = minreal(tf(ss(LAz, LBz, LCz, 0))); %tf of LQRY
step(tfz);
title("10.65 - Rotational LQR");
