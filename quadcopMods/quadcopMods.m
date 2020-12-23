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
tf1 = minreal(tf(a(1,:),b));
tf2 = tf(a(2,:),b);

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
tf1y = minreal(tf(c(1,:),d));
tf2y = tf(c(2,:),d);

%%Z
Az = [0 0; 1 0];
Bz = [1/200; 0];
Cz = [0 1];

%%Simulink Vars
pOuterX = 0.88;
dOuterX = 0.4; 
pOuterY = 0.75;
dOuterY = 0.25;

%%Simulink Trajectory - Mod:1 - Reduce Settling Time(Gain Modification)
kzMod = 10; %this is a gain for the yaw -> makes it respond faster
pd = tf1*500*(s+4);
c2 = 0.4*(s+2.2);
pdx = -minreal(c2*feedback(pd,1)*tf2/tf1);
%rltool(pdx);
kMod = 0.25;%from rltool on pdx -> decrease response time, from 0.12
pdy = tf1y*500*(s+3);
c2y = 0.25*(s+3);
pdOuterY = -minreal(c2y*feedback(pdy,1)*tf2y/tf1y);
%rltool(pdOuterY);
kyMod = 0.1746;

originalX = feedback(pdx*0.12, 1); %used throughout the rest to help compare

simOut = sim('quadcopMod');
xTra = simOut.get('xModTrajec');
yTra = simOut.get('yModTrajec');
figure(9);
plot(xTra.data, yTra.data);
title("Modified Gains - COMMAND W/ WHOLE DRONE");


%%Mod:2 - Reduce Settling Time/Peak Time(PD Changes & Gain Modification)
figure(10);
hold
step(originalX);
stepinfo(originalX)
pd = tf1*500*(s+4);
c2 = 0.3*(s+2.5);
pdx = -minreal(c2*feedback(pd,1)*tf2/tf1);
%rltool(pdx);
kMod = 0.21238;%from rltool on pdx
closedXMod = feedback(pdx*kMod,1);
step(closedXMod);
stepinfo(closedXMod)
title("Modified PD Controller Outer Loop");

%Square Wave Command Test
figure(11);
hold
[u,t] = gensig("square", 35,40);
u = u*100;
lsim(originalX, u, t);
lsim(closedXMod, u, t);
title("Modified PD Controller Outer Loop Command");

%%Mod:3 - (Complex Zero Controller) - Reduce Peak/Settling Time, Reduce
%%overshoot
figure(12);
hold
step(originalX);
stepinfo(originalX)
pd = tf1* 1.8452e08*(s^2 + 4.275*s + 8.483);
c2 = 0.4*(s+2.2);
pdx = -minreal(c2*feedback(pd, 1)*tf2/tf1);
%rltool(pdx);
kMod = 8.23333;%from rltool on pdx
closedXMod = feedback(pdx*kMod, 1);
step(closedXMod);
stepinfo(closedXMod)
title("Modified Complex Zero Controller Outer Loop");

%Square Wave Command Test
figure(13);
hold;
[u,t] = gensig("square", 35,40);
u = u*100;
lsim(originalX, u, t);
lsim(closedXMod, u, t);
title("Modified Complex Zero Controller Outer Loop Command");



