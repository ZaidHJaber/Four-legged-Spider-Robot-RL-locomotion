% Zaid H. Jaber <z.h.jaber@outlook.com>
% Princess Sumaya University for Technology
% Created: Sep 2023
% Available online: https://github.com/ZaidHJaber/Four-legged-Spider-Robot-RL-locomotion.git

% Mass Parameters for each part
ServoMass = 13; 
ServoHeadMass =1;
TibiaMass= 2.8;
CoxaMass=3.1;
S_HoldMass=0.275;
FemurMass =5;
BodyMass =148+18; %battery+Lower body
CoverMass=34+25+18; %controller + driver+ upper body 
% Gravity
g = -9.81;
rampAngle = -10;

% Sample Time
Ts = 0.025;  % change to 0.025

% Simulation Time
Tf = 15;
d2r = pi/180;
% Desired height
h_final = 0.020 ; % 20mm  
%x_displacmet of each leg tip from its body-coxa joint 
x_displacment = 40; %mm
% Initial body height 
init_6DOF_highet = h_final;
%calculate the angles using inverse kinematics 
angles = InverseKinematics2(x_displacment,init_6DOF_highet); %<<<


% Initial joint angles and velocities
%angles:.................................................

init_tibiaRL_ang =-angles(1);   %30*d2r;
init_tibiaRR_ang =angles(1);%30*d2r;
init_tibiaFL_ang =angles(1);%30*d2r;
init_tibiaFR_ang =-angles(1);%30*d2r;
init_femurRL_ang=-angles(2);%30*d2r;
init_femurRR_ang=angles(2);%30*d2r;
init_femurFL_ang=angles(2);%30*d2r;
init_femurFR_ang=-angles(2);%30*d2r;
init_coxaRL_ang =-angles(3);%30*d2r;
init_coxaRR_ang =angles(3);%30*d2r;
init_coxaFL_ang =angles(3);%30*d2r;
init_coxaFR_ang =-angles(3);%30*d2r;
angles*180/pi

% initial height


% Initial body speeds in x,z
vx_init = 0;
vz_init = 0;

% Contact friction properties
mu_kinetic = 0.88;
mu_static = 0.9;
v_critical = 0.001;

% Ground properties
ground.stiffness = 1e3;  %1e3
ground.damping = 1e2;    %1e2
TranRegion = 0.01;
ground.length = 10;
ground.width = 1.5;
ground.depth = 0.05;


% Joints properties
joint.stiffness = 0;
joint.damping = 3;                               
joint.limitStiffness = 500;
joint.limitDamping = 50;
joint.transitionWidth = 2 * d2r;

%>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% Define limits on variables
z_min = 0.001; % min height of body 
y_max = 0.15; % max translation in y
vx_max = 0.25; % max horizontal speed of body 
vy_max = 0.25; % max lateral speed of body
vz_max = 0.25; % max vertical speed of body
roll_max =  20* d2r;  % max roll angle of body
pitch_max = 30 * d2r; % max pitch angle of body
yaw_max = 30 * d2r;   % max yaw angle of body
omega_x_max = pi/2; % max angular speed about x
omega_y_max = pi/2;  % max angular speed about y
omega_z_max = pi/2;   % max angular speed about z
%angles limits, 
min_tibiaRL_ang=-60 * d2r; %SV1,SV2 and SV3 joint angle limit
max_tibiaRL_ang= 0 * d2r;                                                               

min_femurRL_ang= 0*d2r; 
max_femurRL_ang = 60*d2r;                                                              

min_coxaRL_ang = -10*d2r; 
max_coxaRL_ang =  33*d2r;

w_max = 2*pi*60/60;                 %joint angular speed limit
z_max = 0.001* (80*cos(max_tibiaRL_ang) - 55*sin(min_femurRL_ang)-23/2);  % max height of body from ground
z_max 
%>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
total_mass = 0.001*((ServoMass+ ServoHeadMass)*12+TibiaMass*4+CoxaMass*4+S_HoldMass*8+FemurMass*4+BodyMass+CoverMass);

normal_force_max = ((total_mass)*abs(g))/4;
friction_force_max = mu_static * normal_force_max;


