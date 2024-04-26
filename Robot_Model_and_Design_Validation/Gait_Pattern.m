% Zaid H. Jaber <z.h.jaber@outlook.com>
% Princess Sumaya University for Technology
% Available online: https://github.com/ZaidHJaber/Four-legged-Spider-Robot-RL-locomotion.git
% 2024
% Trajectory Setup to control the x,z coordinates of each leg 
% number of steps moving forward
number_of_steps = 5; 
%velocity boundary conditions 
Velocity_boundary_conditions = zeros( 2, 6*number_of_steps );
%x and z coordainates Sequance of RLleg
RLstepx = [-10, 0, 40, 40, 0,-10];
RLstepz = [ 0, 0 , 0, 0, 10, 0 ];
%x and z coordainates Sequance of RRleg
RRstepx = [ 40, 0,-10,-10, 0, 40];
RRstepz = [ 0, 10, 0, 0, 0 ,0 ];
%x and z coordainates Sequance of FRleg
FRstepx = [ 40, 0,-10,-10, 0, 40]; 
FRstepz = [ 0, 0 , 0, 0, 10,0];
%x and z coordainates Sequance of FLleg
FLstepx = [-10, 0, 40, 40, 0,-10];
FLstepz = [ 0, 10, 0, 0, 0 ,0];
%time sequance to control one step
SteptimeSeries =[ 0.2, 0.4,0.6,0.8,1,1.2];
%x and z coordainates Sequance of each leg based on the number of steps 
RLwaypoints = [repmat(RLstepx,1,number_of_steps);repmat(RLstepz,1,number_of_steps)];
RRwaypoints = [repmat(RRstepx,1,number_of_steps);repmat(RRstepz,1,number_of_steps)];
FRwaypoints = [repmat(FRstepx,1,number_of_steps);repmat(FRstepz,1,number_of_steps)];
FLwaypoints = [repmat(FLstepx,1,number_of_steps);repmat(FLstepz,1,number_of_steps)];
%simulation time based on the number of steps 
timeSeries =0.2:0.2:(number_of_steps*1.2);
