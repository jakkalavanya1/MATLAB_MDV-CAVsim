% mixed traffic scenario
% 2 CAVs and one MDV on each road

% while giving input of position also specify CAV/MDV

s.a = {[-150 200 300],'c',[1 2 1]}
s.b = {[-155 205 305],'m', [2 1 2]}

field = 'f';
value = {'some text';
         [10, 20, 30];
         magic(5)};
s = struct(field,value)

road_1 = 'a';
road_2 = 'b';
value = {[-150 200 300]; [3 4 3]};
s1 = struct(road_1,value);
s2 = struct(road_2,value);

R1_ini(1,:) = s1(1).a
R1_ini(2,:) = s1(2).a


 %%

 
clear; 
clear all
close all; 
clc

% vmax_2, vmax_mr and vmax_sr, are to be used only in the case we want to
% set different thresholds for the final speed which can be used
% according to the traffic level. For now, we only allow the
% vehicles reaching the end of the control zone at v=vavg

dt=1;
vmax_2 = 13.4112; % [m/s] ==> 30 MPH
vmax_mr = 13.4112; % [m/s] ==> 30 MPH
vmax_sr = 13.4112;   
vavg_mr = 13.4112; % [m/s] ==> 30 MPH
vavg_sr = 13.4112; % 30 MPH

vmin_mr = 0; % [m/s] 
vmin_sr = 0;
vmax = 30; 

uavg = 0.3; 
cz_length = 400; % Length of the control zone [m]
iz_length = 30; % Length of the merging zone [m], or desired intervehicular distance at the end of the control zone
t_sim = 80; % Simulation time [s]
yo_mr = 107.625; % initial position on y
numel = t_sim/dt;

%% Define initial position for the vehicles on each road

% Aleatory intevehicular distance

R1_xo = {[-125 -150],[3,4]}; % 3= CAV, 4= MANUAL
R2_xo = {[ -150 -200],[4,4]};
road_1 = 'r1';
road_2 = 'r2';
s1 = struct(road_1,R1_xo);
s2 = struct(road_2,R2_xo);


%% Initial conditions for each vehicle

% Vehicles on main road

% Identify main road as 1
R1_ini(1,:)= ones(1,length(s1(1).r1));
% xo: Initial position on x
R1_ini(2,:)= s1(1).r1; % x position 
% yo: Initial position on y
R1_ini(3,:)= 107.625; 
% xf: Initial position
R1_ini(4,:)= ones(1,length(R1_xo))*cz_length;
%vo: Initial speed
R1_ini(5,:)= ones(1,length(R1_xo))*vavg_mr; % CHANGE
% vf: final speed
R1_ini(6,:)= ones(1,length(R1_xo))*vmax_mr; % CHANGE
% t2iz: Time to intersection zone at constant speed = vavg
R1_ini(7,:)= (cz_length-s1(1).r1)./R1_ini(5,:); % CHANGES
R1_ini(8,:)=s1(2).r1;
% Vehicles on secundary road

% Identify secundary road as 2
R2_ini(1,:)= ones(1,length(s2(1).r2)).*2; 
% xo: Initial position on x
R2_ini(2,:)= s2(1).r2; 
% yo: Initial position on y
R2_ini(3,:)= s2(1).r2*0.25; 
% xf: final position
R2_ini(4,:)= ones(1,length(R2_xo))*(cz_length); 
% vo: Initial speed
R2_ini(5,:)= ones(1,length(R2_xo))*vavg_sr; 
% vf: final speed
R2_ini(6,:)= ones(1,length(R2_xo))*vmax_sr; 
% tf: time to reach intersection
R2_ini(7,:)= (cz_length-s2(1).r2)./R2_ini(5,:); 
R2_ini(8,:)=s2(2).r2;
% Reorganize vehicles in hierarchy according to time to reach control zone

% Concatenate matrices
R12_ini = horzcat(R1_ini, R2_ini); % Concatenate the two matrices 
R12_ini_h = sortrows(R12_ini',7)'; % Ordering by time to reach intersection zone
% R12_ini_h = sortrows(R12_ini',[2],{'descend'})';


%% Define initial conditions

xo=R12_ini_h(2,:); % This is still the initial position to start before the control zone
yo=R12_ini_h(3,:);
xf=R12_ini_h(4,:);
vo=R12_ini_h(5,:);
vf=R12_ini_h(6,:);
to=0.*R12_ini_h(2,:);
% tf=R12_ini_h(8,:);
% t2exit=R12_ini_h(8,:);
t=zeros(1,length(tf));
