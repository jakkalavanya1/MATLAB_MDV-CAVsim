% REV: LAVANYA  MAY 2018
% ONLY ONE MDV ONE CAV with out RSUs
% Writen by: Jackeline Rios-Torres based on Andreas Malikopoulos control
% algorithm
% This script allows visualizing the animation of vehicles on two merging
% roads. The feedback control is done by using Pontryagin's Minimum
% Principle.
% Collision of the vehicles is avoided by imposing the condition that
% each vehicle enters the merging zone only after the previous one has exit
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
R1_xo = [ -150];
R2_xo = [ -150];
%============== PRE-DEFINED VALUES =======================================
%               MANUAL VEHICLE - Secondary Road                 
%========================================================================
a_model=-0.05; %acceleration for manualy driven vehicle(MDV) on secondary road

           % take time headway=time required for MDV to cross merging

           

% for loop to calculate MDV model values
dt=1;
s(1,:)=R2_xo; %distance travelled  
for i=1:length(R2_xo)
    %vf_model(i) = 12 + (14-12).*rand(1) % pre-defining v-final for MDV
    vf_model(i) = 12.7631
    v1_model(i) = sqrt((vf_model(i))^2+2*a_model*s(i)); %initial velocity    
    v100_sr(i)=sqrt((v1_model(i)^2)+(2*a_model*100));  % velocity at RSU2  
    v400_sr(i)=sqrt((v1_model(i)^2)+(2*a_model*400)); % velocity while entering merging
    v430_sr(i)=sqrt((v1_model(i)^2)+(2*a_model*430)); % vel while leaving merging
    tm_sr(i)=(v400_sr(i)-v100_sr(i))/a_model;  % time while entering merging
    tme_sr(i)=(v430_sr(i)-v100_sr(i))/a_model; % time while exit merging
    t_headway(i)=tme_sr(i)-tm_sr(i) 
    v1(1,i)=v1_model(i);
end
  
v2_RSU2= v100_sr;
cz_sr=400;
cz_mr=400;
a_CAV = 0;

%% Initial conditions for each vehicle
% Vehicles on main road
% Identify main road as 1
R1_ini(1,:)= ones(1,length(R1_xo));
% xo: Initial position on x
R1_ini(2,:)= R1_xo; 
% yo: Initial position on y
R1_ini(3,:)= 107.625; 
% xf: Initial position
R1_ini(4,:)= ones(1,length(R1_xo))*cz_length;
%vo: Initial speed
R1_ini(5,:)= ones(1,length(R1_xo))*vavg_mr;
% vf: final speed
R1_ini(6,:)= ones(1,length(R1_xo))*vmax_mr;
% t2iz: Time to intersection zone at constant speed = vavg
R1_ini(7,:)= (cz_length-R1_xo)./R1_ini(5,:); 
R1_ini(8,:)= ones(1,length(R1_xo))*a_CAV;   % store acceleration

% Vehicles on secundary road

% Identify secundary road as 2
R2_ini(1,:)= ones(1,length(R2_xo)).*2; 
% xo: Initial position on x
R2_ini(2,:)= R2_xo; 
% yo: Initial position on y
R2_ini(3,:)= R2_xo*0.25; 
% xf: final position
R2_ini(4,:)= ones(1,length(R2_xo))*(cz_length); 
% vo: Initial speed
R2_ini(5,:)= v1_model; 
% vf: final speed
R2_ini(6,:)= vf_model; 
% tf: time to reach intersection
%R2_ini(7,:)= (cz_length-R2_xo)./R2_ini(5,:); 
R2_ini(7,:)= tm_sr; %tf = time to reach intersection is known from model
R2_ini(8,:)= ones(1,length(R2_xo))*a_model; % assumed deceleration

% Reorganize vehicles in hierarchy according to time to reach control zone
%===========================================


%R2_ini_h = sortrows(R2_ini',7)'; %if there are more vehicles update it by time

%R12_ini = horzcat(R1_ini, R2_ini); % Concatenate the two matrices 
%R12_ini_h = sortrows(R12_ini',7)'; % Ordering by time to reach intersection zone
%=========================================
% Concatenate matrices
R1_ini = horzcat(R1_ini, R2_ini); % Concatenate the two matrices 
R1_ini_h = sortrows(R1_ini',7)'; % Ordering by time to reach intersection zone

% Calculate the time to leave the intersection for each vehicle on the hierarchy  

for i3=1:length(R1_ini_h(7,:))
    if i3==1
        % Since tf=time to reach intersection zone, the time to leave
        % intersection for the first vehicle is the time it takes to cross 
        % the control zone at constant speed=vavg plus the time it takes 
        % to cross the intersection zone at constant speed = vmax
        R1_ini_h(8,i3) = R1_ini_h(4,i3)./R1_ini_h(5,i3); %+(iz_length)./R12_ini_h(6,i3);
    else
        % For the rest of the vehicles it is the time to leave the
        % intersection of the previous vehicle, plus the time it takes to
        % cross the intersection zone at constant speed vmax
        R1_ini_h(8,i3) = R1_ini_h(8,i3-1)+(iz_length)./R1_ini_h(6,i3);
    end
end

%--------------------------------------------------------------------------------
%                   MODEL for MANUAL VEHICLE
%--------------------------------------------------------------------------------

sim_time=100; %simulation time in seconds
theta=atan(0.25); % the inclination of secondary road
%v1(1,:)=v1_model;
for i=1:sim_time
    for j=1:length(R2_ini(1,:))
        if s(i,j)<400    
            u_model(i,j)=a_model;
            s(i+1,j)=v1_model(1,j)*i+(0.5*a_model*i.^2); % distance update
            v1(i+1,j) = sqrt((vf_model(1,j)).^2-(2*a_model*s(i+1,j))); % velocity for each time step
            x_model(i,j)=cos(theta)*s(i,j); % x positions
            y_model(i,j)=x_model(i,j)*0.25; % y psoitions
            t1(i)=i*1; % to record how many times for loop is run to plot
        else  % when vehicle enters main road y position is constant
            u_model(i,j)=a_model;
            s(i+1,j)=v1_model(1,j)*i+(0.5*a_model*i.^2);%but this should go till 530 CHECK
            v1(i+1,j) = sqrt((vf_model(1,j)).^2-(2*a_model*s(i+1,j)));
            x_model(i,j)=400+v1(i+1,j)*(i-tm_sr(j))+(0.5*a_model*(i-tm_sr(j)).^2);
            p=v1(i,j)*(i-tm_sr(j))+(0.5*a_model*(i-tm_sr(j)).^2);
            y_model(i,j)=107.625; 
            t1(i)=i*1;
       end
     
    end
end
%% Define initial conditions

xo=R1_ini_h(2,:); % This is still the initial position to start before the control zone
yo=R1_ini_h(3,:);
xf=R1_ini_h(4,:);
vo=R1_ini_h(5,:);
vf=R1_ini_h(6,:);
to=0.*R1_ini_h(2,:);
tf=R1_ini_h(8,:);
t2exit=R1_ini_h(8,:);
t=zeros(1,length(tf));

% State and control values for t=0
x=xo;
y=yo;
v=vo;
u=0*v;


for i=2:sim_time
    for j=1:length(R1_ini(1,:))
        %v1(i+1,j) = 13.5; % velocity for each time step
        x(i,j) = x(i-1,j)+ 13.5*dt;  % x positions
        y(i,j)=107.625; % y psoitions
        t1(i)=i*1; % to record how many times for loop is run to plot
        %p=v1(i,j)*(i-tm_sr(j));
    end
end

%% Show animation
i4=find(R1_ini_h(1,:)==2); % Identifies secondary road
i5=find(R1_ini_h(1,:)==1); % Identifies main road
x1= x(:,i4); % Secondary road
y1= y(:,i4);
u1= u(:,i4);
v1= v(:,i4);

x2=x(:,i5); % Main road
y2=y(:,i5);      
u2= u(:,i5);
v2= v(:,i5);

% Display results as animation
figure(1)
n=1; 
for n=1:t_sim
    clf
    plot(x2(n,:),y2(n,:),'or','MarkerSize',5, 'MarkerFaceColor','r' );
    hold on
    plot(x1(n,:),y1(n,:),'ob','MarkerSize',5,  'MarkerFaceColor','b');
    hold off
    axis([0,600,-50,150]);
    xlabel('x (m)');
    ylabel('y (m)');
    line([0 700],[111.5 111.5],'color','k','LineWidth',2)
    line([0 400],[103.75 103.75],'color','k','LineWidth',2)
    line([430 700],[103.75 103.75],'color','k','LineWidth',2)
    line([400 400],[100 111.5],'color','r','LineWidth',2,'LineStyle','--')
    line([430 430],[103.75 111.5],'color','r','LineWidth',2,'LineStyle','--')
    
    line([0 400],[3.75 103.75],'color','k','LineWidth',2)
    line([0 430],[-4 103.75],'color','k','LineWidth',2)
    grid on
    title('Vehicles Trajectory');
    drawnow
    pause(0.25)
    M(n)=getframe;
end

figure
axes('Position',[0 0 1500 1500])
movie(M,1)

myVideo= VideoWriter('MergingRoads_Coordinated_DS.avi');
myVideo.FrameRate = 5;  % Default 30
myVideo.Quality = 90;    % Default 75
open(myVideo);
writeVideo(myVideo, M);
close(myVideo);

        