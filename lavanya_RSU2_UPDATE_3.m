% for CAV we assume constant velcoity 13.5m/s
% control zone for both roads= 400m
% merging zone = 30m
% case 1: RSU 1 gives initial velocity, position of manual vehicle to CAV
% at beginning of its control zone

% based on that initial velocity, CAV calculates its time to merge


close all;  % Close all figures (except those of imtool.)
clear;  % Erase all existing variables. Or clearvars if you want.
clc;    % Clear the command window.

vmin_mr = 0;
iz_length = 30; % Length of the merging zone [m]

% positions of vehicles
R1_xo = [-250]; % CAV on main road
R2_xo = [-150]; % MDV on secondary road

%============== PRE-DEFINED VALUES =======================================
%               MANUAL VEHICLE - Secondary Road                 
%========================================================================
a_model=-0.05; %acceleration for manualy driven vehicle(MDV) on secondary road
vf_model = 12 + (14-12).*rand(1) % pre-defining v-final for MDV
s(1,1)=0; %distance travelled 
dt=1;
v1_model(1,1) = sqrt((vf_model)^2+2*a_model*s(1,1)); %initial velocity
v100_sr=sqrt((v1_model(1)^2)+(2*a_model*100))  % velocity at RSU2
v2_RSU2= v100_sr;
v400_sr=sqrt((v1_model(1)^2)+(2*a_model*400)); % velocity while entering merging
v430_sr=sqrt((v1_model(1)^2)+(2*a_model*430)); % vel while leaving merging

tm_100=(v100_sr-v1_model(1,1))/a_model; % TIME at RSU 2
tm_sr=(v400_sr-v100_sr)/a_model  % time while entering merging
tme_sr=(v430_sr-v100_sr)/a_model; % time while exit merging

t_headway=tme_sr-tm_sr;            % take time headway=time required for MDV to cross merging

vavg_sr=v1_model(1,1);
vmax_sr=vf_model;
cz_sr=400;
cz_mr=400;

%===============================================================
%               CAV  - main road
%===============================================================
vavg_mr=13.5; % initial velocity vo
vmax_mr=13.5; % final velocity vf
vf=vmax_mr;
a_CAV=0;      % acceleration = 0 


% Vehicles on main road
% Identify main road as 1

R1_ini(1,:)= ones(1,length(R1_xo)); % index of road=1
R1_ini(2,:)= R1_xo;                 % initial x position
R1_ini(3,:)= 107.625;               % initial y position
R1_ini(4,:)= ones(1,length(R1_xo))*cz_mr; 
R1_ini(5,:)= ones(1,length(R1_xo))*vavg_mr; % initial velocity 
R1_ini(6,:)= ones(1,length(R1_xo))*vmax_mr; % ignore it for now ### final velocity
R1_ini(7,:)= (cz_mr-R1_xo)./R1_ini(5,:);    % time for merging
R1_ini(8,:)= ones(1,length(R1_xo))*a_CAV;   % store acceleration

R1_ini_h = sortrows(R1_ini',7)'; %if there are more vehicles update it by time

% Vehicles on secundary road
% Identify secundary road as 2

R2_ini(1,:)= ones(1,length(R2_xo)).*2;  %index of road
R2_ini(2,:)= R2_xo;                     % xo initial poisition
R2_ini(3,:)= R2_xo*0.25;                % initial y position
R2_ini(4,:)= ones(1,length(R2_xo))*(cz_sr); % final position 
R2_ini(5,:)= ones(1,length(R2_xo))*vavg_sr; % initial vel of MDV 
R2_ini(6,:)= ones(1,length(R2_xo))*vmax_sr; % is pre-defined as vf_model 
R2_ini(7,:)= tm_sr; %tf = time to reach intersection is known from model
R2_ini(8,:)= ones(1,length(R2_xo))*a_model; % assumed deceleration

R2_ini_h = sortrows(R2_ini',7)'; %if there are more vehicles update it by time

%R12_ini = horzcat(R1_ini, R2_ini); % Concatenate the two matrices 
%R12_ini_h = sortrows(R12_ini',7)'; % Ordering by time to reach intersection zone
%--------------------------------------------------------------------------------
%                   MODEL for MANUAL VEHICLE
%--------------------------------------------------------------------------------

sim_time=100; %simulation time in seconds
theta=atan(0.25); % the inclination of secondary road

for i=1:sim_time
    for j=1:length(R2_ini(1,:))
        if s(i,j)<430    
            u_model(i,j)=a_model;
            s(i+1,j)=v1_model(1,1)*i+(0.5*a_model*i^2); % distance update
            v1_model(i,j) = sqrt((vf_model)^2+2*a_model*s(i+1,j)); % velocity for each time step
            x_model(i,j)=cos(theta)*s(i,j); % x positions
            y_model(i,j)=x_model(i,j)*0.25; % y psoitions
        else  % when vehicle enters main road y position is constant
            u_model(i,j)=a_model;
            s(i+1,j)=v1_model(1,1)*i+(0.5*a_model*i^2);%but this should go till 530 CHECK
            v1_model(i,j) = sqrt((vf_model)^2+2*a_model*s(i+1,j));
            x_model(i,j)=430+v1_model(i,j)*(i-tm_sr)+(0.5*a_model*(i-tm_sr)^2);
            y_model(i,j)=107.625; 
       end
     
    end
end

for i1=1:sim_time-1
    for i2=1:length(R1_ini_h(2,:))
        %p(i1+1,i2+1)=sqrt((x(i1+1,i2+1)-x(i1,i2))^2+(y(i1+1,i2+1)-y(i1,i2))^2);
        p(i1+1,1)=sqrt((x_model(i1+1,1)-x_model(i1,1))^2+(y_model(i1+1,1)-y_model(i1,1))^2);
    end
end

%-------------------------------------------------------------------------
%                       CAV control
%-------------------------------------------------------------------------
xo=R1_ini_h(2,:); % initial x position
yo=R1_ini_h(3,:); %
xf=R1_ini_h(4,:); %
vo=R1_ini_h(5,:);
vf=R1_ini_h(6,:); % v final is same as initial
to=0.*R1_ini_h(2,:); % initial time
tf=R1_ini_h(7,:); % ignore tf for now, later it is predicted based on input from RSU1
t2exit=R1_ini_h(7,:); 
t=zeros(1,length(tf));
% initialize State and control values for t=0
x=xo;
y=yo;
v=vo;
u=0*v;

    %gaussian model from RSU1

vf_gaussian=normrnd(v1_model(1,1),1)
a_predict=(vf_gaussian^2-v1_model(1,1)^2)/(2*400); %500 is distance
v2_predict=sqrt(v1_model(1,1).^2+2*a_predict*(100)) % vel at RSU2 predicted using acclf from gaussain
tm_predict=2*(400)/(vf_gaussian+v1_model(1,1))   

vf_newpredict= 12.5; % predict one value between 12 to 14
error=v2_predict-v2_RSU2

for i1=2:sim_time
    for i2=1:length(R1_ini_h(2,:))        
        % Update variables before control zone 
        if x(i1-1,i2)<0
            
            % Update initial conditions:
            xo(i1,i2) = x(i1-1,i2); %##### We want xo to be constant until reaching the control zone
            xf(i1,i2) = xf(i1-1,i2); % We want xf to be constant always
            vo(i1,i2) = v(i1-1,i2); % We want vo to be constant until reaching the control zone
            vf(i1,i2) = vf(i1-1,i2); % We want vf to be constant always
            to(i1,i2) = i1-1; % We want to update to at each instant of time
            tf(i1,i2) = tm_predict; % time taken from RSU 2 
            
            t(i1,i2)=i1*dt; 
            x(i1,i2) = x(i1-1,i2)+ vo(i1,i2)*dt; 
            v(i1,i2) = vo(i1,i2);
            u(i1,i2) = 0;
            flag(i1,i2) = 0; % this flag is to detect that the minimum speed value has been reached
            
            if x(i1,i2)>=0
                xo(i1,i2) = x(i1,i2);
                to(i1,i2) = t(i1,i2);
                vo(i1,i2) = v(i1,i2);
            end
            
        end            
        
        % when vehicle enters merging zone
        
        if x(i1-1,i2)>=0 && x(i1-1,i2)<=400 
            %if R1_ini_h(1,:)==1 %choosing only mr vehcles
            if i2==1
                % Update the initial conditions:
                xo(i1,i2) = x(i1-1,i2);
                xf(i1,i2) = xf(i1-1,i2);
                vo(i1,i2) = v(i1-1,i2);
                
                vf(i1,i2) = vf(i1-1,i2);
                to(i1,i2) = t(i1-1,i2);
                tf(i1,i2) = tm_predict+t_headway; % will change based on RSU 2 update
                
                if tf(i1,i2)-to(i1,i2)<= 2*dt  % when the vechile comes nearer to the
                    a(i1,i2) = a(i1-1,i2);      % merging zone, control will not update
                    b(i1,i2) = b(i1-1,i2);      % instead it is just equal to the previous control
                    c(i1,i2) = c(i1-1,i2);
                    d(i1,i2) = d(i1-1,i2);
                else
                    [a(i1,i2),b(i1,i2),c(i1,i2),d(i1,i2)]=RTControl(to(i1,i2),tf(i1,i2),xo(i1,i2),xf(i1,i2),vo(i1,i2),vf(i1,i2));
                end
                 
                %[a(i1,i2),b(i1,i2),c(i1,i2),d(i1,i2)]=RTControl(to(i1,i2),tf(i1,i2),xo(i1,i2),xf(i1,i2),vo(i1,i2),vf(i1,i2));
                t(i1,i2)=i1*dt;
                x(i1,i2) = a(i1,i2)*t(i1,i2)^3/6 + b(i1,i2)*t(i1,i2)^2/2 + c(i1,i2)*t(i1,i2) + d(i1,i2);
                v(i1,i2) = a(i1,i2)*t(i1,i2)^2/2 + b(i1,i2)*t(i1,i2) + c(i1,i2);
                u(i1,i2) = a(i1,i2)*t(i1,i2) + b(i1,i2);
                % if the vehicle hit the lower bound on speed, don't apply 
                % control. Update states and initial conditions:
                if v(i1,i2)<vmin_mr
                    
                    % Update states and control
                    x(i1,i2) = x(i1-1,i2)+ v(i1-1,i2)*dt;
                    v(i1,i2) = v(i1-1,i2);
                    u(i1,i2) = 0;
                    flag(i1,i2) = 1;
                    
                    % Update initial conditions
                    xo(i1,i2) = x(i1-1,i2);
                    xf(i1,i2) = xf(i1-1,i2);
                    vo(i1,i2) = v(i1-1,i2);
                    vf(i1,i2) = vf(i1-1,i2);
                    to(i1,i2) = t(i1-1,i2);
                    tf(i1,i2) = tf(i1-1,i2)-0.02;                  
                end
                
            else % for following vehicles                
                % Update the initial conditions
                xo(i1,i2) = x(i1-1,i2);
                xf(i1,i2) = xf(i1-1,i2);
                vo(i1,i2) = v(i1-1,i2);
                vf(i1,i2) = vf(i1-1,i2);
                to(i1,i2) = t(i1-1,i2);
                tf(i1,i2) = tm_predict+t_headway;
                
                % Calculate constants. If to is too close to tf, control
                % signal will go to infinity, so, the control should be
                % stopped if the difference between the two is less than 3 seconds
                if tf(i1,i2)-to(i1,i2)<= 2
                    a(i1,i2) = a(i1-1,i2);
                    b(i1,i2) = b(i1-1,i2);
                    c(i1,i2) = c(i1-1,i2);
                    d(i1,i2) = d(i1-1,i2);
                else
                    [a(i1,i2),b(i1,i2),c(i1,i2),d(i1,i2)]=RTControl(to(i1,i2),tf(i1,i2),xo(i1,i2),xf(i1,i2),vo(i1,i2),vf(i1,i2));
                end
                % Update states and control
                t(i1,i2)=i1*dt;
                x(i1,i2) = a(i1,i2)*t(i1,i2)^3/6 + b(i1,i2)*t(i1,i2)^2/2 + c(i1,i2)*t(i1,i2) + d(i1,i2);
                v(i1,i2) = a(i1,i2)*t(i1,i2)^2/2 + b(i1,i2)*t(i1,i2) + c(i1,i2);
                u(i1,i2) = a(i1,i2)*t(i1,i2) + b(i1,i2);
                
                if tf(i1,i2)-to(i1,i2)<= 2 && (v(i1,i2)> vf(1,i2) || v(i1,i2)< vf(1,i2))
                    v(i1,i2) = vf(1,i2);
                    u(i1,i2) = 0;
                    x(i1,i2) = x(i1-1,i2) + vf(1,i2)*dt;
                end
                % if the vehicle hit the lower bound on speed, don't apply 
                % control. Update states and initial conditions
                if v(i1,i2)<vmin_mr 
                    
                    % Update states and control
                    x(i1,i2) = x(i1-1,i2)+ v(i1-1,i2)*dt; % delta t = 1
                    v(i1,i2) = v(i1-1,i2);
                    u(i1,i2) = 0;
                    flag(i1,i2) = 1;
                    
                    % Update initial conditions
                    xo(i1,i2) = x(i1-1,i2);
                    xf(i1,i2) = xf(i1-1,i2);
                    vo(i1,i2) = v(i1-1,i2);
                    vf(i1,i2) = vf(i1-1,i2);
                    to(i1,i2) = t(i1-1,i2);
                    tf(i1,i2) = tf(i1,i2-1) + iz_length/vf(i1,i2-1)-0.02;                    
            %----------------------------------------------------------------               
            end
        end
        
        % vehicle after crossing control zone
        if x(i1-1,i2)>400 
            
                
            
            % Update initial conditions:
            xf(i1,i2) = xf(i1-1,i2); % We want xf to be constant always
            vf(i1,i2) = vf(i1-1,i2); % We want vf to be constant always
            to(i1,i2) = to(i1-1,i2); % We want to=0 always
            tf(i1,i2) = tf(i1-1,i2); % We want tf to keep constant until we enter the control zone
            
            % Update states
            t(i1,i2)=i1*dt;
            x(i1,i2) = x(i1-1,i2)+ vf(i1,i2)*dt;
            v(i1,i2) = vf(i1,i2);
            u(i1,i2) = 0;
        end
        if x(i1-1,i2)==400 % to record time at which CAV merges
            tm_CAV=i1*dt
        end
%         if x(i1,i2)<530 && R12_ini(1,i2)==2
%             y(i1,i2)=x(i1,i2)*0.25;
%         else
        y(i1,i2)=107.625; % y position on main road is same always
        
    end
    %=============== at time when manual reaches RSU 2, the tm will be
    %updated for CAV
    
    if i==tm_100 && error >0.3
        tm_predict=2*300/(vf_newpredict+v2_RSU);
    else
%    vf_consider = vf_model %then the initial predicted can be there as
                            % it does not make much difference
        tm_predict=tm_predict;
    end
    end
end

%% plots
%% plot
% %tm_CAV
% tm_model=
% tm_predicted=


%% Show animation
i4=find(R2_ini(1,:)==2); % Identifies secondary road
i5=find(R1_ini_h(1,:)==1); % Identifies main road
%out put of i4 and i5 is 1
x1= x_model(:,i4); % Secondary road
y1= y_model(:,i4); 
u_model=u_model';
u1= u_model(:,1); % constant
v1_model=v1_model';
v1= v1_model(:,1);

x2=x(:,i5); % Main road
y2=y(:,i5);      
u2= u(:,i5);
v2= v(:,i5);


% Display results as animation
figure(1)
nn=1; 
for n=1:sim_time
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

figure(3)
time=[1:sim_time];
plot(time,v1_model)
hold on;
plot(time,v)
xlabel('position (meters)')
ylabel('velocity (m/s)')
title('position Vs velocity with RSU')
legend('Secondary Road','Main Road')
