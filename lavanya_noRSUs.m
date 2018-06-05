% for CAV we assume constant velcoity 13.5m/s
% control zone for both roads= 400m
% merging zone = 30m
% case 1: RSU 1 gives initial velocity, position of manual vehicle to CAV
% at beginning of its control zone

% based on that initial velocity, CAV calculates its time to merge


close all;  % Close all figures (except those of imtool.)
clear;  % Erase all existing variables. Or clearvars if you want.
clc;    % Clear the command window.

% positions of vehicles
R1_xo = [-250]; % CAV on main road
R2_xo = [-150]; % MDV on secondary road

%============== PRE-DEFINED VALUES =======================================
%               MANUAL VEHICLE - Secondary Road                 
%========================================================================
a_model=-0.1; %acceleration for manualy driven vehicle(MDV) on secondary road
vf_model = 12 + (14-12).*rand(1) % pre-defining v-final for MDV
s(1,1)=0; %distance travelled 
dt=1;
v1_model(1,1) = sqrt((vf_model)^2+2*a_model*s(1,1)); %initial velocity
v100_sr=sqrt((v1_model(1)^2)+(2*a_model*100));  % velocity at RSU2
v2_RSU2= v100_sr;
v400_sr=sqrt((v1_model(1)^2)+(2*a_model*400)); % velocity while entering merging
v430_sr=sqrt((v1_model(1)^2)+(2*a_model*430)); % vel while leaving merging

tm_sr=(v400_sr-v100_sr)/a_model;  % time while entering merging
tme_sr=(v430_sr-v100_sr)/a_model; % time while exit merging

t_headway=tme_sr-tm_sr            % take time headway=time required for MDV to cross merging

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


% predict final velocity and v2 using Gaussian distribution
%vf=normrnd(13.5,1) %try to make it skewd to left
vf_gaussian=normrnd(v1_model(1,1),1);
a_predict=(vf_gaussian^2-v1_model(1,1)^2)/(2*400) % acceleration predicted based on vf_gaussian
tm_predict=2*(400)/(vf_gaussian+v1_model(1,1))    % time for merging is predicted from RSU1 DATA


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
            tf(i1,i2) = tf(i1-1,i2); % time 
            
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
            if R1_ini_h(1,:)==1 %choosing only mr vehcles
                % Update the initial conditions:
                xo(i1,i2) = x(i1-1,i2);
                xf(i1,i2) = xf(i1-1,i2);
                vo(i1,i2) = v(i1-1,i2);
                
                vf(i1,i2) = vf(i1-1,i2);
                to(i1,i2) = t(i1-1,i2);
                tf(i1,i2) = tf(i1-1,i2); % remains same. not updated using rsu
                [a(i1,i2),b(i1,i2),c(i1,i2),d(i1,i2)]=RTControl(to(i1,i2),tf(i1,i2),xo(i1,i2),xf(i1,i2),vo(i1,i2),vf(i1,i2));
                t(i1,i2)=i1*dt;
                x(i1,i2) = a(i1,i2)*t(i1,i2)^3/6 + b(i1,i2)*t(i1,i2)^2/2 + c(i1,i2)*t(i1,i2) + d(i1,i2)
                v(i1,i2) = a(i1,i2)*t(i1,i2)^2/2 + b(i1,i2)*t(i1,i2) + c(i1,i2)
                u(i1,i2) = a(i1,i2)*t(i1,i2) + b(i1,i2);
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
% 
%         if x(i1,i2)<530 && R12_ini(1,i2)==2
%             y(i1,i2)=x(i1,i2)*0.25;
%         else
        y(i1,i2)=107.625; % y position on main road is same always
    end
end

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

                    
             
%             [a(,b,c,d]=RTControl(to(i1,i2),tf,xo,xf,vo,vf);
% [a,b,c,d]=RTControl(to,tf,xo,xf,vo,vf);
% 
% x = a*t^3/6 + b*t^2/2 + c*t+ d;
% v = a*t^2/2 + b*t+ c;
% u = a*t + b;
% ========================== can generte vf_newpredict ==================
% r = 12 + (14-12).*rand(20,1);
% k=1
% for i=length(r)
%     if r(i)<=13.5
%         r_skew(k)=r(i)
%         k=k+1;
%     end
% end
% assign one value of r_skew every time to vf_newpredict
%=======================================================================
vf_newpredict= 12.5;
error=v2_predict-v2_RSU2;
if error >1
    tm_update=2*300/(vf_newpredict+v2_RSU)
else
%     vf_consider = vf_model %then the initial predicted can be there as
                            % it does not make much difference
    tm_update=tm_predict-t_headway
end
% now based on vf _consider predict merging time for CAV


% it gets update of final time so v1_CAV has to be  changed to 
% by the time




%=====================================================
%           SECOND UPDATE
%========================================================
if tm_cav-tm_MDV<= 2*dt %this time should depend on the length of vehicle and time it takes to cross merging zone
    
    [a(i1,i2),b(i1,i2),c(i1,i2),d(i1,i2)]=RTControl(to(i1,i2),tf(i1,i2),xo(i1,i2),xf(i1,i2),vo(i1,i2),vf(i1,i2));
else
%update to the same values as previous
    a(i1,i2) = a(i1-1,i2);
    b(i1,i2) = b(i1-1,i2);
    c(i1,i2) = c(i1-1,i2);
    d(i1,i2) = d(i1-1,i2);

    t(i1,i2)=i1*dt;
    x(i1,i2) = a(i1,i2)*t(i1,i2)^3/6 + b(i1,i2)*t(i1,i2)^2/2 + c(i1,i2)*t(i1,i2) + d(i1,i2);
    v(i1,i2) = a(i1,i2)*t(i1,i2)^2/2 + b(i1,i2)*t(i1,i2) + c(i1,i2);
    u(i1,i2) = a(i1,i2)*t(i1,i2) + b(i1,i2);
end

% after getting input from second road side unit, we have to define again
to
tf
xo
xf
vo
vf

for i=1:t
    
    
    
    
    
end


