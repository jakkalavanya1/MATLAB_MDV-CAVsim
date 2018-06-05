close all;  % Close all figures (except those of imtool.)
clear;  % Erase all existing variables. Or clearvars if you want.
clc;    % Clear the command window.

% positions of vehicles
R1_xo = [-150]; % CAV on main road
R2_xo = [ -150]; % MDV on secondary road

%==============simulating for entire time CREATEING MODEL============
% let the model be pre defined since manaul vehicle
a_model=-0.1;
vf_model = 12 + (14-12).*rand(1);
s2(1,1)=0;
dt=1;
v1_model(1,1) = sqrt((vf_model)^2+2*a_model*s2(1,1));
sim_time=100; %seconds
dx=540/sim_time;

v100_sr=sqrt((v1_model(1)^2)+(2*a_model*100));  % velocity at RSU2
v2_RSU2= v100_sr;

v400_sr=sqrt((v1_model(1)^2)+(2*a_model*500)); 
tm_sr=(v400_sr-v100_sr)/a_model;

vavg_sr=v1_model(1);
vmax_sr=vf_model;
cz_sr=400;
cz_mr=400;
%=========== defining values for main road==============
vavg_mr=7.5;
vf=normrnd(7.5,1) %try to make it skewd to left
vmax_mr=vf;
a_CAV=0.1;
%==================================================================
% Vehicles on main road
% Identify main road as 1
R1_ini(1,:)= ones(1,length(R1_xo)); % index of road
R1_ini(2,:)= R1_xo;                 % 
R1_ini(3,:)= 107.625;               % initial y position
R1_ini(4,:)= ones(1,length(R1_xo))*cz_mr; 
R1_ini(5,:)= ones(1,length(R1_xo))*vavg_mr; % initial velocity 
R1_ini(6,:)= ones(1,length(R1_xo))*vmax_mr; % ignore it for now
R1_ini(7,:)= (cz_mr-R1_xo)./R1_ini(5,:);% time of merging our vel is not constant
%================ s=ut+1/2at^2
R1_ini(8,:)= ones(1,length(R1_xo))*a_CAV;   % gives acceleration

R1_ini_h = sortrows(R1_ini',7)'; %if there are more vehicles

% Vehicles on secundary road
% Identify secundary road as 2
R2_ini(1,:)= ones(1,length(R2_xo)).*2;  %index of road
R2_ini(2,:)= R2_xo;                     % xo initial poisition
R2_ini(3,:)= R2_xo*0.25;                % initial y position
R2_ini(4,:)= ones(1,length(R2_xo))*(cz_sr); % final position xf
R2_ini(5,:)= ones(1,length(R2_xo))*vavg_sr; % initial vel of MDV vo=initial speed
R2_ini(6,:)= ones(1,length(R2_xo))*vmax_sr; %  %FINAL VELOCITY
R2_ini(7,:)= tm_sr; %tf=time to reach intersection is known from model
R2_ini(8,:)= ones(1,length(R1_xo))*a_model; % assumed deceleration

R12_ini = horzcat(R1_ini, R2_ini); % Concatenate the two matrices 
R12_ini_h = sortrows(R12_ini',7)'; % Ordering by time to reach intersection zone


theta=atan(0.25);
for i=1:sim_time
    for j=1:length(R2_ini(1,:))
        if s2(i,j)<430
            u_model(i,j)=a_model;
            s2(i+1,j)=v1_model(1,1)*i+(0.5*a_model*i^2);%but this should go till 530 CHECK
            v1_model(i,j) = sqrt((vf_model)^2+2*a_model*s2(i+1,j));
            x_model(i,j)=cos(theta)*s2(i,j);
            y_model(i,j)=x_model(i,j)*0.25;
        else 
            u_model(i,j)=a_model;
            s2(i+1,j)=v1_model(1,1)*i+(0.5*a_model*i^2);%but this should go till 530 CHECK
            v1_model(i,j) = sqrt((vf_model)^2+2*a_model*s2(i+1,j));
            x_model(i,j)=430+v1_model(i,j)*(i-tm_sr)+(0.5*a_model*(i-tm_sr)^2);
            y_model(i,j)=107.625;
       end
     
    end
end

%===========================================
% CAV as soon as it enters control zone it receives information about MDV
% based on which it decides it vel, accln and time of merging using 
% control
%===============
%============================ PREDICTION OF VEL AT RSU 2 BY CAV(rsu 1 gives this data to CAV)=====
% v1_CAV = 13.5 %let CAV start with certain velocity just 150 meters(R1_xo) outside control zone
% say accln a1=0.1;
% so it will have some tm = time to merge(R1_ini(7,:)) based on the a1 and v1_CAV

% predict final velocity and v2 using Gaussian distribution
%vf=normrnd(13.5,1) %try to make it skewd to left
a_predict=(vf^2-v1_model(1)^2)/(2*400) %500 is distance
v2_preidct=sqrt(v1_model(1).^2+2*a_predict*(100)) % vel at RSU2 predicted using acclf from gaussain
                                % 100 is distance between two RSUs
  %now based on this predicted v2 compare it with actual reading at RSU2
%=================================
xo=R12_ini_h(2,:); % This is still the initial position to start before the control zone
yo=R12_ini_h(3,:);
xf=R12_ini_h(4,:); %merging region
vo=R12_ini_h(5,:);
vf=R12_ini_h(6,:);
to=0.*R12_ini_h(2,:);
tf=R12_ini_h(7,:); %PREDICTED TIME FROM DISTRIBUTION
t2exit=R12_ini_h(7,:); %PREDICTED TIME FROM DISTRIBUTION
t=zeros(1,length(tf));
% State and control values for t=0
x=xo;
y=yo;
v=vo;
u=0*v;

%=========== manual vehicle position===============

for i1=2:sim_time
    for i2=1:length(R1_ini_h(2,:))
        
        % Update variables before control zone 
        if x(i1-1,i2)<0
            
            % Update initial conditions:
            xo(i1,i2) = x(i1-1,i2); % We want xo to be constant until reaching the control zone
            xf(i1,i2) = xf(i1-1,i2); % We want xf to be constant always
            vo(i1,i2) = v(i1-1,i2); % We want vo to be constant until reaching the control zone
            vf(i1,i2) = vf(i1-1,i2); % We want vf to be constant always
            to(i1,i2) = i1-1; % We want to update to at each instant of time
            tf(i1,i2) = tf(i1-1,i2); % We want tf to keep constant until we enter the control zone
            
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
        
        
        
        
        
        if x(i1-1,i2)>=0 && x(i1-1,i2)<=400 
            if R1_ini_h(1,:)==1 %choosing only mr vehcles
                % Update the initial conditions:
                xo(i1,i2) = x(i1-1,i2);
                xf(i1,i2) = xf(i1-1,i2);
                vo(i1,i2) = v(i1-1,i2);
                
                vf(i1,i2) = vf(i1-1,i2);
                to(i1,i2) = t(i1-1,i2);
                tf(i1,i2) = tf(i1-1,i2);
                [a(i1,i2),b(i1,i2),c(i1,i2),d(i1,i2)]=RTControl(to(i1,i2),tf(i1,i2),xo(i1,i2),xf(i1,i2),vo(i1,i2),vf(i1,i2));                t(i1,i2)=i1*dt;
                x(i1,i2) = a(i1,i2)*t(i1,i2)^3/6 + b(i1,i2)*t(i1,i2)^2/2 + c(i1,i2)*t(i1,i2) + d(i1,i2)
                v(i1,i2) = a(i1,i2)*t(i1,i2)^2/2 + b(i1,i2)*t(i1,i2) + c(i1,i2)
                u(i1,i2) = a(i1,i2)*t(i1,i2) + b(i1,i2)
            end
        end
        
        
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
        y(i1,i2)=107.625;
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
% =====================================================================
error=v2_predict-v2_RSU2;
if error >1
    vf_consider=vf %take final velociy predicted from gaussian
else
    vf_consider = vf_model %then the initial predicted can be there as
                            % it does not make much difference
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

% animation part
i4=find(R12_ini_h(1,:)==2); % Identifies secondary road
i5=find(R12_ini_h(1,:)==1); % Identifies main road
x1= x(:,i4); % Secondary road
y1= y(:,i4);
u1= u(:,i4);
v1= v(:,i4);
