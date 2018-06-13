
% REV: June, 2015

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

R1_xo = [-125 -130];
R2_xo = [-125 -200];


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
R2_ini(5,:)= ones(1,length(R2_xo))*vavg_sr; 
% vf: final speed
R2_ini(6,:)= ones(1,length(R2_xo))*vmax_sr; 
% tf: time to reach intersection
R2_ini(7,:)= (cz_length-R2_xo)./R2_ini(5,:); 

% Reorganize vehicles in hierarchy according to time to reach control zone

% Concatenate matrices
R12_ini = horzcat(R1_ini, R2_ini); % Concatenate the two matrices 
R12_ini_h = sortrows(R12_ini',7)'; % Ordering by time to reach intersection zone

% Calculate the time to leave the intersection for each vehicle on the hierarchy  

for i3=1:length(R12_ini_h(7,:))
    if i3==1
        % Since tf=time to reach intersection zone, the time to leave
        % intersection for the first vehicle is the time it takes to cross 
        % the control zone at constant speed=vavg plus the time it takes 
        % to cross the intersection zone at constant speed = vmax
        R12_ini_h(8,i3) = R12_ini_h(4,i3)./R12_ini_h(5,i3); %+(iz_length)./R12_ini_h(6,i3);
    else
        % For the rest of the vehicles it is the time to leave the
        % intersection of the previous vehicle, plus the time it takes to
        % cross the intersection zone at constant speed vmax
        R12_ini_h(8,i3) = R12_ini_h(8,i3-1)+(iz_length)./R12_ini_h(6,i3);
    end
end

%% Define initial conditions

xo=R12_ini_h(2,:); % This is still the initial position to start before the control zone
yo=R12_ini_h(3,:);
xf=R12_ini_h(4,:);
vo=R12_ini_h(5,:);
vf=R12_ini_h(6,:);
to=0.*R12_ini_h(2,:);
tf=R12_ini_h(8,:);
t2exit=R12_ini_h(8,:);
t=zeros(1,length(tf));

% State and control values for t=0
x=xo;
y=yo;
v=vo;
u=0*v;

% Loop for the simulation time
for i1=2:numel
    
    % Loop for each vehicle
    for i2=1:length(R12_ini_h(1,:))
        
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
        
        % Update variables and control on the control zone
        if x(i1-1,i2)>=0 && x(i1-1,i2)<=400
                        
            % Calculate control for first vehicle:
            if i2==1
                
                % Update the initial conditions:
                xo(i1,i2) = x(i1-1,i2);
                xf(i1,i2) = xf(i1-1,i2);
                vo(i1,i2) = v(i1-1,i2);
                
                vf(i1,i2) = vf(i1-1,i2);
                to(i1,i2) = t(i1-1,i2);
                tf(i1,i2) = tf(i1-1,i2);
                
                if tf(i1,i2)-to(i1,i2)<= 2*dt
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
                
                
            else % for the rest of the vehicles
                
                % Update the initial conditions
                xo(i1,i2) = x(i1-1,i2);
                xf(i1,i2) = xf(i1-1,i2);
                vo(i1,i2) = v(i1-1,i2);
                vf(i1,i2) = vf(i1-1,i2);
                to(i1,i2) = t(i1-1,i2);
                tf(i1,i2) = tf(i1,i2-1) + iz_length/vf(i1,i2-1);
                
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
                end

            end
            
        end

        % Update variables after control zone 
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

        if x(i1,i2)<430 && R12_ini_h(1,i2)==2
            y(i1,i2)=x(i1,i2)*0.25;
        else
            y(i1,i2)=107.625;
        end
    end
end

%% Show animation
i4=find(R12_ini_h(1,:)==2); % Identifies secondary road
i5=find(R12_ini_h(1,:)==1); % Identifies main road
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
nn=1; 
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


                    
                    