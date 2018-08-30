% mixed traffic scenario-WORKING
% 2 CAVs and one MDV on each road
% need to add rear end collision constraint for manual vehicles
% CAVs should sense rear end collision with MDV

% created on AUGUST 21
% working on animation using handlers
%  all input combinations are working but plot needs to be changed AUG 25
%%
% clears workspace, closes all figures, and clears command window
clear;
clear all
close all;
clc

% define parameters
dt=1;               % time step
vmax_2 = 13.4112;   % [m/s] ==> 30 MPH mr=main road, sr= secondary road
vmax_mr = 13.4112;  % [m/s] ==> 30 MPH
vmax_sr = 13.4112;
vavg_mr = 13.4112;  % [m/s] ==> 30 MPH
vavg_sr = 13.4112;  % 30 MPH
Ssafe = 3;          % [meters]safe distance
% changed Ssafe value from 10 to 3 to avoid the velocity value going to -2
vmin_mr = 0; % [m/s]
vmin_sr = 0;

uavg = 0.3;         % [m/s^2] acceleration
cz_length = 400;    % Length of the control zone [m]
iz_length = 30;     % Length of the merging zone [m], or desired intervehicular distance at the end of the control zone
t_sim = 80;         % Simulation time [s]
yo_mr = 107.625;    % initial position on y-axis
numel = t_sim/dt;   % Time step

% Define initial position for the vehicles on each road
% R1_xo = {[-125 -130],[4,3]};           % 3= CAV, 4= MANUAL % error
% R2_xo = {[ -125 -130],[3,4]};
% R1_xo = {[-125 -130 -140],[3,4,3]};    % 3= CAV, 4= MANUAL %error
% R2_xo = {[ -125 -150],[3,4]};
R1_xo = {[-125 -150],[4,3]};             % 3= CAV(blue), 4= MANUAL(red)
R2_xo = {[-125 -155 ],[4,4]};
road_1 = 'r1';
road_2 = 'r2';
s1 = struct(road_1,R1_xo);
s2 = struct(road_2,R2_xo);

% vehicles on main road
% Identify main road as 1
R1_ini(1,:)= ones(1,length(s1(1).r1)); % gets number of vehicles in road 1
% xo: Initial position on x
R1_ini(2,:)= s1(1).r1;                 % takes only positions(X) of the vehicles on road 1
% yo: Initial position on y
R1_ini(3,:)= 107.625;
% xf: Initial position
R1_ini(4,:)= ones(1,length(s1(1).r1))*cz_length;
%vo: Initial speed
R1_ini(5,:)= ones(1,length(s1(1).r1))*vavg_mr; % CHANGE % it is just like a reference value because it will be different for MDV
% vf: final speed
R1_ini(6,:)= ones(1,length(s1(1).r1))*vmax_mr; % CHANGE
% t2iz: Time to intersection zone at constant speed = vavg
R1_ini(7,:)= (cz_length-s1(1).r1)./R1_ini(5,:); % CHANGES
% gives approximate time to enter merging zone % its okay to consider
% this because our problem is single lane. no overtakes so
% definitely vehicles reach based onthe order of positions they start
R1_ini(8,:)=s1(2).r1; % identifies if vehicle is CAV or MDV


% vehicles on secondary road
R2_ini(1,:)= ones(1,length(s2(1).r2)).*2;
% xo: Initial position on x
R2_ini(2,:)= s2(1).r2;
% yo: Initial position on y
R2_ini(3,:)= s2(1).r2*0.25;
% xf: final position
R2_ini(4,:)= ones(1,length(s2(1).r2))*(cz_length);
% vo: Initial speed
R2_ini(5,:)= ones(1,length(s2(1).r2))*vavg_sr;
% vf: final speed
R2_ini(6,:)= ones(1,length(s2(1).r2))*vmax_sr;
% tf: time to reach intersection
R2_ini(7,:)= (cz_length-s2(1).r2)./R2_ini(5,:);
R2_ini(8,:)=s2(2).r2;% identifies if vehicle is CAV or MDV
% Reorganize vehicles in hierarchy according to time to reach control zone

% Concatenate matrices
R12_ini = horzcat(R1_ini, R2_ini); % Concatenate the two matrices
R12_ini_h = sortrows(R12_ini',7)';

manualFlag = false;     % new variable to check for manual vehicle AUG 19

%               MANUAL VEHICLE
%========================================================================
a_model=-0.05; %acceleration for manualy driven vehicle(MDV) on secondary road
% for loop to calculate MDV model values for more than one vehicle


for i = 1:length(R12_ini_h(2,:))
    if R12_ini_h(8,i)==4 % manual vehicle
        s(i)=R12_ini_h(2,i)
        manualFlag = true; % flag set for manual vehicle AUG 19
        %j=j+1;
    end
end

if manualFlag==true     % checking for manual vehicle AUG 19
    for i=1:length(s)
        vf_model(i) = 12 + (14-12).*rand(1) % pre-defining v-final for MDV
        %vf_model(i) = 12.6808
        v1_model(i) = sqrt((vf_model(i))^2+2*a_model*s(i)); %initial velocity
        v100_sr(i)=sqrt((v1_model(i)^2)+(2*a_model*100));  % velocity at RSU2
        v400_sr(i)=sqrt((v1_model(i)^2)+(2*a_model*400)); % velocity while entering merging
        v430_sr(i)=sqrt((v1_model(i)^2)+(2*a_model*430)); % vel while leaving merging
        tm_sr(i)=(v400_sr(i)-v100_sr(i))/a_model;  % time while entering merging
        tme_sr(i)=(v430_sr(i)-v100_sr(i))/a_model; % time while exit merging
        t_headway(i)=tme_sr(i)-tm_sr(i);
        v1(1,i)=v1_model(i);
    end
    v2_RSU2= v100_sr;
end


%% Initial conditions for each vehicle
% this loop calculates the time to leave the intersection zone
for i3=1:length(R12_ini_h(7,:))
    if i3==1
        % Since tf=time to reach intersection zone, the time to leave
        % intersection for the first vehicle is the time it takes to cross
        % the control zone at constant speed=vavg plus the time it takes
        % to cross the intersection zone at constant speed = vmax
        R12_ini_h(9,i3) = R12_ini_h(4,i3)./R12_ini_h(5,i3) +(iz_length)./R12_ini_h(6,i3);
    else
        % For the rest of the vehicles it is the time to leave the
        % intersection of the previous vehicle, plus the time it takes to
        % cross the intersection zone at constant speed vmax
        R12_ini_h(9,i3) = R12_ini_h(9,i3-1)+(iz_length)./R12_ini_h(6,i3);
    end
end

%% Define initial conditions

xo=R12_ini_h(2,:); % This is still the initial position to start before the control zone
yo=R12_ini_h(3,:);
xf=R12_ini_h(4,:);
vo=R12_ini_h(5,:);
vf=R12_ini_h(6,:);
to=0.*R12_ini_h(2,:);
tf=R12_ini_h(9,:);
t2exit=R12_ini_h(9,:);
t=zeros(1,length(tf));
% State and control values for t=0
x=xo;
y=yo;
v=vo;
u=0*v;

% vf_gaussian=normrnd(v1_model(1,1),1)
% a_predict=(vf_gaussian^2-v1_model(1,1)^2)/(2*400); %500 is distance
% v2_predict=sqrt(v1_model(1,1).^2+2*a_predict*(100)) % vel at RSU2 predicted using acclf from gaussain
% tm_predict=2*(400)/(vf_gaussian+v1_model(1,1))
%
% vf_newpredict= 12.5; % predict one value between 12 to 14
% % error=v2_predict-v2_RSU2

i5=find(R12_ini_h(1,:)==1); % Identifies main road AUG 7
q1=1; % we define q as an index for i5 AUG 7
z=length(i5); % generalize (to check the previous car on same road) for any number of cars given AUG 20
% Loop for the simulation time

% mainRoad(numel,R12_ini_h,dt,vf_model,xo,yo,xf,vo,vf,to,tf,t,x,y,v,u,i5,q)
% function[xo,yo,xf,vo,vf,to,tf,t,x,y,v,u] = mainRoad(numel,R12_ini_h,dt,vf_model,xo,yo,xf,vo,vf,to,tf,t,x,y,v,u,i5,q)
for i1=2:numel
    % Loop for each vehicle
    for i2=1:length(R12_ini_h(1,:))
        if mod(q1,z+1)==0 % check q modulus 3 so that for alternate cars are taken in i5(q-1) AUG 19
            q1=1;
        end
        if R12_ini_h(1,i2)==1 % if main road and q is used to check if previous vehicle(CAV/MDV) on main road
            if R12_ini_h(8,i2)== 3  % if CAV
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
                    % REAR END COLLISION CONSTRAINT--------------------
                    if(i2~=1)
                        
                        if x(i1,i2)-x(i1,i5(q1-1))<Ssafe
                            v(i1,i2)=v(i1,i5(q1-1))-2;
                        end
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
                        
                        %q=q+1; % increase q because one car exists and we go to next car in the next i2 loop
                    else % for the rest of the vehicles
                        %if q== 1
                        %q=2; % used this if condition because when cav is 2nd car (q-1)value is becomeing 0 AUG 13
                        %end
                        % Update the initial conditions
                        %disp(q);
                        xo(i1,i2) = x(i1-1,i2);
                        xf(i1,i2) = xf(i1-1,i2);
                        vo(i1,i2) = v(i1-1,i2);
                        vf(i1,i2) = vf(i1-1,i2);
                        to(i1,i2) = t(i1-1,i2);
                        tf(i1,i2) = tf(i1,i5(q1-1)) + iz_length/vf(i1,i5(q1-1));
                        %tf(i1,i2) = tf(i1,i2-1) + iz_length/vf(i1,i2-1);
                        %changed i2-1(any previous vehicle) to previous main
                        %road vehicle (i5(q-1)) AUG 7
                        % Calculate constants. If to is too close to tf, control
                        % signal will go to infinity, so, the control should be
                        % stopped if the difference between the two is less than 3 seconds
                        if tf(i1,i2)-to(i1,i2)<= 2
                            a(i1,i2) = a(i1-1,i2);
                            b(i1,i2) = b(i1-1,i2);
                            c(i1,i2) = c(i1-1,i2);
                            d(i1,i2) = d(i1-1,i2);
                            
                        else
                            %disp('values for RTcontrol')
                            %disp(to)
                            %disp(tf)
                            [a(i1,i2),b(i1,i2),c(i1,i2),d(i1,i2)]=RTControl(to(i1,i2),tf(i1,i2),xo(i1,i2),xf(i1,i2),vo(i1,i2),vf(i1,i2));
                        end
                        % Update states and control
                        t(i1,i2)=i1*dt;
                        x(i1,i2) = a(i1,i2)*t(i1,i2)^3/6 + b(i1,i2)*t(i1,i2)^2/2 + c(i1,i2)*t(i1,i2) + d(i1,i2);
                        v(i1,i2) = a(i1,i2)*t(i1,i2)^2/2 + b(i1,i2)*t(i1,i2) + c(i1,i2);
                        u(i1,i2) = a(i1,i2)*t(i1,i2) + b(i1,i2);
                        % EXPLAIN
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
                            tf(i1,i2) = tf(i1,i5(q1-1)) + iz_length/vf(i1,i5(q1-1))-0.02;
                            %tf(i1,i2) = tf(i1,i2-1) + iz_length/vf(i1,i2-1)-0.02;
                            %changed i2-1(any previous vehicle) to previous main
                            %road vehicle (i5(q-1)) AUG 7
                        end
                        %                         q=q+1; % increase q to next vehicle number AUG 7
                        % REAR END COLLISION CONSTRAINT--------------------
                        if(i2~=1)
                            
                            if x(i1,i2)-x(i1,i5(q1-1))<Ssafe
                                v(i1,i2)=v(i1,i5(q1-1))-2;
                            end
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
                    
                    % REAR END COLLISION CONSTRAINT--------------------
                    if(i2~=1)
                        
                        if x(i1,i2)-x(i1,i5(q1-1))<Ssafe
                            v(i1,i2)=v(i1,i5(q1-1))-2;
                        end
                    end
                end
                
                if x(i1,i2)<430 && R12_ini_h(1,i2)==2
                    y(i1,i2)=x(i1,i2)*0.25;
                else
                    y(i1,i2)=107.625;
                end
            end
            if R12_ini_h(8,i2)== 4 % if MDV  ------------------------------------
                %disp(x(i1-1,i2))
                if x(i1-1,i2)<0
                    % Update initial conditions:
                    xo(i1,i2) = x(i1-1,i2); % We want xo to be constant until reaching the control zone
                    xf(i1,i2) = xf(i1-1,i2); % We want xf to be constant always
                    vo(i1,i2) = v(i1-1,i2);
                    vf(i1,i2) = vf(i1-1,i2);
                    to(i1,i2) = i1-1; % We want to update to at each instant of time
                    tf(i1,i2) = tf(i1-1,i2); % We want tf to keep constant until we enter the control zone
                    
                    t(i1,i2)=i1*dt;
                    x(i1,i2) = x(i1-1,i2)+ vo(i1,i2)*dt;
                    v(i1,i2) = vo(i1,i2);
                    u(i1,i2) =-0.05;
                    flag(i1,i2) = 0; % this flag is to detect that the minimum speed value has been reached
                    % rear end collision constraint ---------------
                    if(i2~=1) % i2 is number of vehicle
                        %                         if (R12_ini_h(1,i2-1)==1 && R12_ini_h(1,i2)==1)
                        % i have removed  || (R12_ini_h(1,i2-1)==2 && R12_ini_h(1,i2)==2) this part of condition so
                        % that it doesnt go into or condition AUGUST 7
                        %if previous and current vehicle are on same road(main
                        %road or secondary)
                        
                        if x(i1,i2)-x(i1,i5(q1-1))<Ssafe
                            v(i1,i2)=v(i1,i5(q1-1))-2;
                        end
                    end
                    if x(i1,i2)>=0
                        xo(i1,i2) = x(i1,i2);
                        to(i1,i2) = t(i1,i2);
                        vo(i1,i2) = v(i1,i2);
                    end
                    
                end
                %disp(x(i1-1,i2))
                % Update variables and control on the control zone
                if x(i1-1,i2)>=0 && x(i1-1,i2)<=430
                    
                    % Calculate control for first vehicle:
                    if i2==1
                        % Update the initial conditions:
                        xo(i1,i2) = x(i1-1,i2);
                        xf(i1,i2) = xf(i1-1,i2);
                        vo(i1,i2) = v(i1-1,i2);
                        
                        % vmf(i1,i2) = vmf(i1-1,i2);% CHECK THIS
                        %vmf(i1,i2) = 12.7631; % from distribution
                        vf(i1,i2) = vf_model(1,i2);
                        to(i1,i2) = t(i1-1,i2);
                        tf(i1,i2) = tf(i1-1,i2);
                        
                        % Update states and control
                        t(i1,i2)=i1*dt;
                        u(i1,i2) = -0.05;
                        v(i1,i2) = sqrt((vf(i1,i2)).^2-(2*u(i1,i2)*(430-xo(i1,i2))));
                        x(i1,i2) = xo(i1,i2)+v(i1,i2)*dt+(0.5*u(i1,i2)*dt^2);
                        
                    else % for rest of the vehicles
                        % Update the initial conditions
                        xo(i1,i2) = x(i1-1,i2);
                        xf(i1,i2) = xf(i1-1,i2);
                        vo(i1,i2) = v(i1-1,i2);
                        
                        vf(i1,i2) = vf_model(1,i2); % from distribution
                        to(i1,i2) = t(i1-1,i2);
                        tf(i1,i2) = tf(i1,i5(q1-1))+ iz_length/vf(i1,i5(q1-1));
                        
                        % Calculate constants. If to is too close to tf, control
                        % signal will go to infinity, so, the control should be
                        % stopped if the difference between the two is less than 3 seconds
                        
                        % Update states and control
                        t(i1,i2)=i1*dt;
                        u(i1,i2) = -0.05;
                        v(i1,i2) = sqrt((vf(i1,i2)).^2-(2*u(i1,i2)*(430-xo(i1,i2))));
                        x(i1,i2) = xo(i1,i2)+v(i1,i2)*dt+(0.5*u(i1,i2)*dt^2);
                        % rear end collision constraint ---------------
                        %  if (R12_ini_h(1,i2-1)==1 && R12_ini_h(1,i2)==1) % || (R12_ini_h(1,i2-1)==2 && R12_ini_h(1,i2)==2)
                        if(i2~=1)
                            if x(i1,i2)-x(i1,i5(q1-1))<Ssafe
                                v(i1,i2)=v(i1,i5(q1-1))-2;
                            end
                        end
                        %                         end
                    end
                end
                %Update variables after control zone
                if x(i1-1,i2)>430
                    % Update initial conditions:
                    xf(i1,i2) = xf(i1-1,i2); % We want xf to be constant always
                    vf(i1,i2) = vf(i1-1,i2); % CHECK THIS GIVE some value
                    to(i1,i2) = to(i1-1,i2); % We want to=0 always
                    tf(i1,i2) = tf(i1-1,i2); % We want tf to keep constant until we enter the control zone
                    
                    % Update states
                    t(i1,i2)=i1*dt;
                    x(i1,i2) = x(i1-1,i2)+ vf(i1,i2)*dt;
                    v(i1,i2) = vf(i1,i2);
                    u(i1,i2) = -0.05;
                end
                % rear end collision constraint ---------------
                if(i2~=1)
                    %                     if (R12_ini_h(1,i2-1)==1 && R12_ini_h(1,i2)==1)
                    % added the if condition above here AUGUST 7
                    if x(i1,i2)-x(i1,i5(q1-1))<Ssafe
                        v(i1,i2)=v(i1,i5(q1-1))-2;
                    end
                end
                %                 end
                if x(i1,i2)<430 && R12_ini_h(1,i2)==2
                    y(i1,i2)=x(i1,i2)*0.25;
                else
                    y(i1,i2)=107.625;
                end
            end % for loop of MDV or CAV
            q1=q1+1;  % increment for q when there is a vehicle in main road
        end % for the if loop of main road
    end
end


%=========================================================
xmo=R12_ini_h(2,:); % This is still the initial position to start before the control zone
ymo=R12_ini_h(3,:);
xmf=R12_ini_h(4,:);
vmo=R12_ini_h(5,:);
vmf=R12_ini_h(6,:);
tmo=0.*R12_ini_h(2,:);
tmf=R12_ini_h(9,:);
tm2exit=R12_ini_h(9,:);
tm=zeros(1,length(tmf));
% State and control values for t=0
xm=xmo;
ym=ymo;
vm=vmo;
um=0*vm;


i4=find(R12_ini_h(1,:)==2); % Identifies vehicles on secondary road AUG 21
q2=1; % we define q2 as an index for i4 AUG 21
z2=length(i5); % generalize (to check the previous car on same road) for any number of cars given AUG 21


%  Loop for the simulation time
for i1=2:numel
    
    % Loop for each vehicle
    for i2=1:length(R12_ini_h(1,:))
        if mod(q2,z2+1)==0 % check q modulus 3 so that for alternate cars are taken in i4(q2-1) AUG 21
            q2=1;
        end
        if R12_ini_h(1,i2)==2 % if secondary road
            if R12_ini_h(8,i2)== 3 % if CAV
                % Update variables before control zone
                if xm(i1-1,i2)<0
                    % Update initial conditions:
                    xmo(i1,i2) = xm(i1-1,i2); % We want xo to be constant until reaching the control zone
                    xmf(i1,i2) = xmf(i1-1,i2); % We want xf to be constant always
                    vmo(i1,i2) = vm(i1-1,i2); % We want vo to be constant until reaching the control zone
                    vmf(i1,i2) = vmf(i1-1,i2); % We want vf to be constant always
                    tmo(i1,i2) = i1-1; % We want to update to at each instant of time
                    tmf(i1,i2) = tmf(i1-1,i2); % We want tf to keep constant until we enter the control zone
                    % tmf(i1,i2)=tf(i1-1,i2) is changed to above line AUGUST 7
                    tm(i1,i2)=i1*dt;
                    xm(i1,i2) = xm(i1-1,i2)+ vmo(i1,i2)*dt;
                    vm(i1,i2) = vmo(i1,i2);
                    um(i1,i2) = 0;
                    flag(i1,i2) = 0; % this flag is to detect that the minimum speed value has been reached
                    
                    if xm(i1,i2)>=0
                        xmo(i1,i2) = xm(i1,i2);
                        tmo(i1,i2) = tm(i1,i2);
                        vmo(i1,i2) = vm(i1,i2);
                    end
                end
                % Update variables and control on the control zone
                if xm(i1-1,i2)>=0 && xm(i1-1,i2)<=400
                    % Calculate control for first vehicle:
                    if i2==2
                        % Update the initial conditions:
                        xmo(i1,i2) = xm(i1-1,i2);
                        xmf(i1,i2) = xmf(i1-1,i2);
                        vmo(i1,i2) = vm(i1-1,i2);
                        
                        vmf(i1,i2) = vmf(i1-1,i2);
                        tmo(i1,i2) = tm(i1-1,i2);
                        tmf(i1,i2) = tmf(i1-1,i2);
                        
                        if tmf(i1,i2)-tmo(i1,i2)<= 2*dt
                            a(i1,i2) = a(i1-1,i2);
                            b(i1,i2) = b(i1-1,i2);
                            c(i1,i2) = c(i1-1,i2);
                            d(i1,i2) = d(i1-1,i2);
                        else
                            [a(i1,i2),b(i1,i2),c(i1,i2),d(i1,i2)]=RTControl(tmo(i1,i2),tmf(i1,i2),xmo(i1,i2),xmf(i1,i2),vmo(i1,i2),vmf(i1,i2));
                        end
                        % Update states and control
                        tm(i1,i2)=i1*dt;
                        xm(i1,i2) = a(i1,i2)*tm(i1,i2)^3/6 + b(i1,i2)*tm(i1,i2)^2/2 + c(i1,i2)*tm(i1,i2) + d(i1,i2);
                        vm(i1,i2) = a(i1,i2)*tm(i1,i2)^2/2 + b(i1,i2)*tm(i1,i2) + c(i1,i2);
                        um(i1,i2) = a(i1,i2)*tm(i1,i2) + b(i1,i2);
                        % if the vehicle hit the lower bound on speed, don't apply
                        % control. Update states and initial conditions:
                        if vm(i1,i2)<vmin_mr
                            
                            % Update states and control
                            xm(i1,i2) = xm(i1-1,i2)+ vm(i1-1,i2)*dt;
                            vm(i1,i2) = vm(i1-1,i2);
                            um(i1,i2) = 0;
                            flag(i1,i2) = 1;
                            
                            % Update initial conditions
                            xmo(i1,i2) = xm(i1-1,i2);
                            xmf(i1,i2) = xmf(i1-1,i2);
                            vmo(i1,i2) = vm(i1-1,i2);
                            vmf(i1,i2) = vmf(i1-1,i2);
                            tmo(i1,i2) = tm(i1-1,i2);
                            tmf(i1,i2) = tmf(i1-1,i2)-0.02;
                        end
                        
                    else % for the rest of the vehicles
                        % Update the initial conditions
                        xmo(i1,i2) = xm(i1-1,i2);
                        xmf(i1,i2) = xmf(i1-1,i2);
                        vmo(i1,i2) = vm(i1-1,i2);
                        vmf(i1,i2) = vmf(i1-1,i2);
                        tmo(i1,i2) = tm(i1-1,i2);
                        tmf(i1,i2) = tmf(i1,i4(q2-1)) + iz_length/vmf(i1,i4(q2-1));
                        % tmf(i1,i2) = tmf(i1,i2-1) +
                        % iz_length/vmf(i1,i2-1);--------------
                        % Calculate constants. If to is too close to tf, control
                        % signal will go to infinity, so, the control should be
                        % stopped if the difference between the two is less than 3 seconds
                        if tmf(i1,i2)-tmo(i1,i2)<= 2
                            a(i1,i2) = a(i1-1,i2);
                            b(i1,i2) = b(i1-1,i2);
                            c(i1,i2) = c(i1-1,i2);
                            d(i1,i2) = d(i1-1,i2);
                        else
                            [a(i1,i2),b(i1,i2),c(i1,i2),d(i1,i2)]=RTControl(tmo(i1,i2),tmf(i1,i2),xmo(i1,i2),xmf(i1,i2),vmo(i1,i2),vmf(i1,i2));
                        end
                        % Update states and control
                        tm(i1,i2)=i1*dt;
                        xm(i1,i2) = a(i1,i2)*tm(i1,i2)^3/6 + b(i1,i2)*tm(i1,i2)^2/2 + c(i1,i2)*tm(i1,i2) + d(i1,i2);
                        vm(i1,i2) = a(i1,i2)*tm(i1,i2)^2/2 + b(i1,i2)*tm(i1,i2) + c(i1,i2);
                        um(i1,i2) = a(i1,i2)*tm(i1,i2) + b(i1,i2);
                        if tmf(i1,i2)-tmo(i1,i2)<= 2 && (vm(i1,i2)> vmf(1,i2) || vm(i1,i2)< vmf(1,i2))
                            vm(i1,i2) = vmf(1,i2);
                            um(i1,i2) = 0;
                            xm(i1,i2) = xm(i1-1,i2) + vmf(1,i2)*dt;
                        end
                        % if the vehicle hit the lower bound on speed, don't apply
                        % control. Update states and initial conditions
                        if vm(i1,i2)<vmin_mr
                            % Update states and control
                            xm(i1,i2) = xm(i1-1,i2)+ vm(i1-1,i2)*dt; % delta t = 1
                            vm(i1,i2) = vm(i1-1,i2);
                            um(i1,i2) = 0;
                            flag(i1,i2) = 1;
                            
                            % Update initial conditions
                            xmo(i1,i2) = xm(i1-1,i2);
                            xmf(i1,i2) = xmf(i1-1,i2);
                            vmo(i1,i2) = vm(i1-1,i2);
                            vmf(i1,i2) = vmf(i1-1,i2);
                            tmo(i1,i2) = tm(i1-1,i2);
                            tmf(i1,i2) = tmf(i1,i4(q2-1)) + iz_length/vmf(i1,i4(q2-1))-0.02;
                            %tmf(i1,i2) = tmf(i1,i2-1) +
                            %iz_length/vmf(i1,i2-1)-0.02;------------
                        end
                    end
                end
                % Update variables after control zone
                if xm(i1-1,i2)>400
                    
                    % Update initial conditions:
                    xmf(i1,i2) = xmf(i1-1,i2); % We want xf to be constant always
                    vmf(i1,i2) = vmf(i1-1,i2); % We want vf to be constant always
                    tmo(i1,i2) = tmo(i1-1,i2); % We want to=0 always
                    tmf(i1,i2) = tmf(i1-1,i2); % We want tf to keep constant until we enter the control zone
                    
                    % Update states
                    tm(i1,i2)=i1*dt;
                    xm(i1,i2) = xm(i1-1,i2)+ vmf(i1,i2)*dt;
                    vm(i1,i2) = vmf(i1,i2);
                    um(i1,i2) = 0;
                end
                if xm(i1,i2)<430 && R12_ini_h(1,i2)==2
                    ym(i1,i2)=xm(i1,i2)*0.25;
                else
                    ym(i1,i2)=107.625;
                end
            end
            if R12_ini_h(8,i2)== 4% if MDV  ------------------------------------
                if xm(i1-1,i2)<0
                    % Update initial conditions:
                    xmo(i1,i2) = xm(i1-1,i2); % We want xo to be constant until reaching the control zone
                    xmf(i1,i2) = xmf(i1-1,i2); % We want xf to be constant always
                    vmo(i1,i2) = vm(i1-1,i2);
                    
                    vmf(i1,i2) = vmf(i1-1,i2);
                    tmo(i1,i2) = i1-1; % We want to update to at each instant of time
                    tmf(i1,i2) = tmf(i1-1,i2); % We want tf to keep constant until we enter the control zone
                    
                    tm(i1,i2)=i1*dt;
                    xm(i1,i2) = xm(i1-1,i2)+ vmo(i1,i2)*dt;
                    vm(i1,i2) = vmo(i1,i2);
                    um(i1,i2) =-0.05;
                    flag(i1,i2) = 0; % this flag is to detect that the minimum speed value has been reached
                    % rear end collision constraint ---------------
                    % TWO IF LOOPS CAN BE REMOVED SINCE i2-1 is changed to i4(q2-1) AUG 21
                    %                     if (R12_ini_h(1,i2-1)==2 && R12_ini_h(1,i2)==2) % added the constraint  of same road AUGUST 7th
                    if(i2~=2)
                        if xm(i1,i2)-xm(i1,i4(q2-1))<Ssafe
                            vm(i1,i2)=vm(i1,i4(q2-1))-2;
                        end
                    end
                    %                     end
                    %
                    if xm(i1,i2)>=0
                        xmo(i1,i2) = xm(i1,i2);
                        tmo(i1,i2) = tm(i1,i2);
                        vmo(i1,i2) = vm(i1,i2);
                    end
                    
                end
                % Update variables and control on the control zone
                if xm(i1-1,i2)>=0 && xm(i1-1,i2)<=430
                    
                    % Calculate control for first vehicle:
                    if i2==2 % in secondary road first vehicle is in 2nd position of R12_ini_h matrix AUGUST 7
                        
                        % Update the initial conditions:
                        xmo(i1,i2) = xm(i1-1,i2);
                        xmf(i1,i2) = xmf(i1-1,i2);
                        vmo(i1,i2) = vm(i1-1,i2);
                        
                        % vmf(i1,i2) = vmf(i1-1,i2);% CHECK THIS
                        %vmf(i1,i2) = 12.7631; % from distribution
                        vmf(i1,i2) = vf_model(1,i2);
                        tmo(i1,i2) = tm(i1-1,i2);
                        tmf(i1,i2) = tmf(i1-1,i2);
                        
                        % Update states and control
                        tm(i1,i2)=i1*dt;
                        um(i1,i2) = -0.05;
                        vm(i1,i2) = sqrt((vmf(i1,i2)).^2-(2*um(i1,i2)*(430-xmo(i1,i2))));
                        xm(i1,i2) = xmo(i1,i2)+vm(i1,i2)*dt+(0.5*um(i1,i2)*dt^2);
                        
                    else % for rest of the vehicles
                        % Update the initial conditions
                        xmo(i1,i2) = xm(i1-1,i2);
                        xmf(i1,i2) = xmf(i1-1,i2);
                        vmo(i1,i2) = vm(i1-1,i2);
                        
                        vmf(i1,i2) = vf_model(1,i2); % from distribution
                        tmo(i1,i2) = tm(i1-1,i2);
                        tmf(i1,i2) = tmf(i1,i4(q2-1))+ iz_length/vmf(i1,i4(q2-1));
                        
                        % Calculate constants. If to is too close to tf, control
                        % signal will go to infinity, so, the control should be
                        % stopped if the difference between the two is less than 3 seconds
                        
                        % Update states and control
                        tm(i1,i2)=i1*dt;
                        um(i1,i2) = -0.05;
                        vm(i1,i2) = sqrt((vmf(i1,i2)).^2-(2*um(i1,i2)*(430-xmo(i1,i2))));
                        xm(i1,i2) = xmo(i1,i2)+vm(i1,i2)*dt+(0.5*um(i1,i2)*dt^2);
                        %                         % rear end collision constraint ---------------
                        %
                        if(i2~=2)
                            %                             if (R12_ini_h(1,i2-1)==2 && R12_ini_h(1,i2)==2) % added the constraint  of same road AUGUST 7th
                            %
                            if xm(i1,i2)-xm(i1,i4(q2-1))<Ssafe
                                vm(i1,i2)=vm(i1,i4(q2-1))-2;
                            end
                        end
                        %                         end
                    end
                end
                % Update variables after control zone
                if xm(i1-1,i2)>430
                    % Update initial conditions:
                    xmf(i1,i2) = xmf(i1-1,i2); % We want xf to be constant always
                    vmf(i1,i2) = vmf(i1-1,i2); % CHECK THIS GIVE some value
                    tmo(i1,i2) = tmo(i1-1,i2); % We want to=0 always
                    tmf(i1,i2) = tmf(i1-1,i2); % We want tf to keep constant until we enter the control zone
                    
                    % Update states
                    tm(i1,i2)=i1*dt;
                    xm(i1,i2) = xm(i1-1,i2)+ vmf(i1,i2)*dt;
                    vm(i1,i2) = vmf(i1,i2);
                    um(i1,i2) = -0.05;
                end
                % rear end collision constraint ---------------
                if(i2~=2)% changed i2=1 to i2=1 because in secondary road always the first vehicle is in second
                    %                     % column (i think based on distance also) in R12_ini_h
                    %                     % matrix
                    %                     if (R12_ini_h(1,i2-1)==2 && R12_ini_h(1,i2)==2) % added the constraint  of same road AUGUST 7th
                    if xm(i1,i2)-xm(i1,i4(q2-1))<Ssafe
                        vm(i1,i2)=vm(i1,i4(q2-1))-2;
                    end
                end
                %                 end
            end % for loop of MDV or CAV
            
            if xm(i1,i2)<430 && R12_ini_h(1,i2)==2
                ym(i1,i2)=xm(i1,i2)*0.25;
            else
                ym(i1,i2)=107.625;
                % changed this if condition out of for loop of MDV,CAV
            end
            
            q2=q2+1;
        end % for if loop of seconsdary road
    end % for loop of vehicles
end % end for for loop of time

% we have x for main road, xm for secondary road
%% Show animation
i4=find(R12_ini_h(1,:)==2); % Identifies secondary road
% i5=find(R12_ini_h(1,:)==1); % Identifies main road
% i5 has been declared before AUG 7
for n=1:numel
    for k=1:length(i5)
%         set(gcf,'Renderer','OpenGL'); 
        if (R12_ini_h(8,i5(k)) == 3 && R12_ini_h(1,i5(k))==1)
            % it is CAV and on main road
            disp('case 3');
            line(x(n,i5(k)),y(n,i5(k)), 'Marker', '.', 'MarkerSize', 20, 'Color', 'b');
%             h1=plot(x(n,i5(k)),y(n,i5(k)),'ob','MarkerSize',5, 'MarkerFaceColor','g');
%             set(h1,'EraseMode','normal');
            hold on;
               
        end
        
        if (R12_ini_h(8,i5(k)) == 4 && R12_ini_h(1,i5(k))==1)
            % it is MDV an on main road
            disp('case 4');
            plot(x(n,i5(k)),y(n,i5(k)),'or','MarkerSize',5,  'MarkerFaceColor','r');
            hold off;
        end
%         
%         disp('main road');
%         disp(x(n,i5(k)))    % displays the x value every time on main road
%     end
%     
%     for k1=1:length(i4)
%         
%         if (R12_ini_h(8,i4(k1)) == 3 && R12_ini_h(1,i4(k1))==2)
%             % it is CAV and on secondary road
%             disp('case 1');
%             plot(xm(n,i4(k1)),ym(n,i4(k1)),'ob','MarkerSize',5, 'MarkerFaceColor','y');
%             hold on;
%         end
%         
%         if (R12_ini_h(8,i4(k1)) == 4 && R12_ini_h(1,i4(k1))==2)
%             % it is MDV and on secondary road
%             %x2(n)= xm(n,i4(k1)); % Secondary road
%             disp('case 2');
%             plot(xm(n,i4(k1)),ym(n,i4(k1)),'or','MarkerSize',5, 'MarkerFaceColor','k');
%             hold on; % changed hold off to hold on AUG 16 to match with prevous working code
%         end
%         disp('xm second road ');
%         disp(xm(n,i4(k1)))
    end
    
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

% Display results as animation
figure
axes('Position',[0 0 1500 1500])
movie(M,1)

myVideo= VideoWriter('MergingRoads_Coordinated_DS.avi');
myVideo.FrameRate = 5;  % Default 30
myVideo.Quality = 90;    % Default 75
open(myVideo);
writeVideo(myVideo, M);
close(myVideo);
