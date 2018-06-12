% code is working for more than one vehicle on each lane  without end to
% end collision.
% try to add collision constraint for MDVs
% date: JUNE 5 2018
clear; 
clear all
close all; 
clc
dt=1;
vmax_2 = 13.4112; % [m/s] ==> 30 MPH
vmax_mr = 13.5; % [m/s] ==> 30 MPH
vmax_sr = 13.4112;   
vavg_mr = 13.5; % [m/s] ==> 30 MPH
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
R1_xo = [ -150 -250 -100 ];
%R2_xo = [ -150 -250 -100 ]; 
R2_xo = -300 + (300).*rand(1,3)% generates random values between -300 and 0
% or just type out some random values
%============== PRE-DEFINED VALUES =======================================
%               MANUAL VEHICLE - Secondary Road                 
%========================================================================
a_model=-0.05; %acceleration for manualy driven vehicle(MDV) on secondary road
% for loop to calculate MDV model values for more than one vehicle
dt=1;
s(1,:)=R2_xo; %distance travelled  
for i=1:length(R2_xo)
    vf_model(i) = 12 + (14-12).*rand(1) % pre-defining v-final for MDV
    %vf_model(i) = 12.6808
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
R1_ini(1,:)= ones(1,length(R1_xo));% xo: Initial position on x
R1_ini(2,:)= R1_xo; % yo: Initial position on y
R1_ini(3,:)= 107.625; % xf: Initial position
R1_ini(4,:)= ones(1,length(R1_xo))*cz_length;%vo: Initial speed
R1_ini(5,:)= ones(1,length(R1_xo))*vavg_mr;% vf: final speed
R1_ini(6,:)= ones(1,length(R1_xo))*vmax_mr;% t2iz: Time to intersection zone at constant speed = vavg
R1_ini(7,:)= (cz_length-R1_xo)./R1_ini(5,:); 
R1_ini(8,:)= ones(1,length(R1_xo))*a_CAV;   % store acceleration

% Vehicles on secondary road Identify secundary road as 2
R2_ini(1,:)= ones(1,length(R2_xo)).*2; % xo: Initial position on x
R2_ini(2,:)= R2_xo; % yo: Initial position on y
R2_ini(3,:)= R2_xo*0.25; % xf: final position
R2_ini(4,:)= ones(1,length(R2_xo))*(cz_length); % vo: Initial speed
R2_ini(5,:)= v1_model; % vf: final speed
R2_ini(6,:)= vf_model; % tf: time to reach intersection
%R2_ini(7,:)= (cz_length-R2_xo)./R2_ini(5,:); 
R2_ini(7,:)= tm_sr; %tf = time to reach intersection is known from model
R2_ini(8,:)= ones(1,length(R2_xo))*a_model; % assumed deceleration

R1_ini = sortrows(R1_ini',7)'

for i3=1:length(R1_ini(7,:))
    if i3==1
        % Since tf=time to reach intersection zone, the time to leave
        % intersection for the first vehicle is the time it takes to cross 
        % the control zone at constant speed=vavg plus the time it takes 
        % to cross the intersection zone at constant speed = vmax
        R1_ini(8,i3) = R1_ini(4,i3)./R1_ini(5,i3); %+(iz_length)./R12_ini_h(6,i3);
    else
        % For the rest of the vehicles it is the time to leave the
        % intersection of the previous vehicle, plus the time it takes to
        % cross the intersection zone at constant speed vmax
        R1_ini(8,i3) = R1_ini(8,i3-1)+(iz_length)./R1_ini(6,i3);
    end
end


%% manual vehicle code

xmo=R2_ini(2,:); % This is still the initial position to start before the control zone
ymo=R2_ini(3,:);
xmf=R2_ini(4,:);
vmo=R2_ini(5,:);
vmf=R2_ini(6,:);
tmo=0.*R2_ini(2,:);
tmf=R2_ini(8,:);
tm2exit=R2_ini(8,:);
tm=zeros(1,length(tmf));

% State and control values for t=0
xm=xmo;
y=ymo;
vm=vmo;
um=-0.05;

% Loop for the simulation time
for i1=2:numel
    % Loop for each vehicle
    for i2=1:length(R2_ini(1,:))        
        % Update variables before control zone 
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
        
            if xm(i1,i2)>=0
                xmo(i1,i2) = xm(i1,i2);
                tmo(i1,i2) = tm(i1,i2);
                vmo(i1,i2) = vm(i1,i2);
            end
            
        end
        
        % Update variables and control on the control zone
        if xm(i1-1,i2)>=0 && xm(i1-1,i2)<=430
                        
            % Calculate control for first vehicle:
            if i2==1
                
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
                tmf(i1,i2) = tmf(i1,i2-1)+ iz_length/vmf(i1,i2-1);
                
                % Calculate constants. If to is too close to tf, control
                % signal will go to infinity, so, the control should be
                % stopped if the difference between the two is less than 3 seconds

                % Update states and control
                tm(i1,i2)=i1*dt;
                um(i1,i2) = -0.05;
                vm(i1,i2) = sqrt((vmf(i1,i2)).^2-(2*um(i1,i2)*(430-xmo(i1,i2))));
                xm(i1,i2) = xmo(i1,i2)+vm(i1,i2)*dt+(0.5*um(i1,i2)*dt^2);    
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
        if xm(i1,i2)<430 && R2_ini(1,i2)==2
            ym(i1,i2)=xm(i1,i2)*0.25;
        else
            ym(i1,i2)=107.625;
        end
    end
end
% %% Define initial conditions of CAV contrl
% xo=R1_ini(2,:); % This is still the initial position to start before the control zone
% yo=R1_ini(3,:);
% xf=R1_ini(4,:);
% vo=R1_ini(5,:);
% vf=R1_ini(6,:);
% to=0.*R1_ini(2,:);
% tf=R1_ini(8,:);
% t2exit=R1_ini(8,:);
% t=zeros(1,length(tf));
% % State and control values for t=0
% x=xo;
% y=yo;
% v=vo;
% u=0*v;
% for i=2:numel
%     for j=1:length(R1_ini(1,:))
%         %v1(i+1,j) = 13.5; % velocity for each time step
%         x(i,j) = x(i-1,j)+ 13.5*dt;  % x positions
%         y(i,j)=107.625; % y psoitions
%         t1(i)=i*1; % to record how many times for loop is run to plot
%         %p=v1(i,j)*(i-tm_sr(j));
%     end
% end
%% CAV simulation 
% Define initial conditions

xo=R1_ini(2,:); % This is still the initial position to start before the control zone
yo=R1_ini(3,:);
xf=R1_ini(4,:);
vo=R1_ini(5,:);
vf=R1_ini(6,:);
to=0.*R1_ini(2,:);
tf=R1_ini(8,:);
t2exit=R1_ini(8,:);
t=zeros(1,length(tf));

% State and control values for t=0
x=xo;
y=yo;
v=vo;
u=0*v;

% Loop for the simulation time
for i1=2:numel
    
    % Loop for each vehicle
    for i2=1:length(R1_ini(1,:))
        
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
        y(i1,i2)=107.625;
        
    end
end

%% Show animation
i4=find(R2_ini(1,:)==2); % Identifies secondary road
i5=find(R1_ini(1,:)==1); % Identifies main road
x1= xm(:,i4); % Secondary road
y1= ym(:,i4);
u1= um(:,i4);
v1= vm(:,i4);

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
    %F(n) = getframe;
    grid on
    title('Vehicles Trajectory');
    drawnow
    pause(0.25)
    M(n)=getframe;
end


axes('Position',[0 0 1500 1500])
movie(M,1)

myVideo= VideoWriter('MergingRoads_Coordinated_DS.avi');
myVideo.FrameRate = 5;  % Default 30
myVideo.Quality = 90;    % Default 75
open(myVideo);
writeVideo(myVideo, M);
close(myVideo);               




% plots
figure(3) % discontinuous plot . for  every five count of time
count=0;
for n=1:t_sim
    count=count+1;
    hold on;
     if(count==3)
         pause(0.1)
         %plot(x1(n),y1(n),'*') %
         plot(n,v2(n,1),'.')
         count=0;
     end
end
figure(4) % continuous plot with pause 
for n=1:t_sim
         pause(0.2)
         %plot(x1(n),y1(n),'*') %
         plot(n,v2(n,1),'.')
         hold on;
end
% hold on;
% plot(time,v)
xlabel('Time (seconds)')
ylabel('velocity (m/s)')
title('time Vs velocity with RSU')
legend('Secondary Road','Main Road')