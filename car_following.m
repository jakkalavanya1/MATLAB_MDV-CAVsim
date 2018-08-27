% Simple car following model. Created on August 22, 2018
% written by Lavanya Jakka

clear all;
clc
clf

xpos = [-75 -100 -125 -150];
ypos = 100;
xfinal = 600;
vavg = 14;
uavg = 0;


for i=1:length(xpos)
    tf(i)= (xfinal-xpos(i))/vavg;
end

R_ini(1,:) = xpos;
R_ini(2,:) = ones(1,length(xpos)).*ypos;
R_ini(3,:) = ones(1,length(xpos)).*xfinal;
R_ini(4,:) = ones(1,length(xpos)).*vavg;
R_ini(5,:) = uavg*length(xpos);
R_ini(6,:) = tf;

% Concatenate the two matrices
R_ini_s = sortrows(R_ini',6)';

dt = 1;
t_sim = 80;
numel = t_sim/dt;
Ssafe = 10;


xo=R_ini_s(1,:);
yo=R_ini_s(2,:);
xf=R_ini_s(3,:);
vo=R_ini_s(4,:);

to=0.*R_ini_s(1,:);
tf=R_ini_s(6,:);
t=zeros(1,length(tf));

% State and control values for t=0
x=xo;
y=yo;
v=vo;
u=0*v;

for i1=2:numel
    for i2 = 1:length(R_ini_s(1,:))
        if i2==1
            
            xo(i1,i2) = x(i1-1,i2); 
            xf(i1,i2) = xf(i1-1,i2); 
            vo(i1,i2) = v(i1-1,i2);
            to(i1,i2) = i1-1; 
            tf(i1,i2) = tf(i1-1,i2); 
            
            t(i1,i2)=i1*dt;
            x(i1,i2) = x(i1-1,i2)+ vo(i1,i2)*dt;
%           v(i1,i2) = vo(i1,i2)+2*to(i1,i2);
            v(i1,i2) = vo(i1,i2);
            u(i1,i2) =uavg;
            
        else
            
            % Update initial conditions:
            xo(i1,i2) = x(i1-1,i2); % We want xo to be constant until reaching the control zone
            xf(i1,i2) = xf(i1-1,i2); % We want xf to be constant always
            vo(i1,i2) = v(i1-1,i2);
            to(i1,i2) = i1-1; % We want to update to at each instant of time
            tf(i1,i2) = tf(i1-1,i2);% CHANGE
            
            
            t(i1,i2)=i1*dt;
            x(i1,i2) = x(i1-1,i2)+ vo(i1,i2)*dt;
%           v(i1,i2) = vo(i1,i2);
            u(i1,i2) =uavg;
            
            % Rear end collision constraint
            
            if x(i1,i2)-x(i1,i2-1)>Ssafe
                v(i1,i2)=v(i1,i2-1)+2
                disp('case2')
            end
            if x(i1,i2)-x(i1,i2-1)<Ssafe
                v(i1,i2)=v(i1,i2-1);
%                 disp('case1')
            end
        end
        y(i1,i2)=107.625;
    end
end
    
    
    
    x1= x(:,1); % First vehicle
    y1= y(:,1);
    u1= u(:,1);
    v1= v(:,1);
    
    i5 = 2:length(xpos)
    x2=x(:,i5); % Following vehicles
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
        axis([0,600,0,150]);
        xlabel('x (m)');
        ylabel('y (m)');
        line([0 700],[111.5 111.5],'color','k','LineWidth',2)
        line([0 700],[103.75 103.75],'color','k','LineWidth',2)
        grid on
        title('Vehicles Trajectory');
        drawnow
        pause(0.25)
        M(n)=getframe;
    end
    
    figure
    axes('Position',[0 0 150 150])
    movie(M,1)
    
    myVideo= VideoWriter('MergingRoads_Coordinated_DS.avi');
    myVideo.FrameRate = 5;  % Default 30
    myVideo.Quality = 90;    % Default 75
    open(myVideo);
    writeVideo(myVideo, M);
    close(myVideo);
    
    
    
