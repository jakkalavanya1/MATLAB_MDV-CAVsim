% Simple car following model. Created on August 22, 2018
% written by Lavanya Jakka

pos = [-50 -100 -125 -150];
vel = [ 14 12 13 12.5];
uavg = 0;

for i=1:length(pos)
    tf(i)= pos(i)/vel(i);
end

R_ini(1,:) = pos;
R_ini(2,:) = vel;
R_ini(3,:) = uavg*length(pos);
R_ini(4,:) = tf;

R_ini_s = sortrwos(R_ini,'4');

dt = 1;
t_sim = 80;
numel = t_sim/dt;


for i = 1:length(R12_ini_h(2,:))
    if R12_ini_h(8,i)==4 % manual vehicle
        s(i)=R12_ini_h(2,i)
    end
end



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

for i1=1:numel
    for i2 = 1:length(pos)
        
        
        if i2 = 1
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
                    u(i1,i2) =uavg;
                    
        else
            
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
                    u(i1,i2) =uavg;
                    
                
                    if (R12_ini_h(1,i2-1)==1 && R12_ini_h(1,i2)==1) 
                    % added the if condition above here AUGUST 7
                        if x(i1,i2)-x(i1,i2-1)<Ssafe 
                            v(i1,i2)=v(i1,i2-1)-2;
                        end
                    end
                
        end
        y(i1,i2)=107.625;
    end
end


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


                    
