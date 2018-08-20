function[xo,yo,xf,vo,vf,to,tf,t,x,y,v,u] = mainRoad(numel,R12_ini_h,dt,vf_model,xo,yo,xf,vo,vf,to,tf,t,x,y,v,u,i5,q)
for i1=2:numel
    
    % Loop for each vehicle
    for i2=1:length(R12_ini_h(1,:))
        if mod(q,3)==0 % check q modulus 3 so that for alternate cars are taken in i5(q-1) AUG 19 
            q=1;
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

%                  q=q+1; % increase q because one car exists and we go to next car in the next i2 loop
                
                else % for the rest of the vehicles
%                 if q== 1
%                     q=2; % used this if condition because when cav is 2nd car (q-1)value is becomeing 0 AUG 13
%                 end
                    % Update the initial conditions
                    xo(i1,i2) = x(i1-1,i2);
                    xf(i1,i2) = xf(i1-1,i2);
                    vo(i1,i2) = v(i1-1,i2);
                    vf(i1,i2) = vf(i1-1,i2);
                    to(i1,i2) = t(i1-1,i2);
                    tf(i1,i2) = tf(i1,i5(q-1)) + iz_length/vf(i1,i5(q-1));
%                     tf(i1,i2) = tf(i1,i2-1) + iz_length/vf(i1,i2-1);
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
                        disp('values for RTcontrol') 
                        disp(to)
                        disp(tf)
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
                        tf(i1,i2) = tf(i1,i5(q-1)) + iz_length/vf(i1,i5(q-1))-0.02;
%                         tf(i1,i2) = tf(i1,i2-1) + iz_length/vf(i1,i2-1)-0.02;
                    %changed i2-1(any previous vehicle) to previous main
                    %road vehicle (i5(q-1)) AUG 7
                    end
                    q=q+1; % increase q to next vehicle number AUG 7
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
        if R12_ini_h(8,i2)== 4 % if MDV  ------------------------------------
            disp(x(i1-1,i2))
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
                    if (R12_ini_h(1,i2-1)==1 && R12_ini_h(1,i2)==1)
                        % i have removed  || (R12_ini_h(1,i2-1)==2 && R12_ini_h(1,i2)==2) this part of condition so
                        % that it doesnt go into or condition AUGUST 7
                        %if previous and current vehicle are on same road(main
                        %road or secondary)
                        if x(i1,i2)-x(i1,i2-1)<Ssafe 
                            v(i1,i2)=v(i1,i2-1)-2;
                        end
                    end
                end    
                if x(i1,i2)>=0
                    xo(i1,i2) = x(i1,i2);
                    to(i1,i2) = t(i1,i2);
                    vo(i1,i2) = v(i1,i2);
                end

            end
            disp('check x1')
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
                    tf(i1,i2) = tf(i1,i5(q-1))+ iz_length/vf(i1,i5(q-1));

                    % Calculate constants. If to is too close to tf, control
                    % signal will go to infinity, so, the control should be
                    % stopped if the difference between the two is less than 3 seconds

                    % Update states and control
                    t(i1,i2)=i1*dt;
                    u(i1,i2) = -0.05;
                    v(i1,i2) = sqrt((vf(i1,i2)).^2-(2*u(i1,i2)*(430-xo(i1,i2))));
                    x(i1,i2) = xo(i1,i2)+v(i1,i2)*dt+(0.5*u(i1,i2)*dt^2);    
            % rear end collision constraint ---------------
                    if (R12_ini_h(1,i2-1)==1 && R12_ini_h(1,i2)==1) % || (R12_ini_h(1,i2-1)==2 && R12_ini_h(1,i2)==2)
                    if(i2~=1)
                        if x(i1,i2)-x(i1,i2-1)<Ssafe 
                            v(i1,i2)=v(i1,i2-1)-2;
                        end
                    end
                    end
               end

            end
%             Update variables after control zone 
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
                if (R12_ini_h(1,i2-1)==1 && R12_ini_h(1,i2)==1) 
                % added the if condition above here AUGUST 7
                    if x(i1,i2)-x(i1,i2-1)<Ssafe 
                        v(i1,i2)=v(i1,i2-1)-2;
                    end
                end
            end
            if x(i1,i2)<430 && R12_ini_h(1,i2)==2
                y(i1,i2)=x(i1,i2)*0.25;
            else
                y(i1,i2)=107.625;
            end       
            
        end % for loop of MDV or CAV
        q=q+1;  % increment for q when there is a vehicle in main road
     end % for the if loop of main road
   end
end


end     % end of the function
