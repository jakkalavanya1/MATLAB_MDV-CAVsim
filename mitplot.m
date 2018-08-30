m = 1;         % mass
L = 1;         % link length
theta1 = 3*pi/4; % initial angle for theta 1 (in rad)
theta2 = 3*pi/8; % initial angle for theta 2 (in rad)
t = linspace(0, 10, 300);  % simulate for 10 seconds with 300 points

% Solving ODE of a double pendulum
[T,Y] = ode45(@(t, x) double_pendulum(t, x, m, L), ...
   t, [theta1, theta2, 0, 0]);

% Calculating joint coordinates for animation purposes
x = [ L*sin(Y(:,1)),  L*sin(Y(:,1))+L*sin(Y(:,2))];
y = [-L*cos(Y(:,1)), -L*cos(Y(:,1))-L*cos(Y(:,2))];

% Convert radians to degrees
ang = Y(:,1:2)*180/pi;
figure;
subplot(2,1,1);
plot(T, ang, 'LineWidth', 2);
hh1(1) = line(T(1), ang(1,1), 'Marker', '.', 'MarkerSize', 20, 'Color', 'b');
hh1(2) = line(T(1), ang(1,2), 'Marker', '.', 'MarkerSize', 20, 'Color', [0 .5 0]);
xlabel('time (sec)'); ylabel('angle (deg)');

subplot(2,1,2);
hh2 = plot([0, x(1,1);x(1,1), x(1,2)], [0, y(1,1);y(1,1), y(1,2)], ...
      '.-', 'MarkerSize', 20, 'LineWidth', 2);
axis equal
axis([-2*L 2*L -2*L 2*L]);
ht = title(sprintf('Time: %0.2f sec', T(1)));

tic;     % start timing
for id = 1:length(T)
   % Update XData and YData
   set(hh1(1), 'XData', T(id)          , 'YData', ang(id, 1));
   set(hh1(2), 'XData', T(id)          , 'YData', ang(id, 2));
   set(hh2(1), 'XData', [0, x(id, 1)]  , 'YData', [0, y(id, 1)]);
   set(hh2(2), 'XData', x(id, :)       , 'YData', y(id, :));
   set(ht, 'String', sprintf('Time: %0.2f sec', T(id)));

   drawnow;
end
fprintf('Animation (Smart update): %0.2f sec\n', toc);