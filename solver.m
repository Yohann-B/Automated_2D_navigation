clear; clc; close all;

%% Setup
% As we will use the following constants in the "robot" function, we 
% describe them as global:
global K_1 K_2 K_G x_1 y_1 x_2 y_2 x_g y_g

% Gains:
K_G = 1;
K_1 = 3;
K_2 = 5;

% Position of Obstacle 1:
x_1 = 5;
y_1 = 6;

% Position of Obstacle 2:
x_2 = 7;
y_2 = 6;

% Position of Goal:
x_g = 12;
y_g = 12;

%% Calculate Potential Fields
% We calculate the potential field on all the area as we did in part 1.
% This is used only for the plot of the robot's trajectory on the potential
% field. 

% We cut the space in a grid with 0.1 by 0.1 cells:
x = 0:0.1:14;
y = 0:0.1:14;

n = length(x);

for i = 1:n
for j = 1:n
V_1(j,i) = (K_1) / sqrt((x(i)-x_1)^2 + (y(j)-y_1)^2);
V_2(j,i) = (K_2) / sqrt((x(i)-x_2)^2 + (y(j)-y_2)^2);
V_G(j,i) = (K_G) * sqrt((x(i)-x_g)^2 + (y(j)-y_g)^2);

% We got the total value on the point i,j by summing all the values we just
% computed:
V = V_1 + V_2 + V_G;
end
end

%% Mobile Robot Dynamics
% This is where all the job is done. Check the "robot()" function to know
% more about it.

% Initial conditions:
Initials = [0; 0; pi/3];
% Setting up when to stop the simulation previously:
options = odeset('events', @StopSimulation);
% Solving the robot's trajectory problem:
[t,States] = ode23(@robot,[0 100],Initials,options);
% Storing the values in explicit and simpler variables (to plot):
x_robot = States(:,1);
y_robot = States(:,2);

% Calculating the potential values where the robot is passing by, so that
% we can plot the trajectory on the potential graph:
V_11 = (K_1) ./ sqrt((x_robot-x_1).^2 + (y_robot-y_1).^2);
V_22 = (K_2) ./ sqrt((x_robot-x_2).^2 + (y_robot-y_2).^2);
V_GG = (K_G) .* sqrt((x_robot-x_g).^2 + (y_robot-y_g).^2);
VV = V_11 + V_22 + V_GG;

%% Plot Trajectories
figure;
plot(x_robot,y_robot,'b.')
hold on
plot(x_g,y_g,'r-s','LineWidth',2)
hold on
plot(x_1,y_1,'k-s','LineWidth',2)
hold on
plot(x_2,y_2,'k-s','LineWidth',2)
grid on
xlim([0 14])
ylim([0 14])
xlabel('\bfx')
ylabel('\bfy')
title('\bfTrajectory of the robot over the area')
legend('Robot','Target','Obstacles','Location','NorthWest');

figure;
contour(x,y,V)
hold on
plot(x_robot,y_robot,'k','LineWidth',2)
hold on
plot(x_g,y_g,'r-s','LineWidth',1)
xlabel('\bfx')
ylabel('\bfy')
title('\bfRobot trajectory on topological plot')

figure;
mesh(x,y,V)
hold on
plot3(x_robot,y_robot,VV,'k','Linewidth',4)
xlabel('\bfx')
ylabel('\bfy')
zlabel('\bfV')
title('\bfRobot trajectory on 3D plot') 

