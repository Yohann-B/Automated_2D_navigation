function dS = robot(t,state)

global K_1 K_2 K_G x_1 y_1 x_2 y_2 x_g y_g

%% States
x = state(1);
y = state(2);
Theta = state(3);

%% Parameters
V = 4;
K_p = 3;
L = 2;

%% Calculating Forces for a Mobile Robot
F_1x = ((-K_1)*((x_1)-x)) / ((x-x_1)^2 + (y-y_1)^2)^(1.5);
F_2x = ((-K_2)*((x_2)-x)) / ((x-x_2)^2 + (y-y_2)^2)^(1.5);
F_Gx = ((K_G)*((x_g)-x)) / sqrt((x-x_g)^2 + (y-y_g)^2);
F_x = F_1x + F_2x + F_Gx;

F_1y = ((-K_1)*((y_1)-y)) / ((x-x_1)^2 + (y-y_1)^2)^(1.5);
F_2y = ((-K_2)*((y_2)-y)) / ((x-x_2)^2 + (y-y_2)^2)^(1.5);
F_Gy = ((K_G)*((y_g)-y)) / sqrt((x-x_g)^2 + (y-y_g)^2);
F_y = F_1y + F_2y + F_Gy;

Alpha = atan2(F_y,F_x);
Phi = K_p*(Alpha-Theta);

%% Mobile Robot Dynamics
dS(1) = V*cos(Phi)*cos(Theta);
dS(2) = V*cos(Phi)*sin(Theta);
dS(3) = (V/L)*sin(Phi);
dS = [dS(1);dS(2);dS(3)];
end