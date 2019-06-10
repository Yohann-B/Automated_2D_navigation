clear all; clc;

%% Gains
K_G = 1;
K_1 = 3;
K_2 = 5;

%% Position of Obstacle 1
x_1 = 5;
y_1 = 6;

%% Position of Obstacle 2
x_2 = 7;
y_2 = 6;

%% Position of Goal
x_g = 12;
y_g = 12;

%% Calculate Potential Fields
x = 0:0.1:14;y = 0:0.1:14;
n = length(x);
for i = 1:n
    for j = 1:n
        V_1(j,i) = (K_1) / sqrt((x(i)-x_1)^2 + (y(j)-y_1)^2);
        V_2(j,i) = (K_2) / sqrt((x(i)-x_2)^2 + (y(j)-y_2)^2);
        V_G(j,i) = (K_G) * sqrt((x(i)-x_g)^2 + (y(j)-y_g)^2);
%         V_1(j,i) = (K_1) / sqrt((-x(i)+x_1)^2 + (-y(j)+y_1)^2);
%         V_2(j,i) = (K_2) / sqrt((-x(i)+x_2)^2 + (-y(j)+y_2)^2);
%         V_G(j,i) = (K_G) * sqrt((-x(i)+x_g)^2 + (-y(j)+y_g)^2);
        V(j,i) = V_1(j,i) + V_2(j,i) + V_G(j,i);
    end
end

%% Plot Figuresfigure;
mesh(x,y,V)
xlabel('\bfx')
ylabel('\bfy')
zlabel('\bfV')
title('\bfPotential field of the area')

figure;
contour(x,y,V)
xlabel('\bfx')
ylabel('\bfy')
title('\bfTopography of the potential field')