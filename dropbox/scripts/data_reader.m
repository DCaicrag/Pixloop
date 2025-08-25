close all, clear, clc

data = readmatrix("prueba7_waypoints.csv");

x = data(:,1);
y = data(:,2);

plot(x,y,'x-')
grid on
hold on
plot(x(1), y(1), 'or')