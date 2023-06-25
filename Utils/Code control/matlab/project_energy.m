function [energy] = project_energy(x)
% function to calculate the projectile energy
g = 9.81;
global final_x;
energy = g*x(2) + 0.5*(vx(x,final_x)^2 + x(3)^2) ;
end