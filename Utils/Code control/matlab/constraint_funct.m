function [c, ceq] = constraint_funct(x)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
a1 = 0.130;
a2 = 0.124;
a3 = 0.126;

global final_x;
global min_arm_lenght;
global max_arm_lenght;

Vmax = 4.8171*0.124;
velocity_up = 9*Vmax*Vmax;

c = [-(vx(x, final_x)^2 + x(3)^2)               ;  -(x(1)^2 + x(2)^2) + min_arm_lenght^2;
      (vx(x, final_x)^2 + x(3)^2) - velocity_up ;  (x(1)^2 + x(2)^2) - max_arm_lenght^2];
     % x(1)*vx(x, final_x) + x(2)*x(3) - x(4)];
    
ceq = x(1)*vx(x, final_x) + x(2)*x(3) ;
 
end

