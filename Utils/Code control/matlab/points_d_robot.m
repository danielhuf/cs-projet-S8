function [base, p1, p2, p3] = points_d_robot(theta)
%points to draw the robot

global a1;
global a2;
global a3;

base = [0 ; 0];
p1 = [a1 * cos(theta(1)); a1*sin(theta(1)) ];
p2 = p1 + [a2*cos(theta(1) + theta(2)) ; a2*sin(theta(1) + theta(2))];
p3 = p2 + [a3*cos(theta(1) + theta(2)+ theta(3)); a3*sin(theta(1) + theta(2)+ theta(3))];


end