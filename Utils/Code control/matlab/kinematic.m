function [x,y] = kinematic(theta)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
global a1;
global a2;
global a3;

x = a1*cos(theta(1,:)) + a2*cos(theta(1,:) + theta(2,:)) + a3*cos(theta(1,:)+theta(2,:)+theta(3,:));
y = a1*sin(theta(1,:)) + a2*sin(theta(1,:) + theta(2,:)) + a3*sin(theta(1,:)+theta(2,:)+theta(3,:));

end