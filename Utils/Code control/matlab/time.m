function [time_total] = time(x)
%mesures the time of the throw based on the initial height and vertical
%velocitytim
g = 9.81;
time_total = (x(3) + sqrt(x(3)*x(3) + 2*g*x(2)))/g;

end


