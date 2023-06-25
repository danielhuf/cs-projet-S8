function [v] = vx(x, final_x)
%given final position and initial position we obtain the x velocity
v = (final_x -x(1))/time(x);

end
