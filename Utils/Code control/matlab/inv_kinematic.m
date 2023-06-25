function theta= inv_kinematic(pos, fi)

% Masses des membres
m1 = 1.3850917e-01; % masse du membre inférieur
m2 = 1.3274562e-01; % masse du membre secondaire
m3 = 1.4327573e-01; % masse de la pince

% Longueurs des membres
l1 = 0.13; % longueur du membre inférieur
l2 = 0.124; % longueur du membre secondaire
l3 = 0.146; % longueur de la pince

x = pos(1);
y = pos(2);


xB = x - l3*cos(fi);
yB = y - l3*sin(fi);

theta2 = acos((xB^2+yB^2-l1^2-l2^2)/(2*l1*l2));

theta1 = asin(((l1+l2*cos(theta2 ))*yB - l2*sin(theta2 )*xB)/(xB^2 +yB^2));

theta3 = fi - theta1 - theta2;


theta = [theta1 ; theta2 ; theta3];