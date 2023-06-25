clear all;
close all;
clc;

g = 9.81;

%% Robot Parameters

% Joints Masses
m1 = 1.3850917e-01; % masse du membre inférieur
m2 = 1.3274562e-01; % masse du membre secondaire
m3 = 1.4327573e-01; % masse de la pince

% Joints Length
l1 = 0.13; % longueur du membre inférieur
l2 = 0.124; % longueur du membre secondaire
l3 = 0.146; % longueur de la pince

lc1 = l1/2;
lc2 = l2/2;
lc3 = l3/2;

% Joints Inertia

Inertia_Computation;


% Symbolic variables and parameters
syms q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3 tau1 tau2 tau3 real



%% Input, State and Ouput Vectors

U = [tau1; tau2; tau3];

X = [q1; q2; q3; dq1; dq2; dq3];

Y = [q1; q2; q3];



%% Mass Matrix

alpha1 = m1*lc1^2 + m2*(l1^2+lc2^2+2*l1*lc2*cos(q2)) + m3*(l1^2+l2^2+lc3^2+2*l1*l2*cos(q2)+2*l2*lc3*cos(q3)+2*l1*lc3*cos(q2+q3));
B = m2*(lc2^2 + l1*lc2*cos(q2)) + m3*(l2^2+lc3^2+l1*l2*cos(q2)+2*l2*lc3*cos(q3)+l1*lc3*cos(q2+q3));
gamma = m3*(lc3^2+l2*lc3*cos(q3)+l1*lc3*cos(q2+q3));
alpha2 = m2*lc2^2+m3*(l2^2+lc3^2+2*l2*lc3*cos(q3));
eta = m3*(lc3^2+l2*lc3*cos(q3));
alpha3 = m3*lc3^2;

M = [ (alpha1 + I1 + I2 + I3) (B + I2 + I3) (gamma + I3) ;
    (B + I2 + I3) (alpha2 + I2 + I3) (eta + I3) ;
    (gamma + I3) (eta + I3) alpha3 + I3 ];

% Verifying that M is symmetric positive definite

try chol(M);
    disp('Matrix is symmetric positive definite.')
catch ME
    disp('Matrix is not symmetric positive definite')
end

%% Coriolis Matrix

% Calcul de M'(teta). tetap

alpha1p = -2*m2*l1*lc2*dq2*sin(q2) -2*m3*l1*l2*dq2*sin(q2) -2*m3*l2*lc3*dq3*sin(q3) -2*m3*l1*lc3*(dq2+dq3)*sin(q2+q3);
Bp = -m2*l1*lc2*dq2*sin(q2) -m3*l1*l2*dq2*sin(q2) -2*m3*l2*lc3*dq3*sin(q3) -m3*l1*lc3*(dq2+dq3)*sin(q2+q3);
gammap = -m3*l2*lc3*dq3*sin(q3) -m3*l1*lc3*(dq2+dq3)*sin(q2+q3);
alpha2p = -2*m3*l2*lc3*dq3*sin(q3) ;
etap = -m3*l2*lc3*dq3*sin(q3) ;
alpha3p = 0;

Mptp = [ dq1*alpha1p + dq2*Bp + dq3*gammap ;
        dq1*Bp + dq2*alpha2p + dq3*etap ;
        dq1*gammap + dq2*etap];



% Calcul de D(teta, tetap)

D = [0;
     -2*m2*l1*lc2*(dq1^2)*sin(q2)-2*m3*l1*l2*(dq1^2)*sin(q2)-2*m3*l1*lc3*(dq1^2)*sin(q2+q3)-2*m2*l1*lc2*dq1*dq2*sin(q2)-2*m3*l1*l2*dq1*dq2*sin(q2)-2*m3*l1*lc3*dq1*dq2*sin(q2+q3)-2*m3*l1*lc3*dq1*dq3*sin(q2+q3);
        -2*m3*l2*lc3*(dq1^2)*sin(q3)-2*m3*l1*lc3*(dq1^2)*sin(q2+q3)-4*m3*l2*lc3*dq1*dq2*sin(q3)-2*m3*l1*lc3*dq1*dq2*sin(q2+q3)-2*m3*l2*lc3*dq1*dq3*sin(q3)-2*m3*l1*lc3*dq1*dq3*sin(q2+q3)-2*m3*l2*lc3*(dq2^2)*sin(q3)-2*m3*l2*lc3*dq2*dq3*sin(q3)];

% Calcul de tetap . C(teta, tetap)

C  = Mptp - 0.5 * D;

%% Gravity matrix
G = [(m1*g*lc1+m2*g*l1+m3*g*l1)*cos(q1)+(m2*g*lc2+m3*g*l2)*cos(q1+q2)+m3*g*lc3*cos(q1+q2+q3) ;
     (m2*g*lc2+m3*g*l2)*cos(q1+q2)+m3*g*lc3*cos(q1+q2+q3);
     m3*g*lc3*cos(q1+q2+q3)];

disp('G is maximum when the arm is fully extended ie teta1 = teta2 = teta3 = 0');
Gmax = vpa(subs(G, [q1, q2, q3], [0, 0, 0]), 3);
disp(Gmax);

%% Friction
% The friction term is proportional to the absolute value of the joint velocity and the sign of the velocity. 
% This is because the friction force acts in the opposite direction to the joint velocity, 
% and its magnitude is proportional to the normal force between the joint surfaces, which is proportional to the joint force and thus the joint velocity.


Ff = [ - sign(dq1)*m3*(l1^2*sin(q1) + l2^2*sin(q1+q2) + l3^2*sin(q1+q2+q3))*abs(dq1);
       - sign(dq2)*m3*(l2^2*sin(q1+q2) + l3^2*sin(q1+q2+q3))*abs(dq2);
       - sign(dq3)*m3*l3^2*sin(q1+q2+q3)*abs(dq3)];


%% Nonlinear dynamics equation
Theta2nde = inv(M)*(U - C - G - Ff);