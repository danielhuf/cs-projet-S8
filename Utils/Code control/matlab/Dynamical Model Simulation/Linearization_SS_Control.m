clear all;
close all;
clc;

%% Running the first script
Dynamical_Model_And_Validation;

% Recall that:

% U = [tau1; tau2; tau3];
% 
% X = [q1; q2; q3; dq1; dq2; dq3];
% 
% Y = [q1; q2; q3];

%% Linearization

% Linearization around Ud, Xd

f1 = dq1;
f2 = dq2;
f3 = dq3;
f4 = Theta2nde(1);
f5 = Theta2nde(2);
f6 = Theta2nde(3);

A = jacobian([f1, f2, f3, f4, f5, f6] , [q1, q2, q3, dq1, dq2, dq3]);
B = jacobian([f1, f2, f3, f4, f5, f6] , [tau1, tau2, tau3]);


% Definition of the equilibrium point
q1_e = 0;
q2_e = 0;
q3_e = 0;
dq1_e = 0.01;
dq2_e = 0.01;
dq3_e = 0.01;

% Computation of the input at equilibrium level
U_e = double(subs(G, [q1, q2, q3], [q1_e, q2_e, q3_e]));
tau1_e = round(U_e(1), 5);
tau2_e = round(U_e(2), 5);
tau3_e = round(U_e(3), 5);

%% State Space Model Computation around the Equilibrium Point

A_e = double(subs(A, [q1, q2, q3, dq1, dq2, dq3, tau1, tau2, tau3], [q1_e, q2_e, q3_e, dq1_e, dq2_e, dq3_e, tau1_e, tau2_e, tau3_e]));

B_e = double(subs(B, [q1, q2, q3, dq1, dq2, dq3, tau1, tau2, tau3], [q1_e, q2_e, q3_e, dq1_e, dq2_e, dq3_e, tau1_e, tau2_e, tau3_e]));

C_e = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0];
D_e = zeros(3,3);

sys = ss(A_e,B_e,C_e,D_e);


%% Designing the Control Law (LQR)
Q = eye(6,6);
R = 0.00001*eye(3,3);

[K, P] = lqr(sys, Q, R);

L = -inv(C_e*inv(A_e-B_e*K)*B_e);


