
m1 = 1.3850917e-01; 
m2 = 1.3274562e-01; 
m3 = 1.4327573e-01;

%% Link 1
% Principal Moments of Inertia
ixx1 = 3.9362142*10^4 * 10^-9; %g.mm2 . We need to convert to Kg.m2

iyy1 = 2.4070845*10^5 * 10^-9;

izz1 = 2.4489452*10^5 * 10^-9;





% Moment of Inertia of Link 1 about an axis through its center of mass
Ic1 = ixx1 + iyy1 + izz1;
d1 = sqrt(0.5*(ixx1+iyy1));

% Moment of Inertia of Link 1
I1 = Ic1 + m1*d1^2;

%% Link 2
% Principal Moments of Inertia
ixx2 = 2.4799466*10^4 * 10^-9; %g.mm2 . We need to convert to Kg.m2

iyy2 = 1.7819113*10^5 * 10^-9;

izz2 = 1.8688812*10^5 * 10^-9;





% Moment of Inertia of Link 2about an axis through its center of mass
Ic2 = ixx2 + iyy2 + izz2;
d2 = sqrt(0.5*(ixx2+iyy2));

% Moment of Inertia of Link 2
I2 = Ic2 + m2*d1^2;

%% Link 3
% Principal Moments of Inertia
ixx3 = 1.6873182*10^4 * 10^-9; %g.mm2 . We need to convert to Kg.m2

iyy3 = 2.0769694*10^5 * 10^-9;

izz3 = 2.7228452*10^5 * 10^-9;



% Moment of Inertia of Link 1 about an axis through its center of mass
Ic3 = ixx3 + iyy3 + izz3;
d3 = sqrt(0.5*(ixx3+iyy3));

% Moment of Inertia of Link 1
I3 = Ic3 + m3*d3^2;

