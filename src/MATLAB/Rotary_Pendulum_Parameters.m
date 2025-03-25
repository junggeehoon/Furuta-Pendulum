close, clear, clc

g = 9.80665; % m/s^2

m1 = 0.038;  % Mass of rotary arm
m2 = 0.128;  % Mass of pendulum with thum screw

L1 = 0.05;   % Rotary arm length from pivot to tip
l1 = 0.5*L1; % Rotary arm length from pivot to center of mass 

L2 = 0.08;   % Total length of pendulum
l2 = 0.5*L2; % Pendulum length from pivot to center of mass 
% l2 = 0.033; % Distance between center of mass and pivot of pendulum

J0_rotor = 1.79584e-3;
J0_cover = 1.36e-5;
J0_mount = 1.843e-5;

J0 = J0_rotor + J0_cover + J0_mount;

I1 = m1*L1^2/12;   % Rotary arm moment of inertia about its center of mass 
I2 = m2*L2^2/12;   % Pendulum moment of inertia about center of mass 
J1 = I1 + m1*l1^2; % Rotary arm moment of inertia about pivot
J2 = I2 + m2*l2^2; % Pendulum moment of inertia about pivot

% Friction coefficients are approximated
d1 = 4.554e-4;
d2 = 1e-5;

%% Define system matrices
A = [0, 0, 1, 0;
0, 0, 0, 1;
0, (L1*g*l2^2*m2^2)/(J0*J2 + J1*J2 + J2*L1^2*m2 + J0*l2^2*m2 + J1*l2^2*m2), -(d1*(m2*l2^2 + J2))/(J0*J2 + J1*J2 + J2*L1^2*m2 + J0*l2^2*m2 + J1*l2^2*m2), -(L1*d2*l2*m2)/(J0*J2 + J1*J2 + J2*L1^2*m2 + J0*l2^2*m2 + J1*l2^2*m2);
0, (g*l2*m2*(m2*L1^2 + J0 + J1))/(J0*J2 + J1*J2 + J2*L1^2*m2 + J0*l2^2*m2 + J1*l2^2*m2), -(L1*d1*l2*m2)/(J0*J2 + J1*J2 + J2*L1^2*m2 + J0*l2^2*m2 + J1*l2^2*m2), -(d2*(m2*L1^2 + J0 + J1))/(J0*J2 + J1*J2 + J2*L1^2*m2 + J0*l2^2*m2 + J1*l2^2*m2)]
B = [0; 0; (m2*l2^2 + J2)/(J0*J2 + J1*J2 + J2*L1^2*m2 + J0*l2^2*m2 + J1*l2^2*m2); (L1*l2*m2)/(J0*J2 + J1*J2 + J2*L1^2*m2 + J0*l2^2*m2 + J1*l2^2*m2)]

%% Output
C = [0 1 0 0];
D = 0;

% Build System
x0 = [0; 0.1; 0; 0];
threshold = deg2rad(15); % Threshold angle to switch LQR controller
r = [0; pi; 0; 0]; % reference position


%% LQR Controller
Q = diag([1 1 1 1]);
R = 200;
K = lqr(A,B,Q,R)