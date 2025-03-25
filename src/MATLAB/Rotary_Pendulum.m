close, clear, clc

% Define symbolic variables
syms theta1 theta2 theta1_dot theta2_dot theta1_ddot theta2_ddot m1 m2 ...
    u u2 d1 d2 L1 L2 l1 l2 I1 I2 J0 J1 J2 g real

M = [J0+J1+m2*L1^2+J2*sin(theta2)^2 m2*L1*l2*cos(theta2);
         m2*L1*l2*cos(theta2) J2];
V = [-m2*L1*l2*theta2_dot^2*sin(theta2) + J2*theta1_dot*theta2_dot*sin(2*theta2);
     -J2*theta1_dot^2*sin(theta2)*cos(theta2)];
G = [0; m2*g*l2*sin(theta2)];
D = [d1 0; d2 0];
B = [1; 0];
Tau = [u 0];

theta_ddot = M \ (-V - D * [theta1_dot; theta2_dot] - G + B * Tau);

% Position vector of the pendulum center of mass in 3D space
x_p = L1*cos(theta1) - l2*sin(theta2)*sin(theta1);
y_p = L1*sin(theta1) + l2*sin(theta2)*cos(theta1);
z_p = -l2*cos(theta2);

p = [x_p; y_p; z_p];

% Velocity vector of the pendulum center of mass in 3D space
v = diff(p, theta1)*theta1_dot + diff(p, theta2)*theta2_dot;

% Calculate v^2
v_square = v' * v;

% Kinetic energy of the motor
T0 = 0.5 * J0 * theta1_dot^2;

% Kinetic energy of the rotary arm
T1 = 0.5 * J1 * theta1_dot^2;

% Kinetic energy of the pendulum
T2 = 0.5 * J2 * theta2_dot^2 + 0.5 * m2 * v_square;

% Total kinetic energy
T = T0 + T1 + T2;

% Total potential energy, zero potential is defined at hanging-down position
U = m2*g*z_p;

% Lagrange’s equation
L = T - U;

partial_theta1_dot = diff(L, theta1_dot);
partial_theta2_dot = diff(L, theta2_dot);
partial_theta1 = diff(L, theta1);
partial_theta2 = diff(L, theta2);

% Differentiate the expression w.r.t. time
partial_theta1_dot_t = diff(partial_theta1_dot, theta1)*theta1_dot ...
         + diff(partial_theta1_dot, theta2)*theta2_dot ...
         + diff(partial_theta1_dot, theta1_dot)*theta1_ddot ...
         + diff(partial_theta1_dot, theta2_dot)*theta2_ddot;

partial_theta2_dot_t = diff(partial_theta2_dot, theta1)*theta1_dot ...
         + diff(partial_theta2_dot, theta2)*theta2_dot ...
         + diff(partial_theta2_dot, theta1_dot)*theta1_ddot ...
         + diff(partial_theta2_dot, theta2_dot)*theta2_ddot;

% Generalized forces including friction
mu1 = -d1*theta1_dot + u;
mu2 = -d2*theta2_dot;

% Lagrange equations of motion with generalized forces
eqn1 = partial_theta1_dot_t - partial_theta1 == mu1;
eqn2 = partial_theta2_dot_t - partial_theta2 == mu2;

% Solve for theta1_ddot and theta2_ddot
sol = solve([eqn1, eqn2], [theta1_ddot, theta2_ddot]);
theta1_ddot_sol = simplify(sol.theta1_ddot);
theta2_ddot_sol = simplify(sol.theta2_ddot);

% Define the state vector x and input u
x = [theta1; theta2; theta1_dot; theta2_dot];

% Define the state derivative f(x,u)
f = [theta1_dot; theta2_dot; theta1_ddot_sol; theta2_ddot_sol];

% Compute the Jacobian matrices
A = jacobian(f, x);
B = jacobian(f, u);

% Evaluate at the equilibrium point theta1=0, theta2=pi, theta1_dot=0, theta2_dot=0
eq_point = [0; pi; 0; 0];

% Substitute the equilibrium point into A and B
A_lin = subs(A, x.', eq_point.');
B_lin = subs(B, x.', eq_point.');

% Simplify the linearized matrices
A_lin = simplify(A_lin);
B_lin = simplify(B_lin);

% Display the linearized matrices
disp('Linearized A matrix:');
disp(A_lin);

disp('Linearized B matrix:');
disp(B_lin);