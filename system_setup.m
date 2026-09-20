%%%%% system setup

clc;
clear;
close all;


%% system variables

syms t g l I m M H V F theta(t) x(t)

% assume that some variables are positive, makes expressions clearer and
% restrict the returned solutions to be feasible
assume([t g l m M] > 0)

% also assume that the other variables are real
assume([I H V F], "real")


%% equations of motion in horizontal direction

% horizontal forces acting on the cart
eq1 = F - H == M*diff(x,2);

% horizontal forces acting on the pole
eq2 = H == m*diff(x-l*sin(theta),2);

% equation without the force H 
maineq1 = eliminate([eq1 eq2],H) == 0;


%% forces and moments acting on pole

% equation to express the forces on the pole along the direction normal to
% the pole
eq3 = V*sin(theta) + H*cos(theta) - m*g*sin(theta) == m*cos(theta)*diff(x,2) - m*l*diff(theta,2);

% poles angular momentum around its center of mass
eq4 = V*l*sin(theta) + H*l*cos(theta) == I*diff(theta,2);

% equation without H and V
maineq2 = eliminate([eq3 eq4],[H V]) == 0;


%% convert to state-space representation

[ssEqs,states] = odeToVectorField([maineq1 maineq2]);

% convert from symbolic form to matlab equation
Ydot = matlabFunction(ssEqs,Vars=["t","Y","F","I","M","g","l","m"]);


%% open-loop solution

F_val = 0;
M_val = 1;
g_val = 9.81;
l_val = 0.2;
m_val = 0.3;
I_val = (1/3)*m_val*(2*l_val)^2;

t_span = [0 10];

% Initial state: [theta, theta_dot, x, x_dot]
Y0 = [deg2rad(5); 0; 0; 0];  % Small perturbation from upright

odefun = @(t_1, Y_1) Ydot(t_1, Y_1, F_val, I_val, M_val, g_val, l_val, m_val);

% Solve using ode45
[t_sol, Y_sol] = ode45(odefun, t_span, Y0);

% Extract results
x_sol = Y_sol(:,3);
theta_sol = rad2deg(Y_sol(:,1));  % Convert to degrees for easier interpretation

figure;
subplot(2,1,1);
plot(t_sol, x_sol, 'b', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Cart Position x (m)');
title('Cart Motion');

subplot(2,1,2);
plot(t_sol, theta_sol, 'r', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Pendulum Angle \theta (deg)');
title('Pendulum Angle');


%% linearize symbolically

syms x dx theta dtheta real

Y = [theta; dtheta; x; dx];

% convert the Y[i] in ssEqs to yi
ssEqs_c = feval(symengine,'evalAt',ssEqs,'Y=[y1,y2,y3,y4]');

% substitute Y = [theta dtheta x dx]
ssEqs_sub = subs(ssEqs, [sym('Y',[1 4])], [theta, dtheta, x, dx]);

% evaluate jacobian with respect to yi
A = jacobian(ssEqs_sub, Y);
B = jacobian(ssEqs_sub, F);

% substitue in the equilibrium points
A_eq = subs(A, [sym('y',[1 4]), F], [0, 0, 0, 0, 0]);
B_eq = subs(B, [sym('y',[1 4]), F], [0, 0, 0, 0, 0]);