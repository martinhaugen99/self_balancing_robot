%%%% inverted pendulum on cart -- cart-pole system

clc;
clear;
close all;


%% system equations

syms theta x dtheta dx ddtheta ddx u real
syms g M m l I real

eq1 = u - M*ddx - m*ddx - m*l*(dtheta^2*sin(theta) - ddtheta*cos(theta));

eq2 = (I + m*l^2)*ddtheta - m*l*(ddx*cos(theta) + g*sin(theta));

sol = solve([eq1==0, eq2==0], [ddx, ddtheta]);

% define state vector
x1 = theta; x2 = dtheta; x3 = x; x4 = dx;
X = [x1; x2; x3; x4];

% define state derivatives
f1 = x2;
f2 = simplify(sol.ddtheta);
f3 = x4;
f4 = simplify(sol.ddx);
f = [f1; f2; f3; f4];

% compute jacobians
A = simplify(jacobian(f, X));
B = simplify(jacobian(f, u));

% linearize around equlibrium point
x_eq = [0; 0; 0; 0;];
u_eq = 0;

A_eq = subs(A, [x1 x2 x3 x4 u], [x_eq.' u_eq]);
B_eq = subs(B, [x1 x2 x3 x4 u], [x_eq.' u_eq]);


%% parameters

params.g = 9.81;       % m/s^2
g = params.g;
params.M = 0.3;        % kg
M = params.M;
params.m = 0.5;        % kg
m = params.m;
params.l = 0.2;        % m
l = params.l;
params.I = (1/3)*params.m*(2*params.l)^2;    
I = params.I;


%% open-loop simulation

% NON-LINEAR
f_fun = matlabFunction(f, Vars=["theta","dtheta","x","dx","u", ...
    "g","M","m","l","I"]);

u = 0;

nonlin_ode = @(t,X) f_fun(X(1),X(2),X(3),X(4),u,g,M,m,l,I);

% small perturbation in angle
X_0 = [deg2rad(5); 0; 0; 0];

% equilibrium
% X_0 = x_eq;

% solve using ode45
[t_nl,X_nl] = ode45(nonlin_ode, [0 10], X_0);

figure;
subplot(2,1,1)
plot(t_nl,rad2deg(X_nl(:,1)),'b','LineWidth',1.5);
grid on;
ylabel('\theta (deg)')
title('Angular Evolution Over Time')

subplot(2,1,2)
plot(t_nl, X_nl(:,3), 'b', 'LineWidth', 1.5); 
grid on;
ylabel('Cart Position (m)')
title('Position Evolution Over Time')

% LINEAR
A_num = double(subs(A_eq));
B_num = double(subs(B_eq));

lin_ode = @(t,X) A_num*X + B_num*0;

[t_l,X_l] = ode45(lin_ode, [0 1], X_0);

figure;
subplot(2,1,1)
plot(t_l,rad2deg(X_l(:,1)),'b','LineWidth',1.5);
grid on;
ylabel('\theta (deg)')
title('Angular Evolution Over Time')

subplot(2,1,2)
plot(t_l, X_l(:,3), 'b', 'LineWidth', 1.5); 
grid on;
ylabel('Cart Position (m)')
title('Position Evolution Over Time')

% comparison
figure;
subplot(2,1,1)
plot(t_nl, rad2deg(X_nl(:,1)),'b','LineWidth',1.5);
hold on;
plot(t_l, rad2deg(X_l(:,1)),'r--','LineWidth',1.5);
grid on;
ylabel('\theta (deg)')
legend('Nonlinear', 'Linearized')
title('Angular Evolution Over Time')

subplot(2,1,2)
plot(t_nl, X_nl(:,3), 'b', 'LineWidth', 1.5); 
hold on;
plot(t_l, X_l(:,3), 'r--', 'LineWidth', 1.5);
grid on;
ylabel('Cart Position (m)')
xlabel('Time (s)')
legend('Nonlinear', 'Linearized')
title('Position Evolution Over Time')


%% system analysis

C = [1 0 0 0;
     0 1 0 0];  % theta, dtheta
D = 0;

sys = ss(A_num, B_num, C, D);

eigs = eig(sys);

for i = 1:size(eigs,1)
    if real(eigs(i)) > 0
        disp('System is unstable');
        break
    end
    disp('System is stable');
end

G = tf(sys);

Ctrb = ctrb(sys);
rank_ctrb = rank(Ctrb);
n = size(A_num,1);

if rank_ctrb == n
    disp("System is controllable");
else
    disp("System is NOT controllable");
end

Obsv = obsv(sys);
rank_obsv = rank(Obsv);

if rank_obsv == n
    disp("System is observable");
else
    disp("System is NOT observable");
end


%% PID controller

params.Ts = 0.005;         % sampling time
Ts = params.Ts;
params.T_end = 10;         % sim duration
T_end = params.T_end;

params.theta_ref = 0;      % reference angle
theta_ref = params.theta_ref;

% pid parameters
params.ku = 0; %7.87;
params.tu = 0; %44.5;
params.kp = 0; %params.ku*0.6;
params.ki = 0; %(1.2*params.ku)/params.tu;
params.kd = 0; %0.075*params.ku*params.tu;

X_0 = [deg2rad(5); 0; 0; 0];    % initial condition

% call ode with wrapper function
pid_ode = @(t,X) controlled_dynamics(t,X,params,f_fun);
[t,X] = ode45(pid_ode, 0:Ts:T_end, X_0);

%compute control input for plotting
U = zeros(size(t));

figure;
subplot(3,1,1)
plot(t,rad2deg(X(:,1)),'b','LineWidth',2);
grid on;
ylabel('\theta (deg)')
title('Angular Evolution Over Time With PID')

subplot(3,1,2)
plot(t,X(:,3),'r','LineWidth',2);
grid on;
ylabel('Car Position (m)')
title('Position Evolution Over Time With PID')

subplot(3,1,3)
plot(t, U, 'b', 'LineWidth', 2); 
grid on;
ylabel('Control Input')
title('PID Output')

clear controlled_dynamics;