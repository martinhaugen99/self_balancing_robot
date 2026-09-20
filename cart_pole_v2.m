%%% cart-pole system 

clc;
clear;
close all;


%% system equations

syms theta x dtheta dx ddtheta ddx u real
syms g M m l J real

eq1 = u - M*ddx - m*ddx - m*l*(dtheta^2*sin(theta) - ddtheta*cos(theta));

eq2 = (J + m*l^2)*ddtheta - m*l*(ddx*cos(theta) + g*sin(theta));

sol = solve([eq1==0, eq2==0], [ddx, ddtheta]);

% define state vector
x1 = x; x2 = dx; x3 = theta; x4 = dtheta;
X = [x1; x2; x3; x4];

% define state derivatives
f1 = x2;
f2 = simplify(sol.ddx);
f3 = x4;
f4 = simplify(sol.ddtheta);
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
g = 9.81;       
M = 0.27;               % cart mass
m_chassis = 0.37;       % rod mass
m_pb = 0.15;            % power bank mass
m_md = 0.03;            % motor driver mass
m_stm = 0.04;           % stm board mass
m_bb = 0.10;            % breadboard mass

m = m_chassis + m_pb + m_md + m_stm + m_bb;     % total body mass

l = 0.135/2;            % half of rod length

J = 0.006767;           % composite modeling


%% numerical linearized system

A_num = double(subs(A_eq));
B_num = double(subs(B_eq));

C = [0 0 1 0];      % theta output
D = 0;

sys = ss(A_num,B_num,C,D);

%pidTuner(sys,'PID')

%% simulink model
T_sim = 5;

theta_ref = 0;      % reference value, want to keep pendulum at 0(upright)
X_0 = [0; 0; deg2rad(5); 0]; % initial condition

sim = sim("cart_pole_nonlinear.slx");

X = sim.X.Data;
t = sim.X.Time;
U = sim.u.Data;

figure(1);
subplot(2,1,1);
plot(t,rad2deg(X(:,3)),'b','LineWidth',1.5);
grid on;
ylabel('\theta (deg)');
title('Angular Evolution with PID');

subplot(2,1,2);
plot(t,U,'r','LineWidth',1.5);
grid on;
ylabel('Control Input');
title('Control Input with PID');


%% adding noise to sensor data

n = size(t,1);      % number of samples

theta_true = X(:,3);
%theta_true = zeros(1,n);

sigma = 0.01;       % noise standard deviation
noise = sigma * randn(size(theta_true));

theta_noise = theta_true + noise;

figure(2);
plot(t,rad2deg(theta_true),'b','LineWidth',1.5); hold on;
plot(t,rad2deg(theta_noise),'or','LineWidth',1.5); hold on;
grid on;
legend('True \theta', 'Measured \theta');
ylabel('\theta (deg)')
title('Sensor Noise Simulation');


%% Kalman filter

dt = 0.0005;     % sample time

sys_d = c2d(sys,dt);        % discretized system

A_d = sys_d.A;
B_d = sys_d.B;
C_d = sys_d.C;

% noise covariances
Q = diag([1e-2 1e-2 1e-7 1e-2]);        % process noise
R = sigma^2;                            % measurement noise

x_hat = zeros(4,n);     % state estimate
x_hat(:,1) = X_0;       % initial state

P = 0.1 * eye(4);        % initial error covariance

% kalman loop
for k = 1:n-1
    % prediction
    x_p = A_d * x_hat(:,k) + B_d * U(k);
    P = A_d * P * A_d' + Q;

    % measurement update
    y = theta_noise(k);                         % measurement
    S = C_d * P * C_d' + R;                     % innovation covariance
    K = P * C_d' / S;                           % kalman gain
    x_p = x_p + K * (y - C_d * x_p);            % state update
    P = (eye(4) - K * C_d) * P;                 % covariance update

    x_hat(:,k+1) = x_p;
end

figure(2);
plot(t,rad2deg(x_hat(3,:)),'k','LineWidth',2,'DisplayName','Estimated \theta');
hold off;
grid on;
