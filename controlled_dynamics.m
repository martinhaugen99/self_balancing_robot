function dXdt = controlled_dynamics(t,X,params,f_fun)
% states
theta = X(1);
dtheta = X(2);
x = X(3);
dx = X(4);

% persistent controller memory
persistent prev_error integral_error last_t
if isempty(prev_error)
    prev_error = 0;
    integral_error = 0;
    last_t = 0;
end

% time step (approx)
dt = t - last_t;
if dt == 0
    dt = params.Ts;
end

% pid control
error = params.theta_ref - theta;
derivative_error = (error - prev_error)/dt;
integral_error = integral_error + error*dt;
u = params.kp*error + params.ki*integral_error + params.kd*derivative_error;

% update memory
prev_error = error;
last_t = t;

% system dynamics
dXdt = f_fun(theta,dtheta,x,dx,u,params.g,params.M,params.m,params.l, ...
    params.I);
end