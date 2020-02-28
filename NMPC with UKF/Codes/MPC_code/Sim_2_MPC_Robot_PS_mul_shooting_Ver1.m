% point stabilization + Multiple shooting
clear all; close all; clc;

% Import the CasADi v3.5.1 and add it to path
addpath('C:\Matlab Toolboxes\tbxmanager\toolboxes\casadi-windows-matlabR2016a-v3.5.1');
import casadi.*

T    = 0.2; % Time step[s]
N    = 10; % prediction horizon
rDia = 0.3; % Robot diameter 

% Input Constraints
v_max     = 0.6; 
v_min     = -v_max;
omega_max = pi/4; 
omega_min = -omega_max;

% Create state variables for using in Casadi
x        = SX.sym('x'); 
y        = SX.sym('y'); 
theta    = SX.sym('theta');

% Create the robot state
states   = [x;y;theta]; 
n_states = length(states);

% Create input variables for using in Casadi
v     = SX.sym('v'); 
omega = SX.sym('omega');

% Create the robot input
controls   = [v;omega]; 
n_controls = length(controls);

% Define the Nonlinear state equation
rhs = [v*cos(theta);v*sin(theta);omega]; 

% Nonlinear State Update function f(x,u)
% Given states and controls as input, returns rhs in terms of the inputs
f = Function('f',{states,controls},{rhs}); 

% Define parameters (which include initial state and the reference state)
U = SX.sym('U',n_controls,N); % Decision variables (controls)
P = SX.sym('P',n_states + n_states);

% A vector that represents the states over the optimization problem.
X = SX.sym('X',n_states,(N+1));

% Objective function - will be defined shortly below
obj = 0; 
% constraints vector
g = [];  

% Define the weighing matrices Q for states and R for inputs
Q = diag([1 5 0.1]); 
R = diag([0.5 0.05]); 

% Specify initial state
st_k = X(:,1); 
% Add the initial state constraint
g = [g; st_k-P(1:3)]; 

% For all remaining time steps, form the obj_fun and update constraints
for k = 1:N
    st_k    = X(:,k);  
    u_k     = U(:,k);
    x_error = st_k-P(4:6);
    obj     = obj + x_error'*Q*x_error + u_k'*R*u_k; % calculate obj
    st_next = X(:,k+1);
    f_value = f(st_k,u_k);
    st_pred = st_k+ (T*f_value);
    % Add the constraints
    g = [g; st_next-st_pred]; % compute constraints
end

% make the decision variable one column  vector
OPT_variables = [reshape(X,3*(N+1),1);reshape(U,2*N,1)];

% Create the nlp problem structure with obj_fun, variables & constraints
nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

% Feed the solver options
opts = struct;
opts.ipopt.max_iter                  = 2000;
opts.ipopt.print_level               = 0;%0,3
opts.print_time                      = 0;
opts.ipopt.acceptable_tol            = 1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

% Call the solver ipopt with problem struct and options
solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

% Create the arguments structure to hold the constraint values
args = struct;

% Dynamics Constraint
args.lbg(1:3*(N+1)) = 0;  % -1e-20 - Equality constraints
args.ubg(1:3*(N+1)) = 0;  % 1e-20  - Equality constraints

% State Constraints
args.lbx(1:3:3*(N+1),1) = -2;   % state x lower bound
args.ubx(1:3:3*(N+1),1) = 2;    % state x upper bound
args.lbx(2:3:3*(N+1),1) = -2;   % state y lower bound
args.ubx(2:3:3*(N+1),1) = 2;    % state y upper bound
args.lbx(3:3:3*(N+1),1) = -inf; % state theta lower bound
args.ubx(3:3:3*(N+1),1) = inf;  % state theta upper bound

% Input Constraints
args.lbx(3*(N+1)+1:2:3*(N+1)+2*N,1) = v_min; %v lower bound
args.ubx(3*(N+1)+1:2:3*(N+1)+2*N,1) = v_max; %v upper bound
args.lbx(3*(N+1)+2:2:3*(N+1)+2*N,1) = omega_min; %omega lower bound
args.ubx(3*(N+1)+2:2:3*(N+1)+2*N,1) = omega_max; %omega upper bound
%----------------------------------------------
% ALL OF THE ABOVE IS JUST A PROBLEM SET UP

% THE SIMULATION LOOP SHOULD START FROM HERE
%-------------------------------------------
t0       = 0;                 % Initial time for each MPC iteration
x0       = [0 ; 0 ; 0.0];     % initial condition.
x_ref    = [1.5 ; 1.5 ; 0.0]; % Reference posture.
sim_time = 20;                % Maximum simulation time

% Define Data Structures
st_hist(:,1) = x0; % contains the history of states
time_hist(1) = t0; % contains the history of initial timess

u0 = zeros(N,2);        % two control inputs for each robot
X0 = repmat(x0,1,N+1)'; % initialization of the states decision variables

% Start NMPC
mpciter = 0;  % MPC iteration counter
x_mpc   = []; % Data structure to store mpc predicted states
u_mpc   = []; % Data structure to store mpc inputs

% NMPC simulaton loop: Loop until error > 10^-6 & #. MPC steps < max value.
main_loop = tic;
while(norm((x0-x_ref),2) > 1e-2 && mpciter < sim_time / T)
    % set the values of the parameters vector
    args.p = [x0;x_ref]; 
    % initial value of the optimization variables
    args.x0 = [reshape(X0',3*(N+1),1);reshape(u0',2*N,1)];
    % Solve the NMPC using IPOPT solver
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
        'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
    % Extract the minimizing control
    u = reshape(full(sol.x(3*(N+1)+1:end))',2,N)'; 
    % Get solution TRAJECTORY    
    x_mpc(:,1:3,mpciter+1) = reshape(full(sol.x(1:3*(N+1)))',3,N+1)';
    % Store only the input at the first time step
    u_mpc= [u_mpc ; u(1,:)];    
    % Update the time history
    time_hist(mpciter+1) = t0;
    % Apply the control and shift the solution
    [t0, x0, u0] = shift(T, t0, x0, u,f);    
    % Update the state history
    st_hist(:,mpciter+2) = x0;
    % Reshape and get solution TRAJECTORY    
    X0 = reshape(full(sol.x(1:3*(N+1)))',3,N+1)'; 
    % Shift trajectory to initialize the next step
    X0 = [X0(2:end,:);X0(end,:)];
    % Display MPC iteration number
    mpciter
    % Increment the MPC iteration number
    mpciter = mpciter + 1;
end

% Record the looping time 
main_loop_time = toc(main_loop);

% Error specifying how close we are w.r.t the reference
ss_error = norm((x0-x_ref),2);

% Record average mpc time = total_time / #. MPC interations
disp(['Average MPC Time = ', num2str(main_loop_time/(mpciter+1))]);

% Draw the robot trajectory with MPC inputs and predictions
Draw_MPC_point_stabilization_v1(time_hist,st_hist,x_mpc,u_mpc,x_ref,N,rDia)