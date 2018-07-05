%               PARAMETERS
% MPC v 2.3

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script initializes the variables needed to run MAIN
% It must be in the same folder as MAIN, to be properly called

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Setting global variables

% boolean
global PARA_useReduced;        % Allows the use of the reduced formulation
global PARA_useSaveData;       % Allows the storage of data during simulation
global PARA_useAnimation;      % Enables the animation at the end of simulation
global PARA_saveFig;           % Allows the save of figure
global PARA_showIter;          % Shows the curent iteration in command window
global PARA_useTorLimMin;      % Enables the use of Minimal Torque Limits
global PARA_useTorLimMax;      % Enables the use of Maximal Torque Limits
global PARA_usePosLimMin;      % Enables the use of Minimal Position Limits
global PARA_usePosLimMax;      % Enables the use of Maximal Position Limits
global PARA_useOpeCons;        % Enables the use of an operational Constraints (Wall)
global PARA_JointObj;          % Define the type of objective. 'true' for joint objective (only PARA_q_des have to be set), 'false' for operational one (only PARA_x_des have to be set).

% WARNING : the use of operational (end effector) objective can resolved in error in objective definition, because the computation
% method is using the Robotics Toolbox inverse kinematics function. Be careful while defining an operational obejcticve PARA_x_des
% Otherwise, there is no problem with joint objective (PARA_q_des)

% string
global PARA_solverSelect       % Select the quadratic solver

% integers
global PARA_N;                 % Length of the time horizon
global PARA_n;                 % Number of DOF (joint space)
global PARA_n_EO;              % Number of DOF (operational space)

% float
global PARA_deltat_simu;       % Time step used by simulation
global PARA_deltat_mpc;        % Time step used by MPC control
global PARA_epsilon;           % Weight of the regularization function
global PARA_omegak;            % Weight of the optimization function
global PARA_kp;                % Proportional gain
global PARA_kd;                % Derivative gain
global PARA_ki;                % Integral gain
global PARA_t0;                % Time of the beginning of the simulation
global PARA_tend;              % Time of the end of the simulation
global PARA_maxVel;            % Maximal velocity used by trajectory generator
global PARA_maxAcc;            % Maximal acceleration used by trajectory generator

% vectors
global PARA_q_des;             % Joint objective
global PARA_x_des;             % End effector objective
global PARA_q_min;             % Lower positions bounds
global PARA_q_max;             % Upper positions bounds
global PARA_tau_min;           % Lower torque limits
global PARA_tau_max;           % Upper torque limits
global PARA_q_0;               % Initial joint positions
global PARA_dotq_0;            % Initial joint velocities
global PARA_normalVect;        % Normal Vector defining the operational constraints (Wall)
global PARA_wallPoint;         % Point in the wall

% SerialLink
global PARA_robot;             % Imported robot model

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calling the Robotic Toolbox (Peter Corke)
run('../rvctools/startup_rvc.m');

% Adding path for OSQP
addpath('./osqp/osqp-0.3.0-matlab-linux64')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Setting global variables

% Control variables
PARA_useReduced = true;           % WARNING : detailed formualtion does not work yet, and has been given up. Always set 'PARA_useReduced' to 'true' and use reduced form       
PARA_useSaveData = true;
PARA_useAnimation = false;
PARA_saveFig = false;
PARA_showIter = true;

PARA_solverSelect ='quadprog'; % 'osqp';% 'cvx';% 
if ~any(strcmp(PARA_solverSelect,{'quadprog','cvx','osqp'}))     % Authorized values of solverSelect are 'quadprog', 'cvx' ans 'osqp'
    error('solverSelect value must be quadprog, cvx or osqp');  % This if-loop check if the value is correct
end
% WARNING : CVX and OSQP does not work with detailed formulation

PARA_useTorLimMin = true;      
PARA_useTorLimMax = true;      
PARA_usePosLimMin = true;      
PARA_usePosLimMax = true;

% Operationnal constraints (Wall)   (NOT yet Available in detailed formulation)
PARA_useOpeCons = true;            % WARNING : if used, PARA_n_EO cannot be more than 3
PARA_normalVect = [0;1;0];
PARA_normalVect = PARA_normalVect *1/norm(PARA_normalVect);
PARA_wallPoint = [0;0;0];

% Simulation
PARA_t0 = 0;
PARA_tend = 200.0;
PARA_deltat_simu = 0.001;

% MPC
PARA_deltat_mpc = PARA_deltat_simu;
PARA_N =299;
PARA_epsilon = 0.1;                 % While testing, the working weight ration omegak/epsilon is 10 000.
PARA_omegak = 1000;

% PID correction
PARA_kp = 1500;
PARA_kd = 2*sqrt(PARA_kp);
PARA_ki = 0;

% Robot model
mdl_puma560;%mdl_onelink;% mdl_twolink;%     % Model must be from the Robotics Toolbox (created or imported)
PARA_robot = p560;%onelink;%twolink;%

if strcmp(PARA_robot.name,'one link')
    
    % Test parameters for R robot
    PARA_n = PARA_robot.n;
    PARA_n_EO = 1;
    
    PARA_q_min = PARA_robot.qlim(:,1)+pi/2;
    PARA_q_max = PARA_robot.qlim(:,2)-pi/2;
    PARA_tau_min = -10;
    PARA_tau_max = 10;
    
    PARA_q_0 = 0.5;
    PARA_dotq_0 = 0;
    PARA_maxVel = 0.5;%0.05;        
    PARA_maxAcc = NaN;
    PARA_x_des =[-1;0];%[0;1];%[sqrt(2)*0.5;-sqrt(2)*0.5];
    
elseif strcmp(PARA_robot.name,'two link')
    
    % Test parameters for RR robot (XZ-plane)
    PARA_n = PARA_robot.n;
    PARA_n_EO = 3;
    
    PARA_q_min = PARA_robot.qlim(:,1)+3*pi/4;
    PARA_q_max = PARA_robot.qlim(:,2)-3*pi/4;
    PARA_tau_min = [-20;-10];
    PARA_tau_max = [20;10];
    
    PARA_q_0 = [0.5;0.5];
    PARA_dotq_0 = [0;0];
    PARA_maxVel = 0.5;        
    PARA_maxAcc = NaN;
    
    PARA_JointObj = true;
    PARA_x_des =[0.2;0;0.5];
    PARA_q_des =[0;0];
    
elseif strcmp(PARA_robot.name,'Puma 560')
    
    % Test parameters for PUMA 560
    PARA_n = PARA_robot.n;
    PARA_n_EO = 6;
    
    PARA_q_min = PARA_robot.qlim(:,1);
    PARA_q_max = PARA_robot.qlim(:,2);
    PARA_tau_min = [-100;-80;-60;-40;-20;-10];
    PARA_tau_max = [100;80;60;40;20;10];
    
    PARA_q_0 = [0.5;0.5;0.5;0.5;0.5;0.5];
    PARA_dotq_0 = [0;0;0;0;0;0];
    PARA_maxVel = 0.05;        
    PARA_maxAcc = NaN;
    
    PARA_JointObj = true;
    PARA_x_des =[0.5;0.2;0.3;0;0;0];
    % If orientation is used, it must be described in RPY convention
    PARA_q_des =[0;0;0;0;0;0];
end
