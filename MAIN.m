
%                           MAIN
% MPC v. 2.3
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Main script for the MPC control simulation 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all

% Setting global variables
% integer
global MAIN_k;                                  % Current iteration
% matrices
global MAIN_M;                                  % Inertia matrix
global MAIN_invM;                               % Invert inertia matrix
global MAIN_J;                                  % Jacobian matrix 
%vectors
global MAIN_b;                                  % Centrifugal and Coriolis effects vector
global MAIN_g;                                  % Gravity effects vector
global MAIN_dotq;                               % Instant joint velocities
global MAIN_q;                                  % Instant joint positions
global MAIN_dotJ_dotq;                          % Product of dot_J and dot_q
global MAIN_distToWall;                         % Distance to operational obstacle (Wall)

% Global variables from other scripts
% From PARAMETERS
% boolean
global PARA_useSaveData;
global PARA_showIter;
global PARA_useOpeCons;
% integers
global PARA_n_EO;
% float
global PARA_t0;
global PARA_tend;
global PARA_deltat_simu;
% vectors
global PARA_q_0;
global PARA_dotq_0;
% SerialLink
global PARA_robot;

% From SAVEDATA
% vector
global SAVE_x_all;
global SAVE_dotq_all;
global SAVE_q_all;

% From MPC
% boolean
global MPC_stopComputation;

% From TASK
% boolean
global TASK_firstCall;


if PARA_showIter
    sprintf('----------------Running MPC Control simulation------------------')
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Import PARAMETERS

if PARA_showIter
    disp('Importing simulation parameters');
end

run('./PARAMETERS.m');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initializing

if PARA_showIter					   % PARA_showIter is used to provide console message while running. Computing time is gained when disabled 
    disp('Initializing MAIN variables');
end

MAIN_q = PARA_q_0;
MAIN_dotq = PARA_dotq_0;
MAIN_J = PARA_robot.jacob0(MAIN_q);

MAIN_J = MAIN_J(1:PARA_n_EO,:);

if strcmp(PARA_robot.name,'one link')
    MAIN_M = 1;                                             % .inertia does not work for one_link : end effector mass is set to 1 kg
    MAIN_b = 0;						    % There is no coupling effect : one_link only have one axis
    MAIN_g = 9.81*1*cos(MAIN_q);                            % .gravload does not work for one_link : gravity effects are computed following the current state, 1 is the lenght of the axis
else
    MAIN_M = PARA_robot.inertia(MAIN_q');
    MAIN_b =PARA_robot.coriolis(MAIN_q', MAIN_dotq')*MAIN_dotq;
    MAIN_g =PARA_robot.gravload(MAIN_q')';
end

MAIN_invM = inv(MAIN_M);

MAIN_dotJ_dotq = PARA_robot.jacob_dot(MAIN_q', MAIN_dotq');
if strcmp(PARA_robot.name,'two link')
    MAIN_dotJ_dotq = [-MAIN_dotJ_dotq(1,1);MAIN_dotJ_dotq(3,1);-MAIN_dotJ_dotq(2,1)];
else
	MAIN_dotJ_dotq = MAIN_dotJ_dotq(1:PARA_n_EO,1);
end

MAIN_k = 0;

TASK_firstCall = true;					   % This variable is used to pre-compute the trajectory to follow. it will be needed to compute the tasks

if PARA_useOpeCons 
    MAIN_distToWall = WALLDISTANCE( );			   % The distance from the end effector to the wall is computed in a separated function
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation loop

if PARA_showIter
    disp('Beginning of simulation loop');
end

for k = (PARA_t0/PARA_deltat_simu):(PARA_tend/PARA_deltat_simu)
    
    if PARA_showIter
        fprintf('Current iteration k : %2.3f - Time corresponding : %2.3f \n', k, k*PARA_deltat_simu)
    end
    
    % Running TASK
    run('./TASK.m');
    
    % Running MPC
    run('./MPC.m');
    
    % Checking oversight variable and updating MAIN_ variables
    if MPC_stopComputation
        disp('ERROR : optimization solver not able to compute');
        break;							    % If there is constraints incompatibility, the simulation is stopped
    else
        upda_out = UPDATEQ();					    % If not, current state of the robot is updated, following the control torque
        MAIN_dotq = upda_out{1};
        MAIN_q = upda_out{2};
        
	% The next variables are updated in loop following the new state
	
        main_J_tmp = PARA_robot.jacob0(MAIN_q);
               
        MAIN_J = main_J_tmp(1:PARA_n_EO,:);
        
        if strcmp(PARA_robot.name,'one link')
            MAIN_M = 1;                                             % .inertia does not work for one_link
            MAIN_b = 0;						    % There is no coupling effect :
            MAIN_g = 9.81*1*cos(MAIN_q);                            % .gravload does not work for one_link
        else
            MAIN_M = PARA_robot.inertia(MAIN_q');
            MAIN_b =PARA_robot.coriolis(MAIN_q', MAIN_dotq')*MAIN_dotq;
            MAIN_g =PARA_robot.gravload(MAIN_q')';
        end

        MAIN_dotJ_dotq = PARA_robot.jacob_dot(MAIN_q', MAIN_dotq');
        if strcmp(PARA_robot.name,'two link')
            MAIN_dotJ_dotq = [-MAIN_dotJ_dotq(1,1);MAIN_dotJ_dotq(3,1);-MAIN_dotJ_dotq(2,1)];
        else
            MAIN_dotJ_dotq = MAIN_dotJ_dotq(1:PARA_n_EO,1);
        end
        
        if PARA_useOpeCons 
            MAIN_distToWall = WALLDISTANCE( );
        end
    end
        

    
    % Saving data
    if PARA_useSaveData						   % Data are saved to be used for plot. If disabled, compting time is gained
        run ('./SAVEDATA.m');
    end
    
    MAIN_k = MAIN_k +1;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Saving initial datas
if PARA_useSaveData
    SAVE_dotq_all = [PARA_dotq_0, SAVE_dotq_all];
    SAVE_q_all = [PARA_q_0, SAVE_q_all];
    main_x = PARA_robot.fkine(PARA_q_0);
    main_rpy = tr2rpy(main_x)';
    SAVE_x_all = [[main_x(1:3,4);main_rpy],SAVE_x_all];

% Displaying plot
    if PARA_showIter
        disp('Displaying animation and plots');
    end
    run('./PLOTANIM.m');
end
