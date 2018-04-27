
%                   TASK
% MPC v. 2.1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This script computes an previewed horizon of desired accelerations
% given a desired objective.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Setting global variables
% boolean
global TASK_firstCall;                           % Active when TASK is called for generating the reference trajectory
% matrix
global TASK_poserr;                              % Actual and reference position error
global TASK_velerr;                              % Actual and reference velocity error
global TASK_kpposerr;                            % Product of proportional gain and position error
global TASK_kdvelerr;                            % Product of derivative gain and velocity error
global TASK_ddotx_desN;                          % Desired op. acceleration
global TASK_ddotx_refN;                          % Reference op. acceleration
global TASK_dotx_refN;                           % Reference op. velocity
global TASK_x_refN;                              % Reference op. position
global TASK_q_ref;


% Global variables from other scripts
% From PARAMETERS
% boolean
global PARA_useReduced;
global PARA_useSaveData;
% integer
global PARA_n;
global PARA_N;
global PARA_n_EO;
% float
global PARA_kp;
global PARA_kd;
global PARA_ki;
global PARA_deltat_simu;
% vector
global PARA_x_des;
% SerialLink
global PARA_robot;

% from MAIN
% integer
global MAIN_k;
% matrices
global MAIN_invM;
% vectors
global MAIN_b;                                  
global MAIN_g;
global MAIN_q;
global MAIN_dotq;
global MAIN_dotJ_dotq; 

% from MPC
% vectors
global MPC_tau_app;     
global MPC_khi_app;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initializing

task_q=MAIN_q;
task_dotq=MAIN_dotq;
task_k = MAIN_k;

if TASK_firstCall
    task_ddotq = zeros(PARA_n,1);   
else
    if PARA_useReduced
        task_ddotq = MAIN_invM*(MPC_tau_app - MAIN_b - MAIN_g);
    else
        task_ddotq = MPC_khi_app((PARA_n+1):(2*PARA_n),1);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generating off-line trajectory

if TASK_firstCall
    task_trajOut = TRAJECTORY(PARA_x_des);

    TASK_ddotx_refN = task_trajOut{3};
    TASK_dotx_refN = task_trajOut{2};
    TASK_x_refN = task_trajOut{1};

    TASK_firstCall = task_trajOut{4};
    
    TASK_q_ref = task_trajOut{5};
    TASK_dotq_ref = task_trajOut{6};
    TASK_ddotq_ref = task_trajOut{7};
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Computing desired task with PID correction

TASK_ddotx_desN = zeros(PARA_n_EO,PARA_N);

% Variables used for diagnosing task issues
%if PARA_useSaveData
%    TASK_poserr = zeros(PARA_n_EO,PARA_N);                   
%    TASK_velerr = zeros(PARA_n_EO,PARA_N);                       
%    TASK_kpposerr = zeros(PARA_n_EO,PARA_N);                          
%    TASK_kdvelerr = zeros(PARA_n_EO,PARA_N);                            
%end

for i=1:(PARA_N+1)
    
    task_q_err = TASK_q_ref(:,task_k + i) - task_q;
    task_dotq_err = TASK_dotq_ref(:,task_k + i) - task_dotq;
    
    task_ddotq_des = TASK_ddotq_ref(:,task_k+i) + PARA_kp*task_q_err + PARA_kd*task_dotq_err + PARA_ki*(PARA_deltat_simu * task_q_err);
    
    task_x = PARA_robot.fkine(task_q);
    task_dotx = PARA_robot.jacob0(task_q)*task_dotq;
    if PARA_n_EO >3
        task_x = [task_x(1:3,4);tr2rpy(task_x(1:3,1:3))'];
    else
        task_x = task_x(1:PARA_n_EO,1);
    end
    task_dotx = task_dotx(1:PARA_n_EO,1);
    
    
    task_x_err = TASK_x_refN(:,task_k+i) - task_x;
    task_dotx_err = TASK_dotx_refN(:,task_k+i) - task_dotx;
    
    task_tmp = PARA_robot.jacob0(task_q);
    TASK_ddotx_desN(:,i) = task_tmp(1:PARA_n_EO,:)*task_ddotq_des + MAIN_dotJ_dotq;
    
    
    task_q = task_q + PARA_deltat_simu*task_dotq + 0.5*(PARA_deltat_simu^2)*task_ddotq_des;
    task_dotq = task_dotq + PARA_deltat_simu*task_ddotq_des;
    
    % Variables used for diagnosing task issues
    %if PARA_useSaveData
    %    TASK_poserr(:,i) = task_x_err;
    %    TASK_velerr(:,i) = task_dotx_err;
    %    TASK_kpposerr(:,i) = PARA_kp*task_x_err;
    %    TASK_kdvelerr(:,i) = PARA_kd*task_dotx_err;
    %end
end
