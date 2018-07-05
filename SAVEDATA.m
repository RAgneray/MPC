
%                       SAVEDATA
% MPC v. 2.3
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This script allows to save datas at each simulation iteration
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Setting up global variables
% matrices
global SAVE_tau_all;                                    % Contains all the applied control torques
global SAVE_tau_minall;                                 % Contains minimal torque bound
global SAVE_tau_maxall;                                 % Contains maximal torque bound
global SAVE_ddotq_all;                                  % Contains all the joint accelerations
global SAVE_dotq_all;                                   % Contains all the joint velocities
global SAVE_q_all;                                      % Contains all the joint positions
global SAVE_ddotq_minposall;                            % Contains all the minimal joint acceleration limits
global SAVE_ddotq_maxposall;                            % Contains all the maximal joint acceleration limits
global SAVE_ddotx_all;                                  % Contains all the operational accelerations
global SAVE_ddotx_refall;                               % Contains all the operational reference accelerations
global SAVE_ddotx_desall;                               % Contains all the operational desired accelerations
global SAVE_x_all;                                      % Contains all the operational positions (and orientations)
global SAVE_x_refall;                                   % Contains all the operational reference positions (and orientations)


% Calling global variables from other script
% From PARAMETERS
% boolean
global PARA_useReduced;
% integer
global PARA_n
% float
global PARA_deltat_simu;
% vectors
global PARA_q_min;
global PARA_q_max;
global PARA_tau_min;
global PARA_tau_max;
% SerialLink
global PARA_robot;

% From MAIN
% matrices
global MAIN_invM;
global MAIN_J;
% vectors
global MAIN_dotJ_dotq;
global MAIN_b;
global MAIN_g;
global MAIN_dotq;
global MAIN_q;

% From MPC
% vectors
global MPC_tau_app;
global MPC_khi_app;

% From TASK
% matrices
global TASK_ddotx_desN;                         
global TASK_ddotx_refN;                          
global TASK_x_refN;
                                                  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% If using reduced form (SAVE variables are iteratively concatenate)
if PARA_useReduced
    SAVE_tau_all=[SAVE_tau_all, MPC_tau_app];
    SAVE_tau_minall = [SAVE_tau_minall, PARA_tau_min];
    SAVE_tau_maxall = [SAVE_tau_maxall, PARA_tau_max];
    SAVE_ddotq_all=[SAVE_ddotq_all, MAIN_invM*(MPC_tau_app - MAIN_b - MAIN_g)];
    SAVE_dotq_all=[SAVE_dotq_all, MAIN_dotq];
    SAVE_q_all = [SAVE_q_all, MAIN_q];
    SAVE_ddotq_minposall = [SAVE_ddotq_minposall, 2/(PARA_deltat_simu^2.0)*(PARA_q_min-MAIN_q-PARA_deltat_simu*MAIN_dotq)];
    SAVE_ddotq_maxposall = [SAVE_ddotq_maxposall, 2/(PARA_deltat_simu^2.0)*(PARA_q_max-MAIN_q-PARA_deltat_simu*MAIN_dotq)];
    SAVE_ddotx_all = [SAVE_ddotx_all, MAIN_J*MAIN_invM*(MPC_tau_app - MAIN_b - MAIN_g) + MAIN_dotJ_dotq];
    SAVE_ddotx_refall = TASK_ddotx_refN;
    SAVE_ddotx_desall= [SAVE_ddotx_desall, TASK_ddotx_desN(:,1)];
    save_x = PARA_robot.fkine(MAIN_q);
    save_rpy = tr2rpy(save_x);
    SAVE_x_all=[SAVE_x_all,[save_x(1:3,4);save_rpy']];
    SAVE_x_refall= TASK_x_refN;
    
    
% If using detailed form

% WARNING : detailed form have been implanted but does not work properly yet. The obtained results using this method are bad control sequences most of the time 
% Moreover, this form have been given up during the coding process because of its computing cost

else
    SAVE_tau_all=[SAVE_tau_all, MPC_khi_app(1:PARA_n,1)];
    SAVE_tau_minall = [SAVE_tau_minall, PARA_tau_min];
    SAVE_tau_maxall = [SAVE_tau_maxall, PARA_tau_max];
    SAVE_ddotq_all=[SAVE_ddotq_all, MPC_khi_app((PARA_n+1):(2*PARA_n),1)];
    SAVE_dotq_all=[SAVE_dotq_all, MAIN_dotq];
    SAVE_q_all = [SAVE_q_all, MAIN_q];
    SAVE_ddotq_minposall = [SAVE_ddotq_minposall, 2/(PARA_deltat_simu^2.0)*(PARA_q_min-MAIN_q-PARA_deltat_simu*MAIN_dotq)];
    SAVE_ddotq_maxposall = [SAVE_ddotq_maxposall, 2/(PARA_deltat_simu^2.0)*(PARA_q_max-MAIN_q-PARA_deltat_simu*MAIN_dotq)];
    SAVE_ddotx_all = [SAVE_ddotx_all, MAIN_J*MPC_khi_app((PARA_n+1):(2*PARA_n),1) + MAIN_dotJ_dotq];
    SAVE_ddotx_refall = TASK_ddotx_refN;
    SAVE_ddotx_desall= [SAVE_ddotx_desall, TASK_ddotx_desN(:,1)];
    save_x = PARA_robot.fkine(MAIN_q);
    save_rpy = tr2rpy(save_x);
    SAVE_x_all=[SAVE_x_all,[save_x(1:3,4);save_rpy']];
    SAVE_x_refall= TASK_x_refN;

end
