%           PARAMETERS_save
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%this script contains the global variables used for the test of SAVEDATA

%PARA
global PARA_useReduced;

global PARA_deltat_simu;

global PARA_q_min;
global PARA_q_max;
global PARA_tau_min;
global PARA_tau_max;

global PARA_robot;

%MAIN
global MAIN_invM;
global MAIN_J;
global MAIN_dotJ;

global MAIN_b;
global MAIN_g;
global MAIN_dotq;
global MAIN_q;

%MPC
global MPC_tau_app;
global MPC_khi_app;

%TASK
global TASK_ddotx_N;
global TASK_x_des;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

PARA_useReduced = true;

PARA_deltat_simu = 0.01;

PARA_q_min = [-0.1;-0.2;-0.3;-0.1;-0.3;-0.2];
PARA_q_max = [0.1;0.2;0.3;0.1;0.3;0.2];
PARA_tau_min = [-6;-5;-4;-3;-2;-1];
PARA_tau_max = [6;5;4;3;2;1];

MAIN_dotq = [0.1;0.2;0.1;0.3;0.2;0.5];
MAIN_q = [0.1;0.4;-0.2;0.1;0.2;0.6];

mdl_puma560;
PARA_robot = p560;

MAIN_b = PARA_robot.coriolis(MAIN_q', MAIN_dotq')*MAIN_dotq;
MAIN_g = PARA_robot.gravload(MAIN_q')';

MAIN_invM = inv(PARA_robot.inertia(MAIN_q'));
MAIN_J = PARA_robot.jacob0(MAIN_q);
MAIN_J = MAIN_J(1:3,:);
MAIN_dotJ = PARA_robot.jacob_dot(MAIN_q,MAIN_dotq);
MAIN_dotJ = MAIN_dotJ(1:3,:);

MPC_tau_app = [5;-9;4;-2;6;8];
MPC_khi_app = [5;-9;4;-2;6;8; -0.02;0.06;0.05;0.07;-0.09;-0.07; 0.11;0.21;0.11;0.51;0.41;0.31; 0.12;0.22;0.12;0.32;0.22;0.52];

TASK_ddotx_N = [1,1,1;2,2,2;3,4,5];
TASK_x_des = [0.5;0.5;0.5];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global SAVE_tau_all;
global SAVE_tau_minall;
global SAVE_tau_maxall;
global SAVE_ddotq_all;
global SAVE_dotq_all;
global SAVE_q_all;
global SAVE_ddotq_minposall;
global SAVE_ddotq_maxposall;
global SAVE_ddotx_all;
global SAVE_ddotx_desall;
global SAVE_x_all
global SAVE_x_desall;


SAVE_tau_all = [];
SAVE_tau_minall = [];
SAVE_tau_maxall = [];
SAVE_ddotq_all = [];
SAVE_dotq_all = [];
SAVE_q_all = [];
SAVE_ddotq_minposall = [];
SAVE_ddotq_maxposall = [];
SAVE_ddotx_all = [];
SAVE_ddotx_desall =[];
SAVE_x_all =[];
SAVE_x_desall = [];