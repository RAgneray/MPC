
%                   PARAMETERS-MPC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This file contains the global variables and the simulated entry for the
% test of the element MPC

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%PARA
global PARA_useReduced;

global PARA_deltat_mpc;
global PARA_epsilon;
global PARA_omegak;

global PARA_n;
global PARA_N;
global PARA_n_EO;

%MAIN
global MAIN_invM;
global MAIN_J;
global MAIN_dotJ;

global MAIN_b;
global MAIN_g;
global MAIN_dotq;
global MAIN_dotJ_dotq;

%TASK
global TASK_ddotx_N;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global robot;
mdl_puma560;
robot = p560;

global MAIN_q;
MAIN_q = [0.1;0.1;0.1;0.2;0.1;0.1];

PARA_n=6;

global PARA_q_min;
global PARA_q_max;
global PARA_tau_min;
global PARA_tau_max;

PARA_q_min = [-1;-2;-1;-1;-1;-2];
PARA_q_max = [1;2;1;1;1;2];
PARA_tau_min = [-10;-9;-8;-7;-6;-5];
PARA_tau_max = [10;9;8;7;6;5];

global MAIN_M;
MAIN_M = robot.inertia(MAIN_q');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%ù
PARA_useReduced = false;

PARA_deltat_mpc = 0.01;
PARA_epsilon = 0.0001;
PARA_omegak = 1;

PARA_N = 10;
PARA_n_EO = 3;

MAIN_invM = inv(robot.inertia(MAIN_q'));
tmp_MAIN_J = robot.jacob0(MAIN_q);
MAIN_J = tmp_MAIN_J(1:PARA_n_EO,:);
tmp_prev_J = tmp_MAIN_J - robot.jacob0(MAIN_q-[0.01;0.01;0.01;0.01;0.01;0.01]);
MAIN_dotJ =1/(PARA_deltat_mpc^2)*tmp_prev_J(1:PARA_n_EO,:);

MAIN_dotq = [0.01;0.02;0.03;0.04;0.05;0.06];
tmp_MAIN_dotJ_dotq = robot.jacob_dot(MAIN_q,MAIN_dotq);
MAIN_dotJ_dotq = tmp_MAIN_dotJ_dotq(1:PARA_n_EO,:);

MAIN_b = robot.coriolis(MAIN_q',MAIN_dotq')*MAIN_dotq;
MAIN_g = (robot.gravload(MAIN_q'))';

TASK_ddotx_N = [0.10,0.11,0.12,0.13,0.14,0.15,0.16,0.17,0.18,0.19,0.2;0.10,0.11,0.12,0.13,0.14,0.15,0.16,0.17,0.18,0.19,0.2;0.10,0.11,0.12,0.13,0.14,0.15,0.16,0.17,0.18,0.19,0.2];
