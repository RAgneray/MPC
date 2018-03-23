
%                   PARAMETERS_TEST
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%This script contains all the necessary variables needed by
%UPDATEQ_TEST

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%PARA
global PARA_useReduced;

global PARA_deltat_simu;

%MAIN
global MAIN_b;
global MAIN_g
global MAIN_q;
global MAIN_dotq;

global MAIN_invM;

%MPC 
global MPC_tau_app;
global MPC_khi_app;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

mdl_puma560;
robot = p560;

PARA_useReduced = false;

PARA_deltat_simu = 0.01;

MAIN_q = [0.1;0.2;0.1;0.5;0.4;0.3];
MAIN_dotq = [0.1;0.2;0.1;0.3;0.2;0.5];

MAIN_b = robot.coriolis(MAIN_q', MAIN_dotq')*MAIN_dotq;
MAIN_g = robot.gravload(MAIN_q')';

MAIN_invM = inv(robot.inertia(MAIN_q'));

MPC_tau_app = [5;-9;4;-2;6;8];
MPC_khi_app = [5;-9;4;-2;6;8; -0.02;0.06;0.05;0.07;-0.09;-0.07; 0.11;0.21;0.11;0.51;0.41;0.31; 0.12;0.22;0.12;0.32;0.22;0.52];

