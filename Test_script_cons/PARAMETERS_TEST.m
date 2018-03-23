
%                   PARAMETERS_TEST
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%This script contains all the necessary variables needed by
%CONSTRAINTS_TEST

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%PARA
global PARA_useReduced;

global PARA_deltat_mpc;

global PARA_N;
global PARA_n;

global PARA_q_min;
global PARA_q_max;
global PARA_tau_min;
global PARA_tau_max;

%MAIN
global MAIN_M;
global MAIN_invM;

global MAIN_b;
global MAIN_g;
global MAIN_dotq;
global MAIN_q;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
PARA_useReduced = false;

PARA_deltat_mpc = 0.01;

PARA_N = 10;
PARA_n = 6;

PARA_q_min = [-1;-2;-3;-1;-2;-3];
PARA_q_max = [2;2;2;2;2;2];
PARA_tau_min = [-100;-50;-5;-8;-4;-2];
PARA_tau_max = [80;40;2;8;4;2];

MAIN_M = [2,0,0,0,0,0;0,5,0,0,0,0;0,0,6,0,0,0;0,0,0,2,0,0;0,0,0,0,3,0;0,0,0,0,0,7];
MAIN_invM = inv(MAIN_M);

MAIN_b = [0;5;2;5;2;5];
MAIN_g = [10;5;5;10;10;10];
MAIN_dotq = [0.1;0.2;0.1;0.3;0.2;0.1];
MAIN_q = [0.5;0.5;0.5;0.5;0.5;0.5];