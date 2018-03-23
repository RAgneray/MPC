
%                   PARAMETERS_TEST
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%This script contains all the necessary variables needed by
%TASK_TEST

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%PARA
global PARA_N;

global PARA_kp;
global PARA_kd;
global PARA_t0;
global PARA_deltat_simu;

global PARA_x_des;

global PARA_robot;

%MAIN
global MAIN_dotq;
global MAIN_q;

global MAIN_J;
global MAIN_dotJ;
global MAIN_invJ;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
PARA_N = 500;
mdl_puma560;
PARA_robot = p560;
PARA_kp = 0.2;
PARA_kd = 2*sqrt(PARA_kp);
PARA_t0 =0;

PARA_deltat_simu = 0.01;

PARA_x_des = [1;1;1];

MAIN_dotq = [0.1;0.2;0.1;0.3;0.2;0.5];
MAIN_q = [0.1;0.4;-0.2;0.1;0.2;0.6];

MAIN_J = PARA_robot.jacob0(MAIN_q);
MAIN_invJ = MAIN_J'*inv(MAIN_J*MAIN_J');
MAIN_J = MAIN_J(1:3,:);
MAIN_invJ = MAIN_invJ(:,1:3);
MAIN_dotJ = PARA_robot.jacob_dot(MAIN_q,MAIN_dotq);
MAIN_dotJ = MAIN_dotJ(1:3,:);

