%               PARAMETERS_TEST
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global PARA_useReduced;
global PARA_useSaveData;

global PARA_N;
global PARA_n;
global PARA_n_EO;

global PARA_deltat_simu;
global PARA_deltat_mpc;
global PARA_epsilon;
global PARA_omegak;
global PARA_kp;
global PARA_kd;
global PARA_t0;
global PARA_tend;
global PARA_x_des;

global PARA_q_min;
global PARA_q_max;
global PARA_tau_min;
global PARA_tau_max;
global PARA_q_0;
global PARA_dotq_0;

global PARA_robot;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
run('../rvctools/startup_rvc.m');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

PARA_useReduced = true;
PARA_useSaveData = true;

PARA_N = 10;

PARA_deltat_simu = 0.01;
PARA_deltat_mpc = PARA_deltat_simu;

PARA_epsilon = 0.0001;
PARA_omegak = 1;

PARA_kp = 10.0;
PARA_kd = 2*sqrt(PARA_kp);

PARA_t0 = 0;
PARA_tend = 5.0;

PARA_x_des = [0.25;0.25;0.25];
PARA_n_EO = size(PARA_x_des,1);

mdl_puma560;
PARA_robot = p560;

PARA_n = PARA_robot.n;
PARA_q_min = PARA_robot.qlim(:,1);
PARA_q_max = PARA_robot.qlim(:,2);

PARA_tau_min = [-100; -80; -60; -40; -20; -10];
PARA_tau_max = [100; 80; 60; 40; 20; 10];

PARA_q_0 = [0; pi/2; -pi/2; 1; 0; 0];
PARA_dotq_0 = [0;0;0;0;0;0];