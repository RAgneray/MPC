%                   MAIN
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
run('../test_script_para/PARAMETERS_test.m');

%%%%%%%%%%%%%%%%%%%%%%%%%%

global PARA_useSaveData;
global MPC_stopComputation;

global PARA_n;
global PARA_n_EO;

global PARA_t0;
global PARA_tend;
global PARA_deltat_simu;

global MAIN_M;
global MAIN_invM;
global MAIN_J;
global MAIN_dotJ;
global MAIN_invJ;

global PARA_q_0;
global PARA_dotq_0;
global MAIN_b;
global MAIN_g;
global MAIN_dotq;
global MAIN_q;
global MAIN_dotJ_dotq;

global PARA_robot;

global SAVE_x_all;
global SAVE_dotq_all;
global SAVE_q_all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

MAIN_q = PARA_q_0;
MAIN_dotq = PARA_dotq_0;
MAIN_J = PARA_robot.jacob0(MAIN_q);
MAIN_dotJ = zeros(PARA_n_EO, PARA_n);
MAIN_invJ = MAIN_J'*inv(MAIN_J*MAIN_J');
MAIN_dotJ = MAIN_dotJ(1:PARA_n_EO,:);
MAIN_invJ = MAIN_invJ(:,1:PARA_n_EO);
MAIN_J = MAIN_J(1:PARA_n_EO,:);


for k = (PARA_t0/PARA_deltat_simu):(PARA_tend/PARA_deltat_simu)
    
    MAIN_M = PARA_robot.inertia(MAIN_q');
    MAIN_invM = inv(MAIN_M);
    MAIN_b = PARA_robot.coriolis(MAIN_q', MAIN_dotq')*MAIN_dotq;
    MAIN_g = PARA_robot.gravload(MAIN_q')';
    MAIN_dotJ_dotq = PARA_robot.jacob_dot(MAIN_q', MAIN_dotq');
    %run Task
    %run MPC
    if MPC_stopComputation
        break;
    else 
        %run UPDATEQ
    end
    
    if PARA_useSaveData
        %run SAVEDATA
    end
    
    MAIN_q = upda_out{2};
    MAIN_dotq = upda_out{1};
    main_J_tmp = PARA_robot.jacob0(MAIN_q);
    MAIN_dotJ = (main_J_tmp(1:PARA_n_EO,:) - MAIN_J)*1/PARA_deltat_simu;
    MAIN_invJ = main_J_tmp'*inv(main_J_tmp*main_J_tmp');
    MAIN_dotJ = MAIN_dotJ(1:PARA_n_EO,:);
    MAIN_invJ = MAIN_invJ(:,1:PARA_n_EO);
    MAIN_J = main_J_tmp(1:PARA_n_EO,:);
    
end

if PARA_useSaveData
    SAVE_dotq_all=[PARA_dotq_0, SAVE_dotq_all];
    SAVE_q_all = [PARA_q_0, MAIN_q];
    main_x = PARA_robot.fkine(PARA_q_0);
    SAVE_x_all=[main_x(1:3,4),SAVE_x_all];
    %run PLOTANIM
end