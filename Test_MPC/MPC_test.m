
%                   MPC_test
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This file is used for test on the MPC element
% it is linked to the PARAMETERS_MPC file

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%call global variables
%PARA
global PARA_useReduced;

global PARA_deltat_mpc;
global PARA_epsilon;
global PARA_omegak;

global PARA_n
global PARA_N;
global PARA_n_EO;

%MAIN
global MAIN_invM;
global MAIN_J;
global MAIN_dotJ_dotq;

global MAIN_dotJ;
global MAIN_b;
global MAIN_g;
global MAIN_dotq;

%TASK
global TASK_ddotx_N;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global MPC_stopComputation;
global MPC_tau_app;
global MPC_khi_app;

MPC_stopComputation = false;

if PARA_useReduced
    
    %Initialization
    mpc_E = zeros(PARA_n_EO*(PARA_N + 1), PARA_n * (PARA_N + 1));
    mpc_f = zeros(PARA_n_EO*(PARA_N +1), 1);
    for i = 1:(PARA_N+1)
        for j = 1:i
            mpc_E((PARA_n_EO*(i-1)+1):(PARA_n_EO*i), (PARA_n*(j-1)+1):(PARA_n*j)) = PARA_deltat_mpc*MAIN_dotJ*MAIN_invM;
        end
        mpc_E((PARA_n_EO*(i-1)+1):(PARA_n_EO*i), (PARA_n*(i-1)+1):(PARA_n*j)) = MAIN_J*MAIN_invM;
        mpc_f((PARA_n_EO*(i-1)+1):(PARA_n_EO*i), 1) = TASK_ddotx_N(:,i) + MAIN_J*MAIN_invM*(MAIN_b+MAIN_g) + (i-1)*PARA_deltat_mpc*MAIN_dotJ*MAIN_invM*(MAIN_b+MAIN_g)-MAIN_dotJ_dotq;
    end
    mpc_Q = mpc_E'*mpc_E;
    mpc_p = -(mpc_E'*mpc_f)';
    
    cons_out = fct_cons();
    
    mpc_qpOptions = optimset('Algorithm','interior-point-convex','Display','off');
    [mpc_tau_N,~,EXITFLAG] = quadprog(mpc_Q,mpc_p,cons_out{1},cons_out{2},[],[],[],[],[],mpc_qpOptions);
    if EXITFLAG == -2
        MPC_stopComputation = true;
    else
        MPC_tau_app = mpc_tau_N(1:6,1);
    end
    
else
    mpc_E = zeros(PARA_n_EO*(PARA_N + 1), 2*PARA_n*(2*PARA_N + 1));
    mpc_f = zeros(PARA_n_EO*(PARA_N + 1),1);
    
    for i = 1:(PARA_N)
        mpc_E((PARA_n_EO*i +1):(PARA_n_EO*(i+1)), (2*PARA_n + (i-1)*4*PARA_n +1):(2*PARA_n + 4*(i-1)*PARA_n + PARA_n)) = MAIN_dotJ;
        mpc_E((PARA_n_EO*i +1):(PARA_n_EO*(i+1)), (5*PARA_n + (i-1)*4*PARA_n +1):(5*PARA_n + 4*(i-1)*PARA_n + PARA_n)) = MAIN_J;
        mpc_f((PARA_n_EO*i +1):(PARA_n_EO*(i+1)),1) = TASK_ddotx_N(:,i+1) ;
    end
    mpc_E(1:3,7:12) = MAIN_J;
    mpc_f(1:3,1) = mpc_f(1:3,1) - MAIN_dotJ_dotq;
    mpc_Q = mpc_E'*mpc_E;
    mpc_p = -(mpc_E'*mpc_f)';
    
    cons_out = fct_cons();
    
    mpc_qpOptions = optimset('Algorithm','interior-point-convex','Display','off');
    [mpc_khi_N,~,EXITFLAG] = quadprog(mpc_Q,mpc_p,cons_out{3},cons_out{4},cons_out{1},cons_out{2},[],[],[],mpc_qpOptions);
    if EXITFLAG == -2
        MPC_stopComputation = true;
    else
        MPC_khi_app = mpc_khi_N(1:24,1);
    end
end


%%%%%%%
% test call global
% %PARA
% disp(PARA_useReduced);
% 
% disp(PARA_deltat_mpc);
% disp(PARA_epsilon);
% disp(PARA_omegak);
% disp(PARA_N);
% 
% %MAIN
% disp(MAIN_invM);
% disp(MAIN_J);
% disp(MAIN_dotJ);
% 
% disp(MAIN_b);
% disp(MAIN_g);
% disp(MAIN_dotq);
% 
% %TASK
% disp(TASK_ddotx_N);