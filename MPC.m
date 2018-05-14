
%                       MPC
% MPC v. 2.2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This script execute the computation of a horizon of control
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Setting global variables
% boolean
global MPC_stopComputation;                     % Used if the optimization solver cannot compute
% vector
global MPC_tau_app;                             % Applied torques (reduced)
global MPC_khi_app;                             % Applied torques and resulting acceleration, velocity and position (detailed)


% Global variables from other scripts
% From PARAMETERS
% boolean
global PARA_useReduced;
% string
global PARA_solverSelect;
% float
global PARA_epsilon;
global PARA_omegak;
% integers
global PARA_n;
global PARA_N;
global PARA_n_EO;

% From MAIN
% matrices
global MAIN_invM;
global MAIN_J;
%global MAIN_dotJ;
% vectors
global MAIN_dotJ_dotq;
global MAIN_b;
global MAIN_g;

% From TASK
% matrix
global TASK_ddotx_desN;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initializing the computation oversight variable
MPC_stopComputation = false;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Reduced formulation

if PARA_useReduced
    
    % Initializing matrix E and vector f
    mpc_E = zeros(PARA_n_EO*(PARA_N + 1), PARA_n * (PARA_N + 1));
    mpc_f = zeros(PARA_n_EO*(PARA_N +1), 1);
    
    % Initializing regularization matrix and vector
    mpc_E_reg = sqrt(PARA_epsilon)*eye(PARA_n * (PARA_N + 1));
    mpc_f_reg = zeros(PARA_n * (PARA_N + 1),1);
    
    % Filling the matrix E and the vector f
    for i = 1:(PARA_N+1)

        mpc_E((PARA_n_EO*(i-1)+1):(PARA_n_EO*i), (PARA_n*(i-1)+1):(i*PARA_n)) = MAIN_J*MAIN_invM;
        mpc_f((PARA_n_EO*(i-1)+1):(PARA_n_EO*i), 1) = TASK_ddotx_desN(:,i); + MAIN_J*MAIN_invM*(MAIN_g+MAIN_b) - MAIN_dotJ_dotq;
        
        mpc_f_reg((PARA_n*(i-1)+1):(PARA_n*i)) = sqrt(PARA_epsilon)*MAIN_g;
    end
    
    % Calling the function computing the constraints
    cons_out = CONSTRAINTS();
    
    % Checking selected solver and compute torque horizon
    % Matlab quadprog solver
    if strcmp(PARA_solverSelect,'quadprog')
        
        % Computing matrix Q and vector p for the quadprog optimization function
        mpc_Q = [sqrt(PARA_omegak)*mpc_E;mpc_E_reg]'*[sqrt(PARA_omegak)*mpc_E;mpc_E_reg];
        mpc_p = -([sqrt(PARA_omegak)*mpc_E;mpc_E_reg]'*[sqrt(PARA_omegak)*mpc_f;mpc_f_reg])';
    
        % Setting up the quadprog parameters
        mpc_qpOptions = optimset('Algorithm','interior-point-convex','Display','off');
    
        % Resolving the minimization problem
        [mpc_tau_N,~,EXITFLAG] = quadprog(mpc_Q,mpc_p,cons_out{1},cons_out{2},[],[],[],[],[],mpc_qpOptions);
        
        % Oversight condition and setting up of the control output (reduced)
        if EXITFLAG == -2
            MPC_stopComputation = true;
        else
            MPC_tau_app = mpc_tau_N(1:PARA_n,1);
        end
    
    % CVX solver
    elseif strcmp(PARA_solverSelect,'cvx')
        % Opening cvx environnement
        cvx_solver sedumi
        cvx_begin quiet
            variable tau((PARA_N+1)*PARA_n);
            minimize (norm([sqrt(PARA_omegak)*mpc_E;mpc_E_reg]* tau - [sqrt(PARA_omegak)*mpc_f;mpc_f_reg]))
            subject to
                cons_out{1} * tau <= cons_out{2};
        cvx_end  
        
        % Oversight condition and setting up of the control output (reduced)
        if strcmp(cvx_status,'Infeasible')
            MPC_stopComputation = true;
        else
            MPC_tau_app = tau(1:PARA_n,1);
        end
     
    %OSQP solver
    elseif strcmp(PARA_solverSelect,'osqp')
        % Creating OSQP object
        mpc_prob = osqp;
        
        % Computing matrix Q and vector p for the OSQP optimization function
        mpc_Q = [sqrt(PARA_omegak)*mpc_E;mpc_E_reg]'*[sqrt(PARA_omegak)*mpc_E;mpc_E_reg];
        mpc_p = -([sqrt(PARA_omegak)*mpc_E;mpc_E_reg]'*[sqrt(PARA_omegak)*mpc_f;mpc_f_reg]);
        
        % Setting up OSQP workplace
        mpc_osqpOptions = mpc_prob.default_settings();
        mpc_osqpOptions.verbose = false;
        
        if isempty(cons_out{1})
            mpc_prob.setup(mpc_Q, mpc_p, eye((size(mpc_Q,1))), -inf(size(mpc_Q,1),1), inf(size(mpc_Q,1),1), 'warm_start', true);
        else
            mpc_prob.setup(mpc_Q, mpc_p, cons_out{1}, -inf(size(cons_out{1},1),1), cons_out{2}, mpc_osqpOptions);
        end
        % Resolving the optimization problem
        mpc_tau_N = mpc_prob.solve();
        
        % Oversight condition and setting up of the control output (reduced)
        if ~strcmp(mpc_tau_N.info.status, 'solved')
            MPC_stopComputation = true;
        else
            MPC_tau_app = mpc_tau_N.x(1:PARA_n,1);
        end
    end
    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Detailed formulation

else
    % Initializing of matrix E and vector f
    mpc_E = zeros(PARA_n_EO*(PARA_N + 1), 2*PARA_n*(2*PARA_N + 1));
    mpc_f = zeros(PARA_n_EO*(PARA_N + 1),1);
    
    % Initializing regularization matrix and vector
    mpc_E_reg = zeros(PARA_n*(PARA_N + 1), 2*PARA_n*(2*PARA_N + 1));
    mpc_f_reg = zeros(PARA_n*(PARA_N + 1),1);
    
    % Filling of matrix E and vector f
    for i = 1:(PARA_N)
        
        mpc_E((PARA_n_EO*i +1):(PARA_n_EO*(i+1)), (5*PARA_n + (i-1)*4*PARA_n +1):(5*PARA_n + 4*(i-1)*PARA_n + PARA_n)) = MAIN_J;
        mpc_f((PARA_n_EO*i +1):(PARA_n_EO*(i+1)),1) = TASK_ddotx_desN(:,i+1)-MAIN_dotJ_dotq;
        
        mpc_E_reg((1+PARA_n*i):((i+1)*PARA_n),(1+4*i*PARA_n):((4*i+1)*PARA_n)) = sqrt(PARA_epsilon)*eye(PARA_n);
        mpc_f_reg((1+PARA_n*i):((i+1)*PARA_n),1) = sqrt(PARA_epsilon)*MAIN_g;
    end
    
    mpc_E(1:PARA_n_EO,(PARA_n+1):(2*PARA_n)) = MAIN_J;
    mpc_f(1:PARA_n_EO,1) = mpc_f(1:PARA_n_EO,1);
    
    mpc_E_reg(1:PARA_n,1:PARA_n) = sqrt(PARA_epsilon)*eye(PARA_n);
    mpc_f_reg(1:PARA_n,1) = sqrt(PARA_epsilon)*MAIN_g;
    
    % Calling the function computing the constraints 
    cons_out = CONSTRAINTS();
    
    % Checking selected solver and compute torque horizon
    % Matlab quadprog solver
    if strcmp(PARA_solverSelect,'quadprog')
    
        % Computing matrix Q and vector p for the quadprog optimization function
        mpc_Q = [sqrt(PARA_omegak)*mpc_E;mpc_E_reg]'*[sqrt(PARA_omegak)*mpc_E;mpc_E_reg];
        mpc_p = -([sqrt(PARA_omegak)*mpc_E;mpc_E_reg]'*[sqrt(PARA_omegak)*mpc_f;mpc_f_reg])';

        % Calling the function computing the constraints
        mpc_qpOptions = optimset('Algorithm','interior-point-convex','Display','off');

        % Resolving the minimization problem
        [mpc_khi_N,~,EXITFLAG] = quadprog(mpc_Q,mpc_p,cons_out{3},cons_out{4},cons_out{1},cons_out{2},[],[],[],mpc_qpOptions);

        % Oversight condition and setting up of the control output (detailed)
        if EXITFLAG == -2
            MPC_stopComputation = true;
        else
            MPC_khi_app = mpc_khi_N(1:(4*PARA_n),1);
        end
        
    % CVX solver
    elseif strcmp(PARA_solverSelect,'cvx')
        % Opening cvx environnement
        cvx_solver sedumi
        cvx_begin quiet
            variable khi(2*PARA_n*(2*PARA_N + 1));
            minimize (norm([sqrt(PARA_omegak)*mpc_E;mpc_E_reg]* khi - [sqrt(PARA_omegak)*mpc_f;mpc_f_reg]))
            subject to
                cons_out{1} * khi == cons_out{2};
                cons_out{3} * khi <= cons_out{4};
        cvx_end  
        
        % Oversight condition and setting up of the control output (reduced)
        if strcmp(cvx_status,'Infeasible')
            MPC_stopComputation = true;
        else
            MPC_khi_app = khi(1:(4*PARA_n),1);
        end
          
    end
    
end
    
