
%                       CONSTRAINTS
% MPC v. 2.2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function is used for the computation of the constraints matrices and
% vector needed by the optimization problem

function [ cons_out ] = CONSTRAINTS( )


% Call global variables from other scripts
% From PARAMETERS
% boolean
global PARA_useReduced;
global PARA_useTorLimMin;     
global PARA_useTorLimMax;      
global PARA_usePosLimMin;     
global PARA_usePosLimMax;      
% float
global PARA_deltat_mpc;
% integers
global PARA_N;
global PARA_n;
% vector
global PARA_q_min;
global PARA_q_max;
global PARA_tau_min;
global PARA_tau_max;

% From MAIN
% matrices
global MAIN_M;
global MAIN_invM;
% vectors
global MAIN_b;
global MAIN_g;
global MAIN_dotq;
global MAIN_q;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Checking how many constraints are enabled

cons_checkCons = 0;

if PARA_useTorLimMin
    cons_checkCons = cons_checkCons +1;
end
if PARA_useTorLimMax
    cons_checkCons = cons_checkCons +1;
end
if PARA_usePosLimMin
    cons_checkCons = cons_checkCons +1;
end
if PARA_usePosLimMax
    cons_checkCons = cons_checkCons +1;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Using the reduced form

    if PARA_useReduced

        % Initializing 
        cons_G = zeros(cons_checkCons * PARA_n * (PARA_N + 1), PARA_n * (PARA_N + 1));   % Inequalities constraints matrix
        cons_h = zeros(cons_checkCons * PARA_n * (PARA_N + 1), 1);                       % Inequalities constraints vector
        cons_coeff_h = 0;                                                                % Iterative variables used by cons_h

        % Building loop
        for i = 0 : PARA_N

            % Filling cons_h
            cons_fillh = 0;
            if PARA_useTorLimMin
                cons_h((cons_checkCons*i*PARA_n + 1) : (cons_checkCons*i*PARA_n + PARA_n) , 1) = -PARA_tau_min;
                cons_fillh = cons_fillh +1;
            end
            if PARA_useTorLimMax
                cons_h(((cons_checkCons*i + cons_fillh)*PARA_n + 1) : ((cons_checkCons*i + cons_fillh)*PARA_n + PARA_n) , 1) = PARA_tau_max;
                cons_fillh = cons_fillh +1;
            end
            if PARA_usePosLimMin
                cons_h(((cons_checkCons*i + cons_fillh)*PARA_n + 1) : ((cons_checkCons*i + cons_fillh)*PARA_n + PARA_n) , 1) = -2/(PARA_deltat_mpc*PARA_deltat_mpc)*(PARA_q_min - MAIN_q - (i+1)*PARA_deltat_mpc*MAIN_dotq + cons_coeff_h*(PARA_deltat_mpc*PARA_deltat_mpc)*MAIN_invM*(MAIN_b+MAIN_g));
                cons_fillh = cons_fillh +1;
            end
            if PARA_usePosLimMax
                cons_h(((cons_checkCons*i + cons_fillh)*PARA_n + 1) : ((cons_checkCons*i + cons_fillh)*PARA_n + PARA_n) , 1) = 2/(PARA_deltat_mpc*PARA_deltat_mpc)*(PARA_q_max - MAIN_q - (i+1)*PARA_deltat_mpc*MAIN_dotq + cons_coeff_h*(PARA_deltat_mpc*PARA_deltat_mpc)*MAIN_invM*(MAIN_b+MAIN_g));
            end
        
            % Updating cons_coeff_h
            cons_coeff_h = (i+1) + 0.5 + cons_coeff_h;


            % Filling cons_G
            cons_fillG = 0;
            if PARA_useTorLimMin
                cons_G((1 + cons_checkCons*i*PARA_n) : (PARA_n + cons_checkCons*i*PARA_n) , (PARA_n*i +1) : (PARA_n*(i+1))) = -eye(PARA_n);
                cons_fillG = cons_fillG +1;
            end
            if PARA_useTorLimMax
                cons_G(( 1 + (cons_checkCons*i + cons_fillG)*PARA_n) : (PARA_n + (cons_checkCons*i + cons_fillG)*PARA_n) , (PARA_n*i +1) : (PARA_n*(i+1))) = eye(PARA_n);
                cons_fillG = cons_fillG +1;
            end
            if PARA_usePosLimMin
                cons_G((1 + (cons_checkCons*i + cons_fillG)*PARA_n) : (PARA_n + (cons_checkCons*i + cons_fillG)*PARA_n) , (PARA_n*i +1) : (PARA_n*(i+1))) = -MAIN_invM;
                if i~=0
                    for j = 0 : (i-1)
                        cons_G((1 + (cons_checkCons*i + cons_fillG)*PARA_n) : (PARA_n + (cons_checkCons*i + cons_fillG)*PARA_n) , (PARA_n*j + 1) : (PARA_n*(j+1))) = -(3+i-j-1)*MAIN_invM;
                    end
                end
                cons_fillG = cons_fillG +1;
            end
            if PARA_usePosLimMax
                cons_G((1 + (cons_checkCons*i + cons_fillG)*PARA_n) : (PARA_n + (cons_checkCons*i + cons_fillG)*PARA_n) , (PARA_n*i +1) : (PARA_n*(i+1))) = MAIN_invM;
                if i~=0
                    for j = 0 : (i-1)
                        cons_G((1 + (cons_checkCons*i + cons_fillG)*PARA_n) : (PARA_n + (cons_checkCons*i + cons_fillG)*PARA_n) , (PARA_n*j + 1) : (PARA_n*(j+1))) = (3+i-j-1)*MAIN_invM;
                    end
                end
            end
        end

        %Setting up the output variable (reduced)
        cons_out = {cons_G , cons_h};



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    % Using the detailed form
    else
        % Initializing
        cons_A = zeros(3 * PARA_n * (PARA_N + 1) , 2 * PARA_n * (2 * PARA_N + 1));      % Equality constraints matrix
        cons_G = zeros(cons_checkCons * PARA_n * (PARA_N + 1), PARA_n * (PARA_N + 1));
        cons_c = zeros(3 * PARA_n * (PARA_N + 1) , 1);                                  % Equality constraints vector
        cons_h = zeros(cons_checkCons * PARA_n * (PARA_N + 1), 1);

        % Creating submatrices for filling constraints matrices
        cons_SubA_begin = [zeros(PARA_n),-PARA_deltat_mpc*eye(PARA_n),eye(PARA_n),zeros(PARA_n);zeros(PARA_n),-0.5*PARA_deltat_mpc*PARA_deltat_mpc*eye(PARA_n),zeros(PARA_n),eye(PARA_n);eye(PARA_n),-MAIN_M,zeros(PARA_n),zeros(PARA_n)];
        cons_SubA_inter = [-eye(PARA_n),zeros(PARA_n),zeros(PARA_n),-PARA_deltat_mpc*eye(PARA_n),eye(PARA_n),zeros(PARA_n);-PARA_deltat_mpc*eye(PARA_n),-eye(PARA_n),zeros(PARA_n),-0.5*PARA_deltat_mpc*PARA_deltat_mpc*eye(PARA_n),zeros(PARA_n),eye(PARA_n);zeros(PARA_n),zeros(PARA_n),eye(PARA_n),-MAIN_M,zeros(PARA_n),zeros(PARA_n)];
        cons_SubA_end = [zeros(PARA_n),zeros(PARA_n);zeros(PARA_n),zeros(PARA_n);eye(PARA_n),-MAIN_M];

        cons_SubG_begin =[];
        cons_SubG_inter =[];
        if PARA_useTorLimMin
            cons_SubG_begin = [-eye(PARA_n),zeros(PARA_n)];
            cons_SubG_inter = [zeros(PARA_n),zeros(PARA_n),-eye(PARA_n),zeros(PARA_n)];
        end
        if PARA_useTorLimMax
            cons_SubG_begin = [cons_SubG_begin; eye(PARA_n),zeros(PARA_n)];
            cons_SubG_inter = [cons_SubG_inter;zeros(PARA_n),zeros(PARA_n),eye(PARA_n),zeros(PARA_n)];
        end
        if PARA_usePosLimMin
            cons_SubG_begin = [cons_SubG_begin; zeros(PARA_n),-eye(PARA_n)];
            cons_SubG_inter = [cons_SubG_inter;-2/PARA_deltat_mpc*eye(PARA_n),-2/(PARA_deltat_mpc*PARA_deltat_mpc)*eye(PARA_n),zeros(PARA_n),-eye(PARA_n)];
        end
        if PARA_usePosLimMax
            cons_SubG_begin = [cons_SubG_begin; zeros(PARA_n),eye(PARA_n)];
            cons_SubG_inter = [cons_SubG_inter;2/PARA_deltat_mpc*eye(PARA_n),2/(PARA_deltat_mpc*PARA_deltat_mpc)*eye(PARA_n),zeros(PARA_n),eye(PARA_n)];
        end

        % Filling matrices and vectors
        for i = 1 : (PARA_N - 1)
            cons_A((i*3*PARA_n +1):((i*3+3)*PARA_n),(2*PARA_n+4*(i-1)*PARA_n + 1):(2*PARA_n+4*(i-1)*PARA_n+ 6*PARA_n)) = cons_SubA_inter;
            cons_G((size(cons_SubG_inter,1)*i +1):(size(cons_SubG_inter,1)*(i+1)),(size(cons_SubG_begin,2) + size(cons_SubG_inter,2)*(i-1)+ 1):(size(cons_SubG_begin,2) + size(cons_SubG_inter,2)*i)) = cons_SubG_inter;
            cons_c((2*PARA_n + 3*PARA_n*(i) +1):(2*PARA_n + 3*PARA_n*(i) + PARA_n),1) = MAIN_b + MAIN_g; 
            
            cons_fillh = 0;
            if PARA_useTorLimMin
                cons_h((cons_checkCons*i*PARA_n + 1) : (cons_checkCons*i*PARA_n + PARA_n) , 1) = -PARA_tau_min;
                cons_fillh = cons_fillh +1;
            end
            if PARA_useTorLimMax
                cons_h(((cons_checkCons*i + cons_fillh)*PARA_n + 1) : ((cons_checkCons*i + cons_fillh)*PARA_n + PARA_n) , 1) = PARA_tau_max;
                cons_fillh = cons_fillh +1;
            end
            if PARA_usePosLimMin
                cons_h(((cons_checkCons*i + cons_fillh)*PARA_n + 1) : ((cons_checkCons*i + cons_fillh)*PARA_n + PARA_n) , 1) = -2/(PARA_deltat_mpc*PARA_deltat_mpc)*PARA_q_min;
                cons_fillh = cons_fillh +1;
            end
            if PARA_usePosLimMax
                cons_h(((cons_checkCons*i + cons_fillh)*PARA_n + 1) : ((cons_checkCons*i + cons_fillh)*PARA_n + PARA_n) , 1) = 2/(PARA_deltat_mpc*PARA_deltat_mpc)*PARA_q_max;
            end
        end

        cons_A(1:(3*PARA_n),1:(4*PARA_n)) = cons_SubA_begin;
        cons_A((3*PARA_N*PARA_n + 1):(3*(PARA_N + 1)*PARA_n),(2*PARA_n*(2*PARA_N )+1):(2*PARA_n*(2*PARA_N +1))) = cons_SubA_end;

        if cons_checkCons ~= 0
            cons_G(1:(size(cons_SubG_begin,1)),1:(size(cons_SubG_begin,2))) = cons_SubG_begin;
            cons_G((size(cons_SubG_inter,1)*PARA_N +1):(size(cons_SubG_inter,1)*(PARA_N+1)),(2*PARA_n + 4*(PARA_N-1)*PARA_n + 1):(2*PARA_n + 4*PARA_N*PARA_n)) = cons_SubG_inter;
        end

        cons_c(1:(3*PARA_n),1) = [MAIN_dotq ; MAIN_q + PARA_deltat_mpc*MAIN_dotq ; MAIN_b+MAIN_g];
        cons_c((2*PARA_n + 1 + 3*PARA_n*PARA_N):(3*PARA_n*(PARA_N+1)),1) = MAIN_b + MAIN_g;
        
        cons_fillh = 0;
        if PARA_useTorLimMin
                cons_h(1:(PARA_n),1) = -PARA_tau_min;
                cons_h((cons_checkCons*(PARA_N)* PARA_n + 1) : (cons_checkCons*(PARA_N)* PARA_n + PARA_n) , 1) = -PARA_tau_min;
                cons_fillh = cons_fillh +1;
        end
        if PARA_useTorLimMax
                cons_h((cons_fillh* PARA_n + 1) : (cons_fillh* PARA_n + PARA_n),1) = PARA_tau_max;
                cons_h(((cons_checkCons*(PARA_N)+ cons_fillh)*PARA_n + 1) : ((cons_checkCons*(PARA_N)+ cons_fillh)*PARA_n + PARA_n) , 1) = PARA_tau_max;
                cons_fillh = cons_fillh +1;
        end
        if PARA_usePosLimMin
                cons_h((cons_fillh* PARA_n + 1) : (cons_fillh* PARA_n + PARA_n),1) = -2/(PARA_deltat_mpc*PARA_deltat_mpc)*(PARA_q_min-MAIN_q-PARA_deltat_mpc*MAIN_dotq);
                cons_h(((cons_checkCons*(PARA_N)+ cons_fillh)*PARA_n + 1) : ((cons_checkCons*(PARA_N)+ cons_fillh)*PARA_n + PARA_n) , 1) = -2/(PARA_deltat_mpc*PARA_deltat_mpc)*PARA_q_min;
                cons_fillh = cons_fillh +1;
        end
        if PARA_usePosLimMax
                cons_h((cons_fillh* PARA_n + 1) : (cons_fillh* PARA_n + PARA_n),1) = 2/(PARA_deltat_mpc*PARA_deltat_mpc)*(PARA_q_max-MAIN_q-PARA_deltat_mpc*MAIN_dotq);
                cons_h(((cons_checkCons*(PARA_N)+ cons_fillh)*PARA_n + 1) : ((cons_checkCons*(PARA_N)+ cons_fillh)*PARA_n + PARA_n) , 1) = 2/(PARA_deltat_mpc*PARA_deltat_mpc)*PARA_q_max;
        end
            
        % Setting up the output variables (detailed)
        cons_out={cons_A,cons_c,cons_G,cons_h};
    end
end
