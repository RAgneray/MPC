
%                   CONSTRAINTS_TEST
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%This script contains is a test one for the element CONSTRAINTS. It is
%linked to PARAMETERS_TEST.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Call global variables 
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Use the reduced form

if PARA_useReduced
    
    %Initialization
    cons_G = zeros(4 * PARA_n * (PARA_N + 1), PARA_n * (PARA_N + 1));
    cons_h = zeros(4 * PARA_n * (PARA_N + 1), 1);
    cons_coeff_h = 0;
    
    %Building loop
    for i = 0 : PARA_N
        %Build cons_h
        cons_h((4*i*PARA_n + 1) : (4*i*PARA_n + PARA_n) , 1) = -PARA_tau_min;
        cons_h(((4*i + 1)*PARA_n + 1) : ((4*i + 1)*PARA_n + PARA_n) , 1) = PARA_tau_max;
        cons_h(((4*i + 2)*PARA_n + 1) : ((4*i + 2)*PARA_n + PARA_n) , 1) = -2/(PARA_deltat_mpc*PARA_deltat_mpc)*(PARA_q_min - MAIN_q - (i+1)*PARA_deltat_mpc*MAIN_dotq + cons_coeff_h*(PARA_deltat_mpc*PARA_deltat_mpc)*MAIN_invM*(MAIN_b+MAIN_g));
        cons_h(((4*i + 3)*PARA_n + 1) : ((4*i + 3)*PARA_n + PARA_n) , 1) = 2/(PARA_deltat_mpc*PARA_deltat_mpc)*(PARA_q_max - MAIN_q - (i+1)*PARA_deltat_mpc*MAIN_dotq + cons_coeff_h*(PARA_deltat_mpc*PARA_deltat_mpc)*MAIN_invM*(MAIN_b+MAIN_g));
        
        cons_coeff_h = (i+1) + 0.5 + cons_coeff_h;
        
        %Build cons_G
        for j = 0 : i
            cons_G((2*PARA_n + 1 + 4*i*PARA_n) : (2*PARA_n + PARA_n + 4*i*PARA_n) , (PARA_n*j + 1) : (PARA_n*(j+1))) = -(3+i-j-1)*MAIN_invM;
            cons_G((3*PARA_n + 1 + 4*i*PARA_n) : (3*PARA_n + PARA_n + 4*i*PARA_n) , (PARA_n*j + 1) : (PARA_n*(j+1))) = (3+i-j-1)*MAIN_invM;
        end
        cons_G((2*PARA_n + 1 + 4*i*PARA_n) : (2*PARA_n + PARA_n + 4*i*PARA_n) , (PARA_n*i +1) : (PARA_n*(i+1))) = -MAIN_invM;
        cons_G((3*PARA_n + 1 + 4*i*PARA_n) : (3*PARA_n + PARA_n + 4*i*PARA_n) , (PARA_n*i +1) : (PARA_n*(i+1))) = MAIN_invM;
        cons_G((1 + 4*i*PARA_n) : (PARA_n + 4*i*PARA_n) , (PARA_n*i +1) : (PARA_n*(i+1))) = -eye(PARA_n);
        cons_G((PARA_n + 1 + 4*i*PARA_n) : (PARA_n + PARA_n + 4*i*PARA_n) , (PARA_n*i +1) : (PARA_n*(i+1))) = eye(PARA_n);
    end
    cons_out = {cons_G , cons_h};
    
    
%Use the detailed form
else
    %Initialization
    cons_A = zeros(3 * PARA_n * (PARA_N + 1) , 2 * PARA_n * (2 * PARA_N + 1));
    cons_G = zeros(4 * PARA_n * (PARA_N + 1) , 2 * PARA_n * (2 * PARA_N + 1));
    cons_c = zeros(3 * PARA_n * (PARA_N + 1) , 1);
    cons_h = zeros(4 * PARA_n * (PARA_N + 1) , 1);
    
    %Create submatrices
    cons_SubA_begin = [zeros(PARA_n),-PARA_deltat_mpc*eye(PARA_n),eye(PARA_n),zeros(PARA_n);zeros(PARA_n),-0.5*PARA_deltat_mpc*PARA_deltat_mpc*eye(PARA_n),zeros(PARA_n),eye(PARA_n);eye(PARA_n),-MAIN_M,zeros(PARA_n),zeros(PARA_n)];
    cons_SubA_inter = [-eye(PARA_n),zeros(PARA_n),zeros(PARA_n),-PARA_deltat_mpc*eye(PARA_n),eye(PARA_n),zeros(PARA_n);-PARA_deltat_mpc*eye(PARA_n),-eye(PARA_n),zeros(PARA_n),-0.5*PARA_deltat_mpc*PARA_deltat_mpc*eye(PARA_n),zeros(PARA_n),eye(PARA_n);zeros(PARA_n),zeros(PARA_n),eye(PARA_n),-MAIN_M,zeros(PARA_n),zeros(PARA_n)];
    cons_SubA_end = [zeros(PARA_n),zeros(PARA_n);zeros(PARA_n),zeros(PARA_n);eye(PARA_n),-MAIN_M];
    
    cons_SubG_begin = [-eye(PARA_n),zeros(PARA_n);eye(PARA_n),zeros(PARA_n);zeros(PARA_n),-eye(PARA_n);zeros(PARA_n),eye(PARA_n)];
    cons_SubG_inter = [zeros(PARA_n),zeros(PARA_n),-eye(PARA_n),zeros(PARA_n);zeros(PARA_n),zeros(PARA_n),eye(PARA_n),zeros(PARA_n);-2/PARA_deltat_mpc*eye(PARA_n),-2/(PARA_deltat_mpc*PARA_deltat_mpc)*eye(PARA_n),zeros(PARA_n),-eye(PARA_n);2/PARA_deltat_mpc*eye(PARA_n),2/(PARA_deltat_mpc*PARA_deltat_mpc)*eye(PARA_n),zeros(PARA_n),eye(PARA_n)];
    
    %Build matrices and vectors
    for i = 1 : (PARA_N - 1)
        cons_A((i*3*PARA_n +1):((i*3+3)*PARA_n),(2*PARA_n+4*(i-1)*PARA_n + 1):(2*PARA_n+4*(i-1)*PARA_n+ 6*PARA_n)) = cons_SubA_inter;
        cons_G((4*PARA_n*i +1):(4*PARA_n*(i+1)),(2*PARA_n + 4*(i-1)*PARA_n + 1):(2*PARA_n + 4*i*PARA_n)) = cons_SubG_inter;
        cons_c((2*PARA_n + 3*PARA_n*(i) +1):(2*PARA_n + 3*PARA_n*(i) + PARA_n),1) = MAIN_b + MAIN_g; 
        cons_h((4*PARA_n*i +1):(4*PARA_n*(i+1)),1) = [-PARA_tau_min;PARA_tau_max;-2/(PARA_deltat_mpc*PARA_deltat_mpc)*PARA_q_min;2/(PARA_deltat_mpc*PARA_deltat_mpc)*PARA_q_max];
    end
    
    cons_A(1:(3*PARA_n),1:(4*PARA_n)) = cons_SubA_begin;
    cons_A((3*PARA_N*PARA_n + 1):(3*(PARA_N + 1)*PARA_n),(2*PARA_n*(2*PARA_N )+1):(2*PARA_n*(2*PARA_N +1))) = cons_SubA_end;
    
    cons_G(1:(4*PARA_n),1:(2*PARA_n)) = cons_SubG_begin;
    cons_G((4*PARA_n*PARA_N +1):(4*PARA_n*(PARA_N+1)),(2*PARA_n + 4*(PARA_N-1)*PARA_n + 1):(2*PARA_n + 4*PARA_N*PARA_n)) = cons_SubG_inter;
    
    cons_c(1:(3*PARA_n),1) = [MAIN_dotq ; MAIN_q + PARA_deltat_mpc*MAIN_dotq ; MAIN_b+MAIN_g];
    cons_c((2*PARA_n + 1 + 3*PARA_n*PARA_N):(3*PARA_n*(PARA_N+1)),1) = MAIN_b + MAIN_g;
    
    cons_h(1:(4*PARA_n),1) = [-PARA_tau_min;PARA_tau_max;-2/(PARA_deltat_mpc*PARA_deltat_mpc)*(PARA_q_min-MAIN_q-PARA_deltat_mpc*MAIN_dotq);2/(PARA_deltat_mpc*PARA_deltat_mpc)*(PARA_q_max-MAIN_q-PARA_deltat_mpc*MAIN_dotq)];
    cons_h((4*PARA_n*PARA_N +1):(4*PARA_n*(PARA_N+1)),1) =  [-PARA_tau_min;PARA_tau_max;-2/(PARA_deltat_mpc*PARA_deltat_mpc)*PARA_q_min;2/(PARA_deltat_mpc*PARA_deltat_mpc)*PARA_q_max];
    
    cons_out={cons_A,cons_c,cons_G,cons_h};
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %test the call of global variables
% 
% disp('PARA_useReduced :');
% disp(PARA_useReduced);
% 
% disp(strcat('PARA_deltat_mpc :',num2str(PARA_deltat_mpc)));
% 
% disp(strcat('PARA_N :',num2str(PARA_N)));
% disp(strcat('PARA_n :',num2str(PARA_n)));
% 
% disp('PARA_q_min :'); disp(PARA_q_min);
% disp('PARA_q_max :'); disp(PARA_q_max);
% disp('PARA_tau_min :'); disp(PARA_tau_min);
% disp('PARA_tau_max :'); disp(PARA_tau_max);
% 
% disp('Main_M :'); disp(MAIN_M);
% disp('Main_invM :'); disp(MAIN_invM);
% 
% disp('MAIN_b :'); disp(MAIN_b);
% disp('MAIN_g :'); disp(MAIN_g);
% disp('MAIN_dotq :'); disp(MAIN_dotq);
% disp('MAIN_q :'); disp(MAIN_q);