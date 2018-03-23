
%                       UPDATEQ_TEST
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This script is a test for the element UPDATEQ
% It is linked to the PARAMETERS_upda file

%call global

%PARA
global PARA_useReduced;

global PARA_deltat_simu;

%MAIN
global MAIN_b;
global MAIN_g;
global MAIN_q;
global MAIN_dotq;

global MAIN_invM;

%MPC 
global MPC_tau_app;
global MPC_khi_app;

%%%%%%%%%
if PARA_useReduced
    upda_dotq = MAIN_dotq + PARA_deltat_simu*(MAIN_invM * (MPC_tau_app - MAIN_b - MAIN_g));
    upda_q = MAIN_q + PARA_deltat_simu*MAIN_dotq + 0.5*(PARA_deltat_simu^2.0)*(MAIN_invM * (MPC_tau_app - MAIN_b - MAIN_g));
else
    upda_dotq = MPC_khi_app(13:18,1);
    upda_q = MPC_khi_app(19:24,1);
end

upda_out={upda_dotq,upda_q};

disp( PARA_useReduced);

disp( PARA_deltat_simu);

%MAIN
disp( MAIN_b);
disp( MAIN_g);
disp( MAIN_q);
disp( MAIN_dotq);

disp( MAIN_invM);

%MPC 
disp( MPC_tau_app);
disp( MPC_khi_app);