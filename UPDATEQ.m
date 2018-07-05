
%                      UPDATEQ
% MPC v. 2.3
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This element is used for the computation of q and dot_q at each iteration
% of the simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [upda_out] = UPDATEQ()

% Calling global variables from other scripts
% From PARA
% boolean
global PARA_useReduced;
% integer
global PARA_n;
% float
global PARA_deltat_simu;

% From MAIN
% vectors
global MAIN_b;
global MAIN_g;
global MAIN_q;
global MAIN_dotq;
% matrix
global MAIN_invM;

% From MPC 
% vectors
global MPC_tau_app;
global MPC_khi_app;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Using Reduced form
if PARA_useReduced
    upda_dotq = MAIN_dotq + PARA_deltat_simu*(MAIN_invM * (MPC_tau_app - MAIN_b - MAIN_g));
    upda_q = MAIN_q + PARA_deltat_simu*MAIN_dotq + 0.5*(PARA_deltat_simu^2.0)*(MAIN_invM * (MPC_tau_app - MAIN_b - MAIN_g));
else

% Using detailed form
% WARNING : detailed form have been implanted but does not work properly yet. The obtained results using this method are bad control sequences most of the time 
% Moreover, this form have been given up during the coding process because of its computing cost.

    upda_dotq = MPC_khi_app((2*PARA_n+1):(3*PARA_n),1);           % Joint velocities for the next iteration
    upda_q = MPC_khi_app((3*PARA_n+1):(4*PARA_n),1);              % Joint positions for the next iteration
end

% Setting up the output variable
upda_out = {upda_dotq, upda_q};

end
