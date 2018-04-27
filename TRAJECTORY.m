
%                   TRAJECTORY
% MPC v 2.1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This script is a function able to generate acceleration, velocity and
% positions references for a minimal-jerk trajectory
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [traj_out] = TRAJECTORY(task_posDes)

%Call the global variables
global PARA_n;
global PARA_n_EO;
global PARA_N;
global PARA_deltat_simu;
global PARA_tend;
global PARA_q_0;
global PARA_robot;
global PARA_maxVel;
global PARA_maxAcc;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Computing parameters for trajectory

if PARA_n == 1                                                                % inverse kinematic model (ikine not adapted to less than 6 DOF robot)  
    traj_qDes = atan2(task_posDes(2,1),task_posDes(1,1));    
elseif PARA_n == 2
    traj_qDes(2,1) = acos(0.5*(task_posDes(1,1)^2 + task_posDes(3,1)^2) - 1);
    traj_qDes(1,1) = atan((task_posDes(3,1)*(1+cos(traj_qDes(2,1))) - task_posDes(1,1)*sin(traj_qDes(2,1)))/(task_posDes(1,1)*(1+cos(traj_qDes(2,1))) + task_posDes(3,1)*sin(traj_qDes(2,1))));
elseif PARA_n == 6
    traj_qDes = PARA_robot.ikine([rpy2r(task_posDes(4:6)'),task_posDes(1:3);[0,0,0,1]]);
    traj_qDes = traj_qDes';                                                   % Joint position corresponding to desired operational position
end


traj_T_v = NaN;                                                               % Minimal trajectory time given velocity limit
traj_T_a = NaN;                                                               % Minimal trajectory time given acceleration limit

if ~isnan(PARA_maxVel)
    traj_T_v = 0;
    for l = 1:PARA_n
        traj_tmp =2*abs(traj_qDes(l,1)-PARA_q_0(l,1))/PARA_maxVel;
        if traj_tmp > traj_T_v
            traj_T_v = traj_tmp;
        end
    end
end

if ~isnan(PARA_maxAcc)
    traj_T_a = 0;
    for l = 1:PARA_n
        traj_tmp =sqrt(8*abs(traj_qDes(l,1)-PARA_q_0(l,1))/PARA_maxAcc);
        if traj_tmp > traj_T_a
            traj_T_a = traj_tmp;
        end
    end
end

traj_T = max(traj_T_a,traj_T_v) + 0.001;                                      % Trajectory duration
traj_alpha = 32*(traj_qDes-PARA_q_0)/(traj_T^3);                              % Min Max Jerk alpha parameter


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initializing 
traj_q_ref =zeros(PARA_n,PARA_tend/PARA_deltat_simu+PARA_N+1);
traj_dotq_ref =zeros(PARA_n,PARA_tend/PARA_deltat_simu+PARA_N+1);
traj_ddotq_ref =zeros(PARA_n,PARA_tend/PARA_deltat_simu+PARA_N+1);

traj_q_ref(:,1) = PARA_q_0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generating the Min Max Jerk trajectory

for k = 1:((PARA_tend/PARA_deltat_simu)+PARA_N)
    if k <= ((traj_T/PARA_deltat_simu)/4)
        traj_ddotq_ref(:,k+1) = traj_alpha*(k*PARA_deltat_simu);
        traj_dotq_ref(:,k+1) = 0.5*traj_alpha*((k*PARA_deltat_simu)^2);
        traj_q_ref(:,k+1) = 1/6*traj_alpha*((k*PARA_deltat_simu)^3)+PARA_q_0;
    elseif k >= (3*(traj_T/PARA_deltat_simu)/4)
        if k <= (traj_T/PARA_deltat_simu)
            traj_ddotq_ref(:,k+1) = traj_alpha*(k*PARA_deltat_simu)-traj_alpha*traj_T;
            traj_dotq_ref(:,k+1) = 0.5*traj_alpha*((k*PARA_deltat_simu)^2) - traj_alpha*traj_T*(k*PARA_deltat_simu)+traj_alpha*(traj_T^2)/2;
            traj_q_ref(:,k+1) = 1/6*traj_alpha*((k*PARA_deltat_simu)^3) - 0.5*traj_alpha*traj_T*((k*PARA_deltat_simu)^2) + 0.5*traj_alpha*(traj_T^2)*(k*PARA_deltat_simu) - 13*traj_alpha*(traj_T^3)/96  +PARA_q_0;
        end
    else
        traj_ddotq_ref(:,k+1) = - traj_alpha*(k*PARA_deltat_simu) + 0.5*traj_alpha*traj_T;
        traj_dotq_ref(:,k+1)  = - 0.5*traj_alpha*((k*PARA_deltat_simu)^2) + 0.5*traj_alpha*traj_T*(k*PARA_deltat_simu) - traj_alpha*traj_T^2/16;
        traj_q_ref(:,k+1)     = - 1/6*traj_alpha*((k*PARA_deltat_simu)^3) + 1/4*traj_alpha*traj_T*((k*PARA_deltat_simu)^2) - 1/16*traj_alpha*(traj_T^2)*(k*PARA_deltat_simu) + 1/192*traj_alpha*(traj_T^3) +PARA_q_0;
    end
end

if floor((traj_T/PARA_deltat_simu))<= (PARA_tend/PARA_deltat_simu+PARA_N)
    for k = (floor((traj_T/PARA_deltat_simu))+1):(PARA_tend/PARA_deltat_simu+PARA_N)
        traj_q_ref(:,k+1) = traj_qDes;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Computing operational reference acceleration, velocity and position

traj_x_ref = zeros(PARA_n_EO,size(traj_q_ref,2));
traj_dotx_ref = zeros(PARA_n_EO,size(traj_q_ref,2));
traj_ddotx_ref = zeros(PARA_n_EO,size(traj_q_ref,2));

for l=1:size(traj_q_ref,2)
    traj_tmp = PARA_robot.fkine(traj_q_ref(:,l));
    
    if PARA_n_EO > 3
        traj_x_ref(1:3,l) = traj_tmp(1:3,4);
        traj_x_ref(4:6,l) = tr2rpy(traj_tmp(1:3,1:3))';
    else
        traj_x_ref(:,l) = traj_tmp(1:PARA_n_EO,4);  
    end
    
    traj_tmp = PARA_robot.jacob0(traj_q_ref(:,l))*traj_dotq_ref(:,l);
    traj_dotx_ref(:,l) = traj_tmp(1:PARA_n_EO,1);

    traj_tmp = PARA_robot.jacob0(traj_q_ref(:,l))*traj_ddotq_ref(:,l)+PARA_robot.jacob_dot(traj_q_ref(:,l),traj_dotq_ref(:,l));
    traj_ddotx_ref(1:PARA_n_EO,l) = traj_tmp(1:PARA_n_EO,1);

    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Building output 

traj_out = {traj_x_ref,traj_dotx_ref,traj_ddotx_ref,false,traj_q_ref,traj_dotq_ref,traj_ddotq_ref};

end