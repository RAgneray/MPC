
%                       TRAJECTORY_TEST
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%This test script is used to experiment on the element TRAJECTORY
%It is linked to the PARAMETERS_TEST_traj file.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Call the global variables

global PARA_deltat_simu;
global PARA_robot;

%Simulate IN variables
task_k = 0;
task_firstTrajCall = true;
task_posDes = [0.5;0.5;0.5];
task_maxVel = 0.2;
task_q = [0.2;0.3;0.5;0.1;0.7;0.4];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% call global test
disp(PARA_deltat_simu);
disp(PARA_robot);
% pos_k=[];
% vel_k=[];
% acc_k=[];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%test loop | WARNING : does not appear in the actual TRAJECTORY
% for task_k = 0:500
    if task_firstTrajCall
        traj_startPos = PARA_robot.fkine(task_q);
        traj_startPos = traj_startPos(1:3,4);
        %disp(traj_startPos);
        traj_alpha = task_posDes - traj_startPos;
        traj_pointToPointDuration = norm(traj_alpha)/task_maxVel;

        traj_firstTrajCall = false;
        traj_t0 = PARA_deltat_simu * task_k;
        task_firstTrajCall = false;%%%%%%
    end

    traj_usingTrajectory = true;

    if (PARA_deltat_simu*task_k) <= traj_pointToPointDuration
        traj_beta = (PARA_deltat_simu*task_k - traj_t0) / traj_pointToPointDuration ;
        traj_posRef = traj_startPos + traj_alpha * ( 10*(traj_beta^3.0) - 15*(traj_beta^4.0) + 6*(traj_beta^5.0));
        traj_velRef = zeros(3,1) + traj_alpha * ( 30*(traj_beta^2.0) - 60*(traj_beta^3.0) + 30*(traj_beta^4.0));
        traj_accRef = zeros(3,1) + traj_alpha * ( 60*(traj_beta^1.0) - 180*(traj_beta^2.0) + 120*(traj_beta^3.0));
    else
        traj_usingTrajectory = false;
        traj_posRef = task_posDes;
        traj_velRef = zeros(3,1);
        traj_accRef = zeros(3,1);
    end
%     pos_k=[pos_k,traj_posRef];
%     vel_k=[vel_k,traj_velRef];
%     acc_k=[acc_k,traj_accRef];
% end

traj_out = {traj_posRef, traj_velRef, traj_accRef, traj_firstTrajCall, traj_usingTrajectory};