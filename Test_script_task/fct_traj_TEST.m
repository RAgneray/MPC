function [ traj_out ] = fct_traj_TEST( task_k, task_firstTrajCall, task_posDes, task_maxVel, task_q)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

global PARA_deltat_simu;
global PARA_robot;

global traj_pointToPointDuration;
global traj_alpha;
global traj_t0;
global traj_startPos;
global traj_firstTrajCall;

if task_firstTrajCall
    
    traj_startPos = PARA_robot.fkine(task_q);
    traj_startPos = traj_startPos(1:3,4);
    %disp(traj_startPos);
    traj_alpha = task_posDes - traj_startPos;
    traj_pointToPointDuration = norm(traj_alpha)/task_maxVel;
    
    traj_firstTrajCall = false;
    traj_t0 = PARA_deltat_simu * task_k;
    
    %disp(traj_pointToPointDuration);
    %task_firstTrajCall = false;%%%%%%
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

traj_out = {traj_posRef, traj_velRef, traj_accRef, traj_firstTrajCall, traj_usingTrajectory};

end

