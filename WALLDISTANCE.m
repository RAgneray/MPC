
%                       DISTANCE TO WALL
% MPC v. 2.3
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function is used for the computation of the distance between
% the end effector and a wall (used as a operational constraints)


function [ distToWall] = WALLDISTANCE( )

% Call global variables from other scripts
% From PARAMETERS
% vectors
global PARA_normalVect;   
global PARA_wallPoint;
% SerialLink
global PARA_robot;

% From MAIN
% vectors
global MAIN_q;



tmp = PARA_robot.fkine(MAIN_q);
distToWall = (tmp(1:3,4)-PARA_wallPoint)'*PARA_normalVect/norm(PARA_normalVect);
disp(distToWall);

end

