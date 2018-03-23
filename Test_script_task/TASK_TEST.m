
%                          TASK_TEST
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%This script is a test version of the element TASK
%It is linked to the files fct_traj_test and PARAMETERS_TEST
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Call global
%PARA
global PARA_N;

global PARA_kp;
global PARA_kd;
global PARA_t0;
global PARA_deltat_simu;

global PARA_x_des;

global PARA_robot;

%MAIN
global MAIN_q;
global MAIN_dotq;
global MAIN_J;
global MAIN_dotJ;
global MAIN_invJ

%OUT
global TASK_ddotx_desN;

%Initialization
task_posDes = PARA_x_des;
task_maxVel = 0.3;
task_firstTrajCall = true;
task_k=PARA_t0*PARA_deltat_simu;
task_q=MAIN_q;
task_dotq=MAIN_dotq;

task_ki=task_k;

TASK_ddotx_desN=zeros(3,PARA_N);
%disp(TASK_ddotx_desN);
posRefN=[];

for i = 1:PARA_N
    task_trajOut = fct_traj_TEST(task_ki,task_firstTrajCall, task_posDes, task_maxVel, task_q);
    
    task_accRef = task_trajOut{3};
    task_velRef = task_trajOut{2};
    task_posRef = task_trajOut{1};
    posRefN=[posRefN,task_posRef];
    task_firstTrajCall = task_trajOut{4};
    
    task_ddotx_des = task_accRef;
    
    if i==2
        task_firstTrajCall = task_trajOut{4};
    
        task_pos_i = PARA_robot.fkine(task_q);
        task_pos_i = task_pos_i(1:3,4);

        task_vel_i = MAIN_J*task_dotq;
        task_vel_i = task_vel_i(1:3,1);

        task_pos_erri = task_posRef - task_pos_i;
        task_vel_erri = task_velRef - task_vel_i;

        task_ddotx_des = task_accRef + PARA_kp * task_pos_erri + PARA_kd * task_vel_erri;
    end
    
    
    TASK_ddotx_desN(:,i) = task_ddotx_des;

    task_q = task_q + PARA_deltat_simu * task_dotq + 0.5*(PARA_deltat_simu^2.0)*MAIN_invJ*(task_ddotx_des - MAIN_dotJ);
    task_dotq = task_dotq + PARA_deltat_simu*MAIN_invJ*(task_ddotx_des - MAIN_dotJ);
    
    task_ki = task_ki +1;
end
disp(TASK_ddotx_desN);
for i=1:3
    figure(i)
    plot(TASK_ddotx_desN(i,:));
    title(strcat('Evolution of prev. op. acc. coord. ',int2str(i) ));
    xlabel('iteration');
    ylabel('ddotx_ref (rad/s²)');
end
for i=4:6
    figure(i)
    plot(posRefN(i-3,:));
    title(strcat('Evolution of prev. op. pos. coord. ',int2str(i) ));
    xlabel('iteration');
    ylabel('x_ref (rad)');
end

% disp (PARA_N);
% 
% disp (PARA_kp);
% disp (PARA_kd);
% disp (PARA_t0);
% disp (PARA_deltat_simu);
% 
% disp ( PARA_x_des);
% 
% disp ( PARA_robot);
% 
% %MAIN
% disp ( MAIN_q);
% disp ( MAIN_dotq);
% disp ( MAIN_J);
% disp ( MAIN_dotJ);
% disp ( MAIN_invJ);