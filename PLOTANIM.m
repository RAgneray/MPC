
%                   PLOTANIM
% MPC v. 2.2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This script displays animation and plots from the SAVEDATA variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Calling global variables from other script
% From PARAMETERS
% boolean
global PARA_useAnimation;
global PARA_saveFig;
% integers
global PARA_n_EO;
global PARA_n;
% SerialLink
global PARA_robot;
% vectors
global PARA_x_des;
global PARA_q_min;
global PARA_q_max;

% From SAVEDATA
% matrices
global SAVE_tau_all;
global SAVE_tau_minall;
global SAVE_tau_maxall;
global SAVE_ddotq_all;
global SAVE_dotq_all;
global SAVE_q_all;
global SAVE_ddotq_minposall;
global SAVE_ddotq_maxposall;
global SAVE_ddotx_all;
global SAVE_ddotx_refall;
global SAVE_ddotx_desall;
global SAVE_x_all;
global SAVE_x_refall;

% Variables used for diagnosing task issues
% global SAVE_poserr;           
% global SAVE_velerr;                 
% global SAVE_kpposerr;                      
% global SAVE_kdvelerr; 

global TASK_q_ref;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Animation
if PARA_useAnimation
    figure(1);
    for l=1:size(SAVE_q_all,2)
        hold on;
        colors={'b', 'r'};
        plot3(PARA_x_des(1,1), PARA_x_des(2,1), PARA_x_des(3,1), colors {1,1}, 'LineWidth', 4);
        hold off
        plot_transpose_q = SAVE_q_all';
        PARA_robot.plot(plot_transpose_q(l,:));
    end
end

%Evolution of joint torque
figure(2);
for l = 1:PARA_n
    subplot(2,3,l);
    hold on;
    plot(1:size(SAVE_tau_all,2), SAVE_tau_all(l,:));
    plot(1:size(SAVE_tau_minall,2), SAVE_tau_minall(l,:));
    plot(1:size(SAVE_tau_maxall,2), SAVE_tau_maxall(l,:));
    xlabel('Iteration');
    ylabel('Torque (N.m)');
    title(strcat('DOF ',32,int2str(l)));
    suptitle('Joint Torques and Bounds for each DOF');
    legend('tau','tau_m_i_n','tau_m_a_x');
    hold off;
end
if PARA_saveFig
    savefig('J_Tor.fig');
end

%Evolution of joint acceleration
figure(3);
for l = 1:PARA_n
    subplot(2,3,l);
    hold on;
    plot(1:size(SAVE_ddotq_all,2), SAVE_ddotq_all(l,:));
    plot(1:size(SAVE_ddotq_minposall,2), SAVE_ddotq_minposall(l,:));
    plot(1:size(SAVE_ddotq_maxposall,2), SAVE_ddotq_maxposall(l,:));
    xlabel('Iteration');
    ylabel('Acceleration (rad/s²)');
    title(strcat('DOF ',32,int2str(l)));
    suptitle('Joint Accelerations and Limits for each DOF');
    legend('ddotq','ddotq_m_i_n','ddotq_m_a_x');
    hold off;
end
if PARA_saveFig
    savefig('J_Acc.fig');
end

%Evolution of joint velocity
figure(4);
for l = 1:PARA_n
    subplot(2,3,l);
    hold on;
    plot(1:size(SAVE_dotq_all,2), SAVE_dotq_all(l,:));
    xlabel('Iteration');
    ylabel('Velocity (rad/s)');
    title(strcat('DOF ',32,int2str(l)));
    suptitle('Joint Velocities for each DOF');
    hold off;
end
if PARA_saveFig
    savefig('J_Vel.fig');
end

%Evolution of joint positions
figure(5);
for l = 1:PARA_n
    subplot(2,3,l);
    hold on;
    plot(1:size(SAVE_q_all,2), SAVE_q_all(l,:));
    plot([1 size(SAVE_q_all,2)], [PARA_q_min(l,1) PARA_q_min(l,1)]);
    plot([1 size(SAVE_q_all,2)], [PARA_q_max(l,1) PARA_q_max(l,1)]);
    plot(1:size(TASK_q_ref,2), TASK_q_ref(l,:));
    xlabel('Iteration');
    ylabel('Position(rad)');
    title(strcat('DOF ',32,int2str(l)));
    suptitle('Joint Positions and Bounds for each DOF');
    legend('q','q_m_i_n','q_m_a_x','q_r_e_f');
    hold off;
end
if PARA_saveFig
    savefig('J_Pos.fig');
end

%Evolution of desired and actual operational acceleration
figure(6);
for l = 1:PARA_n_EO
    subplot(2,3,l);
    hold on;
    plot(1:size(SAVE_ddotx_all,2), SAVE_ddotx_all(l,:));
    plot(1:size(SAVE_ddotx_desall,2), SAVE_ddotx_desall(l,:));
    plot(1:size(SAVE_ddotx_refall,2), SAVE_ddotx_refall(l,:));
    xlabel('Iteration');
    if l <= 3
        ylabel('Acceleration (m/s²)');
    else
        ylabel('Acceleration (rad/s²)');
    end
    if l==1
        title('x');
    elseif l==2
        title('y');
    elseif l==3
        title('z');
    elseif l==4
        title('Roll');
    elseif l==5
        title('Pitch');
    elseif l==6
        title('Yaw');
    end
    suptitle('Actual and Reference Operational Accelerations for each Coordinates');
    legend('actual','desired','reference');
    hold off;
end
if PARA_saveFig
    savefig('Op_Acc.fig');
end

%Evolution of desired and actual operational acceleration
figure(7);
for l = 1:PARA_n_EO
    subplot(2,3,l);
    hold on;
    plot(1:size(SAVE_x_all,2), SAVE_x_all(l,:));
    plot(1:size(SAVE_x_refall,2), SAVE_x_refall(l,:));
    plot([1 size(SAVE_x_all,2)], [PARA_x_des(l,1) PARA_x_des(l,1)]);
    xlabel('Iteration');
    if l <= 3
        ylabel('Position (m)');
    else
        ylabel('Position (rad)');
    end
    if l==1
        title('x');
    elseif l==2
        title('y');
    elseif l==3
        title('z');
    elseif l==4
        title('Roll');
    elseif l==5
        title('Pitch');
    elseif l==6
        title('Yaw');
    end
    suptitle('Actual, Reference and Desired Operational Positions for each Coordinates');
    legend('actual','reference','desired');
    hold off;
end
if PARA_saveFig
    savefig('Op_Pos.fig');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Figure used for diagnosing error in task

% % Evolution of error between actual and reference position
% figure(8)
% for l = 1:PARA_n_EO
%     hold on;
%     subplot(2,3,l);
%     plot(SAVE_poserr(l,:));
%     title(strcat('DOF ',32,int2str(l)));
%     suptitle('Error between actual and reference position');
%     hold off;
% end
% 
% % Evolution of error between actual and reference velocity
% figure(9)
% for l = 1:PARA_n_EO
%     hold on;
%     subplot(2,3,l);
%     plot(SAVE_velerr(l,:));
%     title(strcat('DOF ',32,int2str(l)));
%     suptitle('Error between actual and reference velocity');
%     hold off;
% end
% 
% % Evolution of the parts of the desired acceleration
% figure(10)
% for l = 1:PARA_n_EO
%     subplot(2,3,l);
%     hold on;
%     plot(1:size(SAVE_kpposerr,2),SAVE_kpposerr(l,:));
%     plot(1:size(SAVE_kdvelerr,2),SAVE_kdvelerr(l,:));
%     plot(1:size(SAVE_ddotx_refall,2),SAVE_ddotx_refall(l,:));
%     plot(1:size(SAVE_ddotx_desall,2),SAVE_ddotx_desall(l,:));
%     legend('kp*pos_r_e_f','kd*vel_r_e_f','reference','desired');
%     title(strcat('DOF ',32,int2str(l)));
%     suptitle('Evolution of the parts of the desired acceleration');
%     hold off;
% end
