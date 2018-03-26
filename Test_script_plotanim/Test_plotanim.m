global PARA_n_EO;
global PARA_n;

global PARA_robot;

global PARA_x_des;
global PARA_q_min;
global PARA_q_max;

global SAVE_tau_all;
global SAVE_tau_minall;
global SAVE_tau_maxall;
global SAVE_ddotq_all;
global SAVE_dotq_all;
global SAVE_q_all;
global SAVE_ddotq_minposall;
global SAVE_ddotq_maxposall;
global SAVE_ddotx_all;
global SAVE_ddotx_desall;
global SAVE_x_all
global SAVE_x_desall;


plot_nbIter = size(SAVE_tau_all,2);
plot_listIter = 1:plot_nbIter;

%Animation
figure(1);
for k=1:plot_nbIter
    hold on;
    colors={'b', 'r'};
    plot3(PARA_x_des(1,1), PARA_x_des(2,1), PARA_x_des(3,1), colors {1,1}, 'LineWidth', 4);
    hold off
    PARA_robot.plot(SAVE_q_all(:,k), 'delay', 0.01);
end


%Evolution of joint torque
figure(2);
for k = 1:PARA_n
    subplot(2,3,k);
    hold on;
    plot(plot_listIter, SAVE_tau_all(k,1:plot_nbIter));
    plot(plot_listIter, SAVE_tau_minall(k,1:plot_nbIter));
    plot(plot_listIter, SAVE_tau_maxall(k,1:plot_nbIter));
    xlabel('Iteration');
    ylabel('Torque (N.m)');
    title(strcat('DOF ',32,int2str(k)));
    suptitle('Joint Torques and Bounds for each DOF');
    legend('tau','tau_min','tau_max');
    hold off;
end

%Evolution of joint acceleration
figure(3);
for k = 1:PARA_n
    subplot(2,3,k);
    hold on;
    plot(plot_listIter, SAVE_ddotq_all(k,1:plot_nbIter));
    plot(plot_listIter, SAVE_ddotq_minposall(k,1:plot_nbIter));
    plot(plot_listIter, SAVE_ddotq_maxposall(k,1:plot_nbIter));
    xlabel('Iteration');
    ylabel('Acceleration (rad/s²)');
    title(strcat('DOF ',32,int2str(k)));
    suptitle('Joint Accelerations and Limits for each DOF');
    legend('ddotq','ddotq_m_i_n','ddotq_m_a_x');
    hold off;
end

%Evolution of joint velocity
figure(4);
for k = 1:PARA_n
    subplot(2,3,k);
    hold on;
    plot(plot_listIter, SAVE_dotq_all(k,1:plot_nbIter));
    xlabel('Iteration');
    ylabel('Velocity (rad/s)');
    title(strcat('DOF ',32,int2str(k)));
    suptitle('Joint Velocities for each DOF');
    hold off;
end

%Evolution of joint positions
figure(5);
for k = 1:PARA_n
    subplot(2,3,k);
    hold on;
    plot(plot_listIter, SAVE_q_all(k,1:plot_nbIter));
    plot([plot_listIter(1,1) plot_listIter(plot_nbIter,1)], [PARA_q_min(k,1) PARA_q_min(k,1)]);
    plot([plot_listIter(1,1) plot_listIter(plot_nbIter,1)], [PARA_q_max(k,1) PARA_q_max(k,1)]);
    xlabel('Iteration');
    ylabel('Position(rad)');
    title(strcat('DOF ',32,int2str(k)));
    suptitle('Joint Positions and Bounds for each DOF');
    legend('ddotq','ddotq_m_i_n','ddotq_m_a_x');
    hold off;
end

%Evolution of desired and actual operational acceleration
figure(6);
for k = 1:PARA_n_EO
    subplot(2,3,k);
    hold on;
    plot(plot_listIter, SAVE_ddotx_all(k,1:plot_nbIter));
    plot(plot_listIter, SAVE_ddotx_desall(k,1:plot_nbIter));
    xlabel('Iteration');
    ylabel('Acceleration (m/s²)');
    title(strcat('DOF ',32,int2str(k)));
    suptitle('Actual and Desired Operational Accelerations for each DOF');
    legend('actual','desired');
    hold off;
end

%Evolution of desired and actual operational acceleration
figure(7);
for k = 1:PARA_n_EO
    subplot(2,3,k);
    hold on;
    plot(plot_listIter, SAVE_x_all(k,1:plot_nbIter));
    plot(plot_listIter, SAVE_x_desall(k,1:plot_nbIter));
    xlabel('Iteration');
    ylabel('Position (m)');
    title(strcat('DOF ',32,int2str(k)));
    suptitle('Actual and Desired Operational Positions for each DOF');
    legend('actual','desired');
    hold off;
end