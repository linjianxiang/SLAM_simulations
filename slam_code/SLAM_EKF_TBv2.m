%try to implement UKF to handle process noise 

clc;clear;%close all

%SLAM-EKF testing bench
iteration = 600;
%%%%%%%%%%%INITIALIZATION
%%%TIME INTERVAL INIT
dt = 0.1;
%%%ROBOT INIT
X_Robot = zeros(3,1); %robot states init, (x,y,theta)
P_xx = zeros(3,3); %robot states error covariance matrix 
%%%LANDMARKS INIT
landmark_number = 10; %number of landmarks
map_length = 10; %map size
figure(1); % create figure
cla % clear axes
axis([-map_length map_length -map_length map_length]) % set axes limits
axis square
X_Landmark = zeros(2*landmark_number,1); %landmarks states init, L_x and L_y
%%%THE MAP INIT
X = [X_Robot ; X_Landmark]; %the map states init
%robot states index
r = 1:3;
%P_LL = diag(inf(2*landmark_number,1)); %landmarks states error covariance matrix
P_LL = 10000*diag(ones(2*landmark_number,1)); %landmarks states error covariance matrix;
P = zeros(3+2*landmark_number,3+2*landmark_number); 
P(r,r)= P_xx; P(4:3+2*landmark_number,4:3+2*landmark_number)=P_LL;%the map error covariance
% P_xL = randn(3,2*landmark_number);P(r,landmark_index)=P_xL;P(landmark_index,r)=P_xL'; 
P_init = P;

%%%LANDMARKS GENERATION
random_landmark = true;%true; %true for random_landmark, false for fixed landmark
L = landmarks_generate(map_length,landmark_number,random_landmark); %landmark location are generated,size of(2,landmark_number)
L_estimate = zeros(2,landmark_number);
L_EKF_free = zeros(2,landmark_number);
%observed landmarks index
landmark_obsved = [];
Y = zeros(2,landmark_number); %landmark observation
%%%noise covariance
Q1 = zeros(3,3);
q = 2*[.01;pi/1000]; % control noise
Q = diag(q.^2);
Q1(1:2,1:2) =Q;
v = 2*[.1;1*pi/180];% measurement noise
V = diag(v.^2);
%%%robot inputs
u_v = 1; %line speed
u_w = pi/10; %angular velocity
u = [u_v ; u_w]; %inputs
u_n = [u_v ; u_w]; %noised inputs
first_obs = true;
%%%expected trajectory
X_expect = [0;0;0];
X_filter_free = [0;0;0];
Tri_shape0 = [0.4 0.1 0.1 0.4;
             0 pi*2/3 -pi*2/3 0]; %polar value [r phi] for triangle shape
Tri_shape_expect = polar2cart(X_expect,Tri_shape0);
Tri_shape_est = polar2cart(X(r),Tri_shape0);
Tri_shape_filter_free= polar2cart(X_filter_free,Tri_shape0);
%%%%%%%%%%%PLOT SETUP
%estimated robot location
robot_plot = line(...
        'linestyle','-',...
        'marker','none',...
        'color','b',...
        'xdata',Tri_shape_est(1,:),...
        'ydata',Tri_shape_est(2,:));
%expected robot location
robot_expect_plot = line(...
        'linestyle','-',...
        'marker','none',...
        'color','r',...
        'xdata',Tri_shape_expect(1,:),...
        'ydata',Tri_shape_expect(2,:));
    
%expected robot location
robot_filter_free_plot = line(...
        'linestyle','-',...
        'marker','none',...
        'color','g',...
        'xdata',Tri_shape_filter_free(1,:),...
        'ydata',Tri_shape_filter_free(2,:));    
%real landmark location
landmark_real_plot = line(...
    'linestyle','none',...
    'marker','+',...
    'color','r',...
    'xdata',L(1,:),...
    'ydata',L(2,:));
%estimated landmark location
landmark_estimated_plot = line(...
    'linestyle','none',...
    'marker','+',...
    'color','b',...
    'xdata',L(1,:),...
    'ydata',L(2,:));

%EKF free observed landmark location
landmark_ekf_free_plot = line(...
    'linestyle','none',...
    'marker','+',...
    'color','g',...
    'xdata',L(1,:),...
    'ydata',L(2,:));

%%%%%%%%%%%%EKF-SLAM
%observe sequence generate
obsv_sequence = randperm(landmark_number,landmark_number);
states_obseved = [1 2 3];
%sigma parameters
alpha = 0.55; 
kappa = 3;
beta = 10; %influence the Weight of mean covariance
sigma_param = struct('alpha', alpha,'kappa',kappa,'beta',beta);
%%% EKF iterations
for j = 1:iteration
    %move landmark
%     if rem(j,30) == 0
%         landmark_move_i = [1:5];
%         move_dist = L(:,landmark_move_i)./100;
%         L(:,landmark_move_i) = landmark_move(L(:,landmark_move_i),move_dist);
%     end
    %%%observe new landmark
    lm_left = find(obsv_sequence);
    if lm_left 
        lm_number = obsv_sequence(lm_left(1));
        obsv_sequence(lm_left(1)) = 0;
        %increase states
        L_index = 3+2*(lm_number-1)+1:3+2*(lm_number-1)+2;
        %robot states indices and observed landmark indeics
        states_obseved = [states_obseved, L_index];
        %observed landmark indices
        landmark_obsved = [landmark_obsved,lm_number];
        %observe the current landmark range and angle [r phi]
        l_temp= landmarks_obsv(X_expect,L(:,lm_number))+ v.*randn(2,1);
        %convert measurement to states
        [X(L_index),dR,dL] = inverse_landmark_obsv(X_expect, l_temp);
        %increase size of error covariance matrix
        P(L_index,L_index) = dR * P(r,r) *dR' + dL * P(L_index,L_index) *dL';
        P(L_index,states_obseved(1:end-2)) = dR*P(r,states_obseved(1:end-2));
        P(states_obseved(1:end-2),L_index) =P(L_index,states_obseved(1:end-2))'; 
    end
    %expect trajectory, for plot
    X_expect = robot_motion(X_expect,u,dt); 
    %control input noise
    p_noise = q.*randn(2,1);
    u_n(1)=u_v+p_noise(1);
    u_n(2)=u_w+p_noise(2);
    X_filter_free =  robot_motion(X_filter_free,u_n,dt);
    %%%EKF PREDICTION
    Q1 = zeros(5,5);
    Q1(r,r) =P(r,r);
    Q1(4:5,4:5) = Q;
    X_temp = zeros(5,1);
    X_temp(r) = X(r);
    X_temp(4:5) = u_n;
    [X_sigma_temp,W_c,W_m] = compute_sigma_points(X_temp,Q1,sigma_param);
    X_sigma_r = X_sigma_temp(r,:);
    u_sigma_r = X_sigma_temp(4:5,:);
    X_Robot_sigma = robot_motion(X_sigma_r,u_sigma_r,dt);  %sigma point after motion
    X_Robot_mean = sigma_mean_cal(X_Robot_sigma,W_m); %state mean
    X_Robot_mean(3) = wrapToPi(X_Robot_mean(3));
    E_x_robot = X_Robot_sigma - X_Robot_mean;
    E_x_robot(3,:) = wrapToPi(E_x_robot(3,:));
    
    X_Robot_sigma2 = robot_motion(X(r),u_sigma_r,dt);
    X_Robot_mean2 = sigma_mean_cal(X_Robot_sigma2,W_m); %state mean
    X_Robot_mean2(3) = wrapToPi(X_Robot_mean2(3));
    E_x_robot2 = X_Robot_sigma2 - X_Robot_mean2;
    E_x_robot2(3,:) = wrapToPi(E_x_robot2(3,:));
    P(r,r) = sigma_cov_cal(E_x_robot,E_x_robot,W_c)+sigma_cov_cal(E_x_robot2,E_x_robot2,W_c);
    X(r) = X_Robot_mean;
    
%     X(r) = robot_motion(X(r),u_n,dt);  %X- calculation
    A_hat_robot = jacobian_robot_motion(X(r),u_n,dt); 
%     N = jacobian_robot_process_noise(X(r),u,dt,q); % use q to generate random n_v and n_w, failure testing
    P(states_obseved,states_obseved)= [A_hat_robot*P(r,r)*A_hat_robot' A_hat_robot*P(r,states_obseved(4:end));
                (A_hat_robot*P(r,states_obseved(4:end)))'  P(states_obseved(4:end),states_obseved(4:end))]; %P- calcualtion
    %%%EKF ESTIMATION
    %landmark observation
    for i = landmark_obsved
        M = landmarks_obsv(X_expect,L(:,i))+ v.*randn(2,1);
        Y(:,i) = M;
    end
    for i = landmark_obsved
        L_index = 3+(i-1)*2+1:3+(i-1)*2+2;
        Y_i = Y(:,i);  %i_th landmark obsv
        [h,C_shrink] = landmark_estimate(X(r),X(L_index)); %landmark estimation and its jacobian
        F_x = zeros(5,3+2*landmark_number); % transformation matrix, for less calculation
        F_x(r,r) = eye(3);
        F_x(4:5,L_index) = eye(2);
        C = C_shrink*F_x;
        K = P*C'*inv(C*P*C'+V); %feedback calucaltion
        E_y = Y_i-h; %output error
        E_y(2) = wrapToPi(E_y(2));
        X = X + K*E_y;  % state estimation
        X(3) = wrapToPi(X(3));
        P = (eye(3+2*landmark_number)-K*C)*P; %P update
        L_estimate(:,i) = X(L_index);
        L_EKF_free(:,i) = inverse_landmark_obsv(X(r), Y_i);
    end
    
    %%%plots update
    Tri_shape_est = polar2cart(X(r),Tri_shape0);
    Tri_shape_expect = polar2cart(X_expect,Tri_shape0);
    Tri_shape_filter_free = polar2cart(X_filter_free,Tri_shape0);
    set(robot_plot,'xdata',Tri_shape_est(1,:),'ydata',Tri_shape_est(2,:));
    set(robot_expect_plot,'xdata',Tri_shape_expect(1,:),'ydata',Tri_shape_expect(2,:));
    set(robot_filter_free_plot,'xdata',Tri_shape_filter_free(1,:),'ydata',Tri_shape_filter_free(2,:));
    set(landmark_real_plot,'xdata',L(1,:),'ydata',L(2,:));
    set(landmark_estimated_plot,'xdata',L_estimate(1,:),'ydata',L_estimate(2,:));
    set(landmark_ekf_free_plot,'xdata',L_EKF_free(1,:),'ydata',L_EKF_free(2,:));
    title('SLAM-EKF simulation')
    
    robot_error_j = X(r) - X_expect;
    robot_error_j(3) = rad2deg(wrapToPi(robot_error_j(3)));
    robot_error_j(1:2) =robot_error_j(1:2).^2; 
    robot_error(:,j) = robot_error_j;
    % for error calculation
    landmark_error = L - L_estimate;
    landmark_error_x(j) = sum(landmark_error(1,:).^2);
    landmark_error_y(j) = sum(landmark_error(2,:).^2);
    drawnow
end
figure(2)
subplot(2,1,1)
plot(landmark_error_x)
title('landmark error for x direction')
xlabel('iteration')
ylabel('sum error square')
subplot(2,1,2)
plot(landmark_error_y)
title('landmark error for y direction')
xlabel('iteration')
ylabel('sum error square')

figure()
subplot(2,1,1)
plot(landmark_error_x(100:end))
title('landmark error for x direction')
xlabel('iteration')
ylabel('sum error square')
subplot(2,1,2)
plot(landmark_error_y(100:end))
title('landmark error for y direction')
xlabel('iteration')
ylabel('sum error square')

figure()
title('robot error')
subplot(3,1,1)
% plot(robot_error_use_CI_sigma(1,:),'color','b');; hold on;
plot(robot_error(1,:),'color','k');; hold off;
subplot(3,1,2)
% plot(robot_error_use_CI_sigma(2,:),'color','b');; hold on;
plot(robot_error(2,:),'color','k');; hold off;
subplot(3,1,3)
% plot(abs(robot_error_use_CI_sigma(3,:)),'color','b');; hold on;
plot(abs(robot_error(3,:)),'color','k');; hold off;

