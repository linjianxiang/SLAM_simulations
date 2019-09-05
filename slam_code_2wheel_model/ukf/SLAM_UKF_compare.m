%%%% compare UKF performance of using control input sigma for updating
%%%% covariance matrix.
%%%% process noise sigma points are generated to avoid choosing process noise
%%%% covariance matrix for states (V)

clc;clear

%SLAM-UKF testing bench
iteration = 1000;

%%%%%%%%%%%INITIALIZATION
%%%TIME INTERVAL INIT
dt = 0.1;
%%%ROBOT INIT
X_Robot = zeros(3,1); %robot states init, (x,y,theta)
P_xx = zeros(3,3); %robot states error covariance matrix 
%%%LANDMARKS INIT
landmark_number = 20; %number of landmarks
n_states = 3+2*landmark_number;
map_length = 10; %map size
figure(1); % create figure
cla % clear axes
axis([-map_length map_length -map_length map_length]) % set axes limits
axis square
X_Landmark = zeros(2*landmark_number,1); %landmarks states init, L_x and L_y
%%%THE MAP INIT
X_use_CI_sigma = [X_Robot ; X_Landmark]; %the map states init
X = X_use_CI_sigma;
%P_LL = diag(inf(2*landmark_number,1)); %landmarks states error covariance matrix
P_LL =100* diag(ones(2*landmark_number,1)); %landmarks states error covariance matrix;
P_use_CI_sigma = zeros(n_states,n_states); 
%P(1:3,1:3)= P_xx; P(4:end,4:end)=P_LL;%the map error covariance
% P_xL = randn(3,2*landmark_number);P(1:3,4:end)=P_xL;P(4:end,1:3)=P_xL'; 
P_init = P_use_CI_sigma;
%robot states index
r = 1:3;
%%%LANDMARKS GENERATION
random_landmark = true; %true for random_landmark, false for fixed landmark
L = landmarks_generate(map_length,landmark_number,random_landmark); %landmark location are generated,size of(2,landmark_number)
L_estimate_use_CI_sigma = zeros(2,landmark_number);
L_estimate = L_estimate_use_CI_sigma;
L_UKF_free = zeros(2,landmark_number);
%observed landmarks index
landmark_obsved = [];
Y = zeros(2,landmark_number); %landmark observation
Y_v = zeros(2*landmark_number,1);
%noise covariance
Q1 = zeros(3,3);
q = 3*[.01;pi/1000];% control noise
% q = [0;0]
Q = diag(q.^2);
Q1(1:2,1:2) = Q;
q_state = [.01;.01;pi/500]; % assumed process noise 
Q_state = diag(q_state.^2); 
 Q_state = 0;
 
v = [0.2;pi/100];% measurement noise
V = diag(v.^2);
v_obsved = []; %noise covariance for all observed landmark
    
%%%robot inputs
u_v = 1; %line speed
u_w = pi/10; %angular velocity
u_origin = [u_v ; u_w]; %inputs
u_n = [u_v ; u_w]; %noised inputs
first_obs = true;
%%%expected trajectory
X_expect = [0;0;0];
robot_error_use_CI_sigma = zeros(3,iteration);
%%% UKF parameter
alpha = 0.55; 
kappa = 3;
beta = 10; %influence the Weight of mean covariance
sigma_param = struct('alpha', alpha,'kappa',kappa,'beta',beta);
%%%%%%%%%%%PLOT SETUP
%cart shape gen
Tri_shape0 = [0.4 0.1 0.1 0.4;
             0 pi*2/3 -pi*2/3 0]; %polar value [r phi] for triangle shape
Tri_shape_expect = polar2cart(X_expect,Tri_shape0);
Tri_shape_est = polar2cart(X_use_CI_sigma(1:3),Tri_shape0);
%estimated robot location
robot_plot_use_CI_sigma = line(...
        'linestyle','-',...
        'marker','none',...
        'color','b',...
        'xdata',Tri_shape_est(1,:),...
        'ydata',Tri_shape_est(2,:));
robot_plot = line(...
        'linestyle','-',...
        'marker','none',...
        'color','k',...
        'xdata',Tri_shape_est(1,:),...
        'ydata',Tri_shape_est(2,:));
%expected robot location
robot_expect_plot = line(...
        'linestyle','-',...
        'marker','none',...
        'color','r',...
        'xdata',Tri_shape_expect(1,:),...
        'ydata',Tri_shape_expect(2,:));
%real landmark location
landmark_real_plot = line(...
    'linestyle','none',...
    'marker','+',...
    'color','r',...
    'xdata',L(1,:),...
    'ydata',L(2,:));
%estimated landmark location
landmark_estimated_plot_use_CI_sigma = line(...
    'linestyle','none',...
    'marker','+',...
    'color','b',...
    'xdata',L(1,:),...
    'ydata',L(2,:));
landmark_estimated_plot = line(...
    'linestyle','none',...
    'marker','+',...
    'color','k',...
    'xdata',L(1,:),...
    'ydata',L(2,:));
%UKF free observed landmark location
% landmark_ekf_free_plot = line(...
%     'linestyle','none',...
%     'marker','+',...
%     'color','g',...
%     'xdata',L(1,:),...
%     'ydata',L(2,:));

%%%%%%%%%%%%UKF-SLAM


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%X_Landmark = landmarks_first_obs(X_Robot,L,noise_info); 
%init
P_use_CI_sigma = 0.001*eye(n_states,n_states);
P = P_use_CI_sigma;
%observe sequence generate
obsv_sequence = randperm(landmark_number,landmark_number);
states_obsved = [1 2 3];

for j = 1:iteration
%     if rem(j,500) == 0
%        P_use_CI_sigma = 0.001*eye(n_states,n_states);
%     end
%     
%      %move landmark
%     if rem(j,30) == 0
%         landmark_move_i = [1:5];
%         move_dist = L(:,landmark_move_i)./100;
%         L(:,landmark_move_i) = landmark_move(L(:,landmark_move_i),move_dist);
%     end
    %%%observe new landmark
    lm_left = find(obsv_sequence);
    if lm_left 
        v_obsved = [v_obsved;v];
        lm_number = obsv_sequence(lm_left(1));
        obsv_sequence(lm_left(1)) = 0;
        %increase states
        L_index = 3+2*(lm_number-1)+1:3+2*(lm_number-1)+2;
        %robot states indices and observed landmark indeics
        states_obsved = [states_obsved, L_index];
        %observed landmark indices
        landmark_obsved = [landmark_obsved,lm_number];
        %observe the current landmark range and angle [r phi]
        L_obsv= landmarks_obsv(X_expect,L(:,lm_number))+ v.*randn(2,1);
        %convert measurement to states
        [X_use_CI_sigma, P_use_CI_sigma] = UKF_newLM(X_expect,X_use_CI_sigma,P_use_CI_sigma,V,L_obsv,states_obsved,L_index,sigma_param);
         [X, P] = UKF_newLM(X_expect,X,P,V,L_obsv,states_obsved,L_index,sigma_param);
% % % % %         [measure_sigma,W_c1,W_m1] = compute_sigma_points(L_obsv,V,sigma_param);
% % % % %         l_sigma = inverse_landmark_obsv(X_expect, measure_sigma);
% % % % %         %%%%%%use CI sigma points
% % % % %         X_use_CI_sigma(L_index) = sigma_mean_cal(l_sigma,W_m1);
% % % % %         %increase size of error covariance matrix
% % % % %         %new landmark auto covariance
% % % % %         E_l_temp = l_sigma - X_use_CI_sigma(L_index);
% % % % %         P_use_CI_sigma(L_index,L_index) = sigma_cov_cal(E_l_temp,E_l_temp,W_c1);
% % % % %         %covariance for new landmark states and other states
% % % % %         [x_sigma_temp,W_c,W_m] = compute_sigma_points(X_use_CI_sigma(states_obsved),P_use_CI_sigma(states_obsved,states_obsved),sigma_param);
% % % % %         x_sigma_temp_ave = sigma_mean_cal(x_sigma_temp,W_m);
% % % % %         x_sigma_temp_ave(3) = wrapToPi(x_sigma_temp_ave(3));
% % % % %         E_x_sigma_temp = x_sigma_temp-x_sigma_temp_ave;
% % % % %         E_x_sigma_temp(3,:) = wrapToPi(E_x_sigma_temp(3,:));
% % % % %         P_use_CI_sigma(states_obsved(1:end-2),L_index) = sigma_cov_cal(E_x_sigma_temp(1:end-2,:),E_x_sigma_temp(end-1:end,:),W_c);
% % % % %         P_use_CI_sigma(L_index,states_obsved(1:end-2)) = P_use_CI_sigma(states_obsved(1:end-2),L_index)'; 
% % % % %         %%%%not use CI sigma points
% % % % %        
% % % % %         X(L_index) = sigma_mean_cal(l_sigma,W_m1); 
% % % % % % % % % % %         increase size of error covariance matrix
% % % % % % % % % % %         new landmark auto covariance
% % % % %         E_l_temp = l_sigma - X(L_index);
% % % % %         P(L_index,L_index) = sigma_cov_cal(E_l_temp,E_l_temp,W_c1);
% % % % % % % % % %         covariance for new landmark states and other states
% % % % %         [x_sigma_temp,W_c,W_m] = compute_sigma_points(X(states_obsved),P(states_obsved,states_obsved),sigma_param);
% % % % %         x_sigma_temp_ave = sigma_mean_cal(x_sigma_temp,W_m);
% % % % %          x_sigma_temp_ave(3) = wrapToPi(x_sigma_temp_ave(3));
% % % % %         E_x_sigma_temp = x_sigma_temp-x_sigma_temp_ave;
% % % % %         E_x_sigma_temp(3,:) = wrapToPi(E_x_sigma_temp(3,:));
% % % % %         P(states_obsved(1:end-2),L_index) = sigma_cov_cal(E_x_sigma_temp(1:end-2,:),E_x_sigma_temp(end-1:end,:),W_c);
% % % % %         P(L_index,states_obsved(1:end-2)) = P(states_obsved(1:end-2),L_index)'; 
    end
    V_obsved = diag(v_obsved);
    %expect trajectory plot
    X_expect = robot_motion(X_expect,u_origin,dt);
    X_expect(3) = wrapToPi(X_expect(3));
    %landmark obvs
    for i = landmark_obsved
        M = landmarks_obsv(X_expect,L(:,i))+ v.*randn(2,1);
        M(2) = wrapToPi(M(2));
        Y(:,i) = M;
        Y_v(2*(i-1)+1:2*(i-1)+2) = M;
    end
    %%%CONTROL INPUT sigma points
    u = [u_v;u_w] + q.*randn(2,1);
%     [u_sigma,]=sigma_points_gen(u,Q,sigma_param);
    u_n(1) = u(1);
    u_n(2)= u(2);
    %%%UKF PREDICTION
    %state predict use control input sigma for states and covariance matrix
    %updating
%     Q1 = zeros(5,5);
%     Q1(r,r) =P_use_CI_sigma(r,r);
%     Q1(4:5,4:5) = Q;
%     X_temp = zeros(5,1);
%     X_temp(r) = X_use_CI_sigma(r);
%     X_temp(4:5) = u_n;
%     [X_sigma_temp,W_c,W_m] = compute_sigma_points(X_temp,Q1,sigma_param);
%     X_sigma_r = X_sigma_temp(r,:);
%     u_sigma_r = X_sigma_temp(4:5,:);
%     X_Robot_sigma = robot_motion(X_sigma_r,u_sigma_r,dt);  %sigma point after motion
%     X_Robot_mean = sigma_mean_cal(X_Robot_sigma,W_m); %state mean
%     X_Robot_mean(3) = wrapToPi(X_Robot_mean(3));
%     E_x_robot = X_Robot_sigma - X_Robot_mean;
%     E_x_robot(3,:) = wrapToPi(E_x_robot(3,:));
%     
%     X_Robot_sigma2 = robot_motion(X_use_CI_sigma(r),u_sigma_r,dt);
%     X_Robot_mean2 = sigma_mean_cal(X_Robot_sigma2,W_m); %state mean
%     X_Robot_mean2(3) = wrapToPi(X_Robot_mean2(3));
%     E_x_robot2 = X_Robot_sigma2 - X_Robot_mean2;
%     E_x_robot2(3,:) = wrapToPi(E_x_robot2(3,:));
%     P_use_CI_sigma(r,r) = sigma_cov_cal(E_x_robot,E_x_robot,W_c)+sigma_cov_cal(E_x_robot2,E_x_robot2,W_c);
%     X_use_CI_sigma(r) = X_Robot_mean;
%     
% %     Q1 = zeros(5,5);
% %     Q1(r,r) =P_use_CI_sigma(r,r);
% %     Q1(4:5,4:5) = Q;
% %     X_temp = zeros(5,1);
% %     X_temp(r) = X_use_CI_sigma(r);
% %     X_temp(4:5) = u_n;
% %     [X_sigma_temp,W_c,W_m] = compute_sigma_points(X_temp,Q1,sigma_param);
% %     X_sigma_r = X_sigma_temp(r,:);
% %     u_sigma_r = X_sigma_temp(4:5,:);
% %     X_Robot_sigma = robot_motion(X_sigma_r,u_sigma_r,dt);  %sigma point after motion
% %     X_Robot_mean = sigma_mean_cal(X_Robot_sigma,W_m); %state mean
% %     X_Robot_mean(3) = wrapToPi(X_Robot_mean(3));
% %     E_x_robot = X_Robot_sigma - X_Robot_mean;
% %     E_x_robot(3,:) = wrapToPi(E_x_robot(3,:));
% %     P_use_CI_sigma(r,r) = sigma_cov_cal(E_x_robot,E_x_robot,W_c);
% %     X_use_CI_sigma(r) = X_Robot_mean;
% %     
    [X_use_CI_sigma,P_use_CI_sigma] = UKF_prediction_original(X_use_CI_sigma,P_use_CI_sigma,u_n,Q,dt,sigma_param,states_obsved,0);
    
    [X,P] = UKF_prediction(X,P,u_n,Q,dt,sigma_param,states_obsved,0);
% %     Q1 = zeros(5,5);
% %     Q1(r,r) =P(r,r);
% %     Q1(4:5,4:5) = Q;
% %     X_temp = zeros(5,1);
% %     X_temp(r) = X(r);
% %     X_temp(4:5) = u_n;
% %     [X_sigma_temp,W_c,W_m] = compute_sigma_points(X_temp,Q1,sigma_param);
% %     X_sigma_r = X_sigma_temp(r,:);
% %     u_sigma_r = X_sigma_temp(4:5,:);
% %     X_Robot_sigma = robot_motion(X_sigma_r,u_sigma_r,dt);  %sigma point after motion
% %     X_Robot_mean = sigma_mean_cal(X_Robot_sigma,W_m); %state mean
% %     X_Robot_mean(3) = wrapToPi(X_Robot_mean(3));
% %     X(r) = X_Robot_mean;
% %     
% %     X_Robot_sigma = robot_motion(X_sigma_r,u_n,dt);  %sigma point after motion
% %     X_Robot_mean = sigma_mean_cal(X_Robot_sigma,W_m); %state mean
% %     X_Robot_mean(3) = wrapToPi(X_Robot_mean(3));    
% %     E_x_robot = X_Robot_sigma - X_Robot_mean;
% %     E_x_robot(3,:) = wrapToPi(E_x_robot(3,:));
% %     
% %     X_Robot_sigma2 = robot_motion(X(r),u_sigma_r,dt);
% %     X_Robot_mean2 = sigma_mean_cal(X_Robot_sigma2,W_m); %state mean
% %     X_Robot_mean2(3) = wrapToPi(X_Robot_mean2(3));
% %     E_x_robot2 = X_Robot_sigma2 - X_Robot_mean2;
% %     E_x_robot2(3,:) = wrapToPi(E_x_robot2(3,:));
% %     P(r,r) = sigma_cov_cal(E_x_robot,E_x_robot,W_c)+sigma_cov_cal(E_x_robot2,E_x_robot2,W_c);

%     Q1 = zeros(5,5);
%     Q1(r,r) =P(r,r);
%     Q1(4:5,4:5) = Q;
%     X_temp = zeros(5,1);
%     X_temp(r) = X(r);
%     X_temp(4:5) = u_n;
%     [X_sigma_temp,W_c,W_m] = compute_sigma_points(X_temp,Q1,sigma_param);
%     X_sigma_r = X_sigma_temp(r,:);
%     u_sigma_r = X_sigma_temp(4:5,:);
%     X_Robot_sigma = robot_motion(X_sigma_r,u_sigma_r,dt);  %sigma point after motion
%     X_Robot_mean = sigma_mean_cal(X_Robot_sigma,W_m); %state mean
%     X_Robot_mean(3) = wrapToPi(X_Robot_mean(3));
%     E_x_robot = X_Robot_sigma - X_Robot_mean;
%     E_x_robot(3,:) = wrapToPi(E_x_robot(3,:));
%     
%     X_Robot_sigma2 = robot_motion(X(r),u_sigma_r,dt);
%     X_Robot_mean2 = sigma_mean_cal(X_Robot_sigma2,W_m); %state mean
%     X_Robot_mean2(3) = wrapToPi(X_Robot_mean2(3));
%     E_x_robot2 = X_Robot_sigma2 - X_Robot_mean2;
%     E_x_robot2(3,:) = wrapToPi(E_x_robot2(3,:));
%     P(r,r) = sigma_cov_cal(E_x_robot,E_x_robot,W_c);%+sigma_cov_cal(E_x_robot2,E_x_robot2,W_c);
%     X(r) = X_Robot_mean;

    
%     
%     %only the noised control input used during the calculation
%     Q1 = zeros(5,5);
%     Q1(r,r) =P(r,r);
%     Q1(4:5,4:5) = Q;
%     X_temp = zeros(5,1);
%     X_temp(r) = X_use_CI_sigma(r);
%     X_temp(4:5) = u_n;
%     [X_sigma_temp,W_c,W_m] = compute_sigma_points(X_temp,Q1,sigma_param);
%     X_sigma_r = X_sigma_temp(r,:);
%     X_Robot_sigma = robot_motion(X_sigma_r,u_n,dt);  %sigma point after motion
%     X_Robot_mean = sigma_mean_cal(X_Robot_sigma,W_m); %state mean
%     X_Robot_mean(3) = wrapToPi(X_Robot_mean(3));
%     E_x_robot = X_Robot_sigma - X_Robot_mean;
%     E_x_robot(3,:) = wrapToPi(E_x_robot(3,:));
%     P(r,r) = sigma_cov_cal(E_x_robot,E_x_robot,W_c)+ Q_state;
%     X(r) = X_Robot_mean;
%     
   %%%%%%Estimation UKF
     %state update use control input sigma for states and covariance matrix updating
% %     [X_sigma,W_c,W_m] = compute_sigma_points(X_use_CI_sigma(states_obsved),P_use_CI_sigma(states_obsved,states_obsved),sigma_param);
% %     X_sigma(3,:) = wrapToPi(X_sigma(3,:));
% %     Z = zeros(length(landmark_obsved)*2,1);
% %     Z_sigma = zeros(2,length(states_obsved)*2+1);
% %     for i = 1:length(landmark_obsved)
% %         L_index = 3+2*(i-1)+1:3+2*(i-1)+2; 
% %         zi_sigma = landmark_estimate_UKF(X_sigma(r,:),X_sigma(L_index,:)); % approach one
% %         z_temp = sigma_mean_cal(zi_sigma,W_m);
% %         z_temp(2) = wrapToPi(z_temp(2));
% %         Z(L_index-3) = z_temp;
% %         Z_sigma(L_index-3,:) = zi_sigma;      
% %     end
% %     E_x = X_sigma - X_use_CI_sigma(states_obsved);
% %     E_x(3,:) = wrapToPi(E_x(3,:));
% %     E_z = Z_sigma - Z;
% %     E_z(2:2:end,:) = wrapToPi(E_z(2:2:end,:));
% %     P_z = sigma_cov_cal(E_z,E_z,W_c);
% %     P_z = P_z + V_obsved;
% %     P_xy = sigma_cov_cal(E_x,E_z,W_c);
% %     K = P_xy * inv(P_z);
% %     D = Y_v(states_obsved(4:end)-3) - Z;
% %     D(2:2:end) = wrapToPi(D(2:2:end));
% %     X_new = X_use_CI_sigma(states_obsved) + K* D;
% %     X_use_CI_sigma(states_obsved) = X_new;
% %     X_use_CI_sigma(3) = wrapToPi(X_use_CI_sigma(3));
% %     P_use_CI_sigma(states_obsved,states_obsved) = P_use_CI_sigma(states_obsved,states_obsved) - K*P_z*K';
% %     for i = 1:landmark_number
% %         L_index = 3+2*(i-1)+1:3+2*(i-1)+2;
% %         L_estimate_use_CI_sigma(:,i) = X_use_CI_sigma(L_index);
% %         L_EKF_free_use_CI_sigma(:,i) = inverse_landmark_obsv(X_use_CI_sigma(r), Y(:,i));
% %     end
% %     
    [X_use_CI_sigma,P_use_CI_sigma,L_estimate_use_CI_sigma] = UKF_update(X_use_CI_sigma,P_use_CI_sigma,Y_v,v,sigma_param,landmark_obsved,landmark_number,states_obsved);
    [X,P,L_estimate] = UKF_update(X,P,Y_v,v,sigma_param,landmark_obsved,landmark_number,states_obsved);
% %     %only the noised control input used during the calculation
% %     [X_sigma,W_c,W_m] = compute_sigma_points(X(states_obsved),P(states_obsved,states_obsved),sigma_param);
% %     X_sigma(3,:) = wrapToPi(X_sigma(3,:));
% %     Z = zeros(length(landmark_obsved)*2,1);
% %     Z_sigma = zeros(2,length(states_obsved)*2+1);
% %     for i = 1:length(landmark_obsved)
% %         L_index = 3+2*(i-1)+1:3+2*(i-1)+2; 
% %         zi_sigma = landmark_estimate_UKF(X_sigma(r,:),X_sigma(L_index,:)); % approach one
% %         z_temp = sigma_mean_cal(zi_sigma,W_m);
% %         z_temp(2) = wrapToPi(z_temp(2));
% %         Z(L_index-3) = z_temp;
% %         Z_sigma(L_index-3,:) = zi_sigma;    
% %     end
% %     E_x = X_sigma - X(states_obsved);
% %     E_x(3,:) = wrapToPi(E_x(3,:));
% %     E_z = Z_sigma - Z;
% %     E_z(2:2:end,:) = wrapToPi(E_z(2:2:end,:));
% %     P_z = sigma_cov_cal(E_z,E_z,W_c);
% %     P_z = P_z + V_obsved;
% %     P_xy = sigma_cov_cal(E_x,E_z,W_c);
% %     K = P_xy * inv(P_z);
% %     D = Y_v(states_obsved(4:end)-3) - Z;
% %     D(2:2:end) = wrapToPi(D(2:2:end));
% %     X(states_obsved) = X(states_obsved) + K* D;
% %     X(3) = wrapToPi(X(3));
% %     P(states_obsved,states_obsved) = P(states_obsved,states_obsved) - K*P_z*K';
% %     for i = 1:landmark_number
% %         L_index = 3+2*(i-1)+1:3+2*(i-1)+2;
% %         L_estimate(:,i) = X(L_index);
% %         L_EKF_free(:,i) = inverse_landmark_obsv(X(r), Y(:,i));
% %     end
    
    %%%plotting
    %used CI sigma
    Tri_shape_est = polar2cart(X_use_CI_sigma(r),Tri_shape0);
    Tri_shape_expect = polar2cart(X_expect,Tri_shape0);
    set(robot_plot_use_CI_sigma,'xdata',Tri_shape_est(1,:),'ydata',Tri_shape_est(2,:));
    set(robot_expect_plot,'xdata',Tri_shape_expect(1,:),'ydata',Tri_shape_expect(2,:));
    set(landmark_real_plot,'xdata',L(1,:),'ydata',L(2,:));
    set(landmark_estimated_plot_use_CI_sigma,'xdata',L_estimate_use_CI_sigma(1,:),'ydata',L_estimate_use_CI_sigma(2,:));
%     set(landmark_ekf_free_plot,'xdata',L_EKF_free_use_CI_sigma(1,:),'ydata',L_EKF_free_use_CI_sigma(2,:));
    %not using CI sigma
    Tri_shape_est = polar2cart(X(r),Tri_shape0);
    set(robot_plot,'xdata',Tri_shape_est(1,:),'ydata',Tri_shape_est(2,:));
    set(landmark_estimated_plot,'xdata',L_estimate(1,:),'ydata',L_estimate(2,:));
%     set(landmark_ekf_free_plot,'xdata',L_EKF_free(1,:),'ydata',L_EKF_free(2,:));
    title('SLAM-UKF simulation')
    % for error calculation
    %use ci sigma
    robot_error_j = X_use_CI_sigma(r) - X_expect;
    robot_error_j(3) = rad2deg(wrapToPi(robot_error_j(3)));
    robot_error_j(1:2) =robot_error_j(1:2).^2; 
    robot_error_use_CI_sigma(:,j) = robot_error_j;
    landmark_error = L(:,landmark_obsved) - L_estimate_use_CI_sigma(:,landmark_obsved);
    landmark_error_x_use_CI_sigma(j) = sum(landmark_error(1,:).^2);
    landmark_error_y_use_CI_sigma(j) = sum(landmark_error(2,:).^2);
    %dont use ci sigma
    robot_error_j = X(r) - X_expect;
    robot_error_j(3) = rad2deg(wrapToPi(robot_error_j(3)));
    robot_error_j(1:2) =robot_error_j(1:2).^2; 
    robot_error(:,j) = robot_error_j;
    landmark_error = L(:,landmark_obsved) - L_estimate(:,landmark_obsved);
    landmark_error_x(j) = sum(landmark_error(1,:).^2);
    landmark_error_y(j) = sum(landmark_error(2,:).^2);
    drawnow
end

figure(2)
subplot(2,1,1)
plot(landmark_error_x_use_CI_sigma,'color','b'); hold on;
plot(landmark_error_x,'color','k'); hold off;
title('landmark error for x direction')
xlabel('iteration')
ylabel('sum error square')
subplot(2,1,2)
plot(landmark_error_y_use_CI_sigma,'color','b'); hold on;
plot(landmark_error_y,'color','k'); hold off;
title('landmark error for y direction')
xlabel('iteration')
ylabel('sum error square')
figure(3)
subplot(3,1,1)
plot(robot_error_use_CI_sigma(1,:),'color','b'); hold on;
plot(robot_error(1,:),'color','k'); hold off;
subplot(3,1,2)
plot(robot_error_use_CI_sigma(2,:),'color','b'); hold on;
plot(robot_error(2,:),'color','k'); hold off;
subplot(3,1,3)
plot(abs(robot_error_use_CI_sigma(3,:)),'color','b'); hold on;
plot(abs(robot_error(3,:)),'color','k'); hold off;


