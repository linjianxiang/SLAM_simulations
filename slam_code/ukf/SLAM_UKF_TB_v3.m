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
X = [X_Robot ; X_Landmark]; %the map states init
%P_LL = diag(inf(2*landmark_number,1)); %landmarks states error covariance matrix
P_LL =100* diag(ones(2*landmark_number,1)); %landmarks states error covariance matrix;
P = zeros(n_states,n_states); 
%P(1:3,1:3)= P_xx; P(4:end,4:end)=P_LL;%the map error covariance
% P_xL = randn(3,2*landmark_number);P(1:3,4:end)=P_xL;P(4:end,1:3)=P_xL'; 
P_init = P;
%robot states index
r = 1:3;
%%%LANDMARKS GENERATION
random_landmark = false; %true for random_landmark, false for fixed landmark
L = landmarks_generate(map_length,landmark_number,random_landmark); %landmark location are generated,size of(2,landmark_number)
L_estimate = zeros(2,landmark_number);
L_UKF_free = zeros(2,landmark_number);
%observed landmarks index
landmark_obsved = [];
Y = zeros(2,landmark_number); %landmark observation
Y_v = zeros(2*landmark_number,1);
%noise covariance
Q1 = zeros(3,3);
q = [.02;pi/100]; % control noise
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
robot_error = zeros(3,iteration);
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
Tri_shape_est = polar2cart(X(1:3),Tri_shape0);
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

%UKF free observed landmark location
landmark_ekf_free_plot = line(...
    'linestyle','none',...
    'marker','+',...
    'color','g',...
    'xdata',L(1,:),...
    'ydata',L(2,:));

%%%%%%%%%%%%UKF-SLAM


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%X_Landmark = landmarks_first_obs(X_Robot,L,noise_info); 
%init
P = 0.001*eye(n_states,n_states);
%observe sequence generate
obsv_sequence = randperm(landmark_number,landmark_number);
states_obseved = [1 2 3];

for j = 1:iteration
    %%%observe new landmark
    lm_left = find(obsv_sequence);
    if lm_left 
        v_obsved = [v_obsved;v];
        lm_number = obsv_sequence(lm_left(1));
        obsv_sequence(lm_left(1)) = 0;
        %increase states
        L_index = 3+2*(lm_number-1)+1:3+2*(lm_number-1)+2;
        %robot states indices and observed landmark indeics
        states_obseved = [states_obseved, L_index];
        %observed landmark indices
        landmark_obsved = [landmark_obsved,lm_number];
        %observe the current landmark range and angle [r phi]
        measure_temp= landmarks_obsv(X_expect,L(:,lm_number))+ v.*randn(2,1);
        %convert measurement to states
        [measure_sigma,W_c,W_m] = compute_sigma_points(measure_temp,V,sigma_param);
        l_sigma = inverse_landmark_obsv(X_expect, measure_sigma);
        X(L_index) = sigma_mean_cal(l_sigma,W_m); 
        %increase size of error covariance matrix
        %new landmark auto covariance
        E_l_temp = l_sigma - X(L_index);
        P(L_index,L_index) = sigma_cov_cal(E_l_temp,E_l_temp,W_c);
        %covariance for new landmark states and other states
        [x_sigma_temp,W_c,W_m] = compute_sigma_points(X(states_obseved),P(states_obseved,states_obseved),sigma_param);
        x_sigma_temp_ave = sigma_mean_cal(x_sigma_temp,W_m);
        E_x_sigma_temp = x_sigma_temp-x_sigma_temp_ave;
        P(states_obseved(1:end-2),L_index) = sigma_cov_cal(E_x_sigma_temp(1:end-2,:),E_x_sigma_temp(end-1:end,:),W_c);
        P(L_index,states_obseved(1:end-2)) = P(states_obseved(1:end-2),L_index)'; 
    end
    V_obsved = diag(v_obsved);
    %expect trajectory plot
    X_expect = robot_motion(X_expect,u_origin,dt);
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
    %state predict
    [X_sigma_r,W_c,W_m] = compute_sigma_points(X(r),P(r,r),sigma_param);
    [u_sigma_r,~,~] = compute_sigma_points(u_n,Q1,sigma_param,length(X(r)));
    X_Robot_sigma = robot_motion(X_sigma_r,u_sigma_r,dt);  %sigma point after motion
    X_Robot_mean = sigma_mean_cal(X_Robot_sigma,W_m); %state mean
    E_x_robot = X_Robot_sigma - X_Robot_mean;
    E_x_robot(3,:) = wrapToPi(E_x_robot(3,:));
    P(r,r) = sigma_cov_cal(E_x_robot,E_x_robot,W_c);
    X(r) = X_Robot_mean;
   %%%%%%Estimation UKF
    [X_sigma,W_c,W_m] = compute_sigma_points(X(states_obseved),P(states_obseved,states_obseved),sigma_param);
    X_sigma(3,:) = wrapToPi(X_sigma(3,:));
    Z = zeros(length(landmark_obsved)*2,1);
    Z_sigma = zeros(2,length(states_obseved)*2+1);
    for i = 1:length(landmark_obsved)
        L_index = 3+2*(i-1)+1:3+2*(i-1)+2; 
        zi_sigma = landmark_estimate_UKF(X_sigma(r,:),X_sigma(L_index,:)); % approach one
        Z(L_index-3) = sigma_mean_cal(zi_sigma,W_m);
        Z_sigma(L_index-3,:) = zi_sigma;      
    end
    E_x = X_sigma - X(states_obseved);
    E_x(3,:) = wrapToPi(E_x(3,:));
    E_z = Z_sigma - Z;
    E_z(2:2:end,:) = wrapToPi(E_z(2:2:end,:));
    P_z = sigma_cov_cal(E_z,E_z,W_c);
    P_z = P_z + V_obsved;
    P_xy = sigma_cov_cal(E_x,E_z,W_c);
    K = P_xy * inv(P_z);
    D = Y_v(states_obseved(4:end)-3) - Z;
    D(2:2:end) = wrapToPi(D(2:2:end));
    X(states_obseved) = X(states_obseved) + K* D;
    X(3) = wrapToPi(X(3));
    P(states_obseved,states_obseved) = P(states_obseved,states_obseved) - K*P_z*K';
    for i = 1:landmark_number
        L_index = 3+2*(i-1)+1:3+2*(i-1)+2;
        L_estimate(:,i) = X(L_index);
        L_EKF_free(:,i) = inverse_landmark_obsv(X(r), Y(:,i));
    end
    
    Tri_shape_est = polar2cart(X(r),Tri_shape0);
    Tri_shape_expect = polar2cart(X_expect,Tri_shape0);
    set(robot_plot,'xdata',Tri_shape_est(1,:),'ydata',Tri_shape_est(2,:));
    set(robot_expect_plot,'xdata',Tri_shape_expect(1,:),'ydata',Tri_shape_expect(2,:));
    set(landmark_real_plot,'xdata',L(1,:),'ydata',L(2,:));
    set(landmark_estimated_plot,'xdata',L_estimate(1,:),'ydata',L_estimate(2,:));
    set(landmark_ekf_free_plot,'xdata',L_EKF_free(1,:),'ydata',L_EKF_free(2,:));
    title('SLAM-UKF simulation')
    % for error calculation
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
plot(landmark_error_x)
title('landmark error for x direction')
xlabel('iteration')
ylabel('sum error square')
subplot(2,1,2)
plot(landmark_error_y)
title('landmark error for y direction')
xlabel('iteration')
ylabel('sum error square')
figure(3)
subplot(3,1,1)
plot(robot_error(1,:));
subplot(3,1,2)
plot(robot_error(2,:));
subplot(3,1,3)
plot(robot_error(3,:));


