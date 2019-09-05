 clc;clear;close all;

%SLAM-EKF testing bench
iteration = 3000;
%%%%%%%%%%%INITIALIZATION
%%%TIME INTERVAL INIT
dt = 0.1;
%%%ROBOT INIT
X_Robot = zeros(3,1); %robot states init, (x,y,theta)
P_xx = zeros(3,3); %robot states error covariance matrix 
%%%LANDMARKS INIT
landmark_number = 20; %number of landmarks
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
state_number = 3+2*landmark_number;
P_LL = 10000*diag(ones(2*landmark_number,1)); %landmarks states error covariance matrix;
P = zeros(state_number,state_number);
P(r,r) = 0.1*diag([1 1 1]);
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
q = 3*[.01;pi/1000]; % control noise
Q = diag(q.^2);
Q1(1:2,1:2) =Q;
v = [.1;1*pi/180];% measurement noise
V = diag(v.^2);
%%%robot inputs
u_v = 1; %line speed
u_w = pi/10; %angular velocity
u = [u_v ; u_w]; %inputs
u_n = [u_v ; u_w]; %noised inputs
first_obs = true;
%%%expected trajectory
X_expect = [0;0;0];
Tri_shape0 = [0.4 0.1 0.1 0.4;
             0 pi*2/3 -pi*2/3 0]; %polar value [r phi] for triangle shape
Tri_shape_expect = polar2cart(X_expect,Tri_shape0);
Tri_shape_est = polar2cart(X(r),Tri_shape0);
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


%%% EKF iterations
for j = 1:iteration
%     %move landmark
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
        P(L_index,L_index) = dR * P(r,r) *dR' + dL * V *dL';
        P(L_index,states_obseved(1:end-2)) = dR*P(r,states_obseved(1:end-2));
        P(states_obseved(1:end-2),L_index) =P(L_index,states_obseved(1:end-2))'; 
    end
    %expect trajectory, for plot
    X_expect = robot_motion(X_expect,u,dt); 
    X_expect(3) = wrapToPi(X_expect(3));
    %control input noise
    p_noise = q.*randn(2,1);
    u_n(1)=u_v+p_noise(1);
    u_n(2)=u_w+p_noise(2);
    %%%EKF PREDICTION
    X(r) = robot_motion(X(r),u_n,dt);  %X- calculation
    X(3) = wrapToPi(X(3));
    A_hat_robot = jacobian_robot_motion(X(r),u_n,dt); 
%     N = jacobian_robot_process_noise(X(r),u,dt,q); % use q to generate random n_v and n_w, failure testing
%     P(states_obseved,states_obseved)= [A_hat_robot*P(r,r)*A_hat_robot' A_hat_robot*P(r,states_obseved(4:end));
%                 (A_hat_robot*P(r,states_obseved(4:end)))'  P(states_obseved(4:end),states_obseved(4:end))]; %P- calcualtion
    P(r,r) = A_hat_robot*P(r,r)*A_hat_robot';
    P(r,states_obseved(4:end)) = A_hat_robot*P(r,states_obseved(4:end));
    P(states_obseved(4:end),r) = P(r,states_obseved(4:end))';
    %%%EKF ESTIMATION
    %landmark observation
    for i = landmark_obsved
        Y(:,i)= landmarks_obsv(X_expect,L(:,i))+ v.*randn(2,1);
    end
    for i = landmark_obsved
% % %         L_index = 3+(i-1)*2+1:3+(i-1)*2+2;
% % %         states_temp = [r,L_index];
% % %         Y_i = Y(:,i);  %i_th landmark obsv
% % %         [h,C_shrink] = landmark_estimate(X(r),X(L_index)); %landmark estimation and its jacobian
% % %         F_x = zeros(5,3+2*landmark_number); % transformation matrix, for less calculation
% % %         F_x(r,r) = eye(3);
% % %         F_x(4:5,L_index) = eye(2);
% % %         C = C_shrink*F_x;
% % %         K = P*C'*inv(C*P*C'+V); %feedback calucaltion
% % %         E_y = Y_i-h; %output error
% % %         E_y(2) = wrapToPi(E_y(2));
% % %         tp = X + K*E_y;
% % %         if abs(tp(1)) > 1.3*abs(X(1))
% % %             a = 1
% % %         end        
% % %         if sum(tp(states_temp)) > 1.3*sum(abs(X(states_temp)))
% % %             a = 1
% % %         end
% % %         X = X + K*E_y;  % state estimation
% % %         X(3) = wrapToPi(X(3));
% % %         P = (eye(3+2*landmark_number)-K*C)*P; %P update
% % %         L_estimate(:,i) = X(L_index);
% % %         L_EKF_free(:,i) = inverse_landmark_obsv(X(r), Y_i);
        L_index = 3+(i-1)*2+1:3+(i-1)*2+2;
        states_temp = [r,L_index];
        Y_i = Y(:,i);  %i_th landmark obsv
        [h,C] = landmark_estimate(X(r),X(L_index)); %landmark estimation and its jacobian
        E = C*P(states_temp,states_temp)*C';
        E_y = Y_i-h; %output error
        E_y(2) = wrapToPi(E_y(2));
        Z = E+V;
        
%         if E_y'*inv(Z)*E_y <9
            K = P(states_obseved,states_temp)*C'*inv(Z); %feedback calucaltion
            X(states_obseved) = X(states_obseved) + K*E_y;  % state estimation
            X(3) = wrapToPi(X(3));
            P(states_obseved,states_obseved) = P(states_obseved,states_obseved) - K*Z*K';%(eye(5)-K*C)*P(states_temp,states_temp); %P update
%         end
        L_estimate(:,i) = X(L_index);
    end
    
    %%%plots update
    Tri_shape_est = polar2cart(X(r),Tri_shape0);
    Tri_shape_expect = polar2cart(X_expect,Tri_shape0);
    robot_error(:,j) = robot_mse(X(r),X_expect);
    set(robot_plot,'xdata',Tri_shape_est(1,:),'ydata',Tri_shape_est(2,:));
    set(robot_expect_plot,'xdata',Tri_shape_expect(1,:),'ydata',Tri_shape_expect(2,:));
    set(landmark_real_plot,'xdata',L(1,:),'ydata',L(2,:));
    set(landmark_estimated_plot,'xdata',L_estimate(1,:),'ydata',L_estimate(2,:));
    set(landmark_ekf_free_plot,'xdata',L_EKF_free(1,:),'ydata',L_EKF_free(2,:));
    title('SLAM-EKF simulation')
    % for error calculation
    landmark_error(:,j) = landmark_sum_mse(L(:,landmark_obsved),L_estimate(:,landmark_obsved));
    drawnow
end
figure()
subplot(2,1,1)
plot(landmark_error(1,:),'color','b');
title('landmark error for x direction')
xlabel('iteration')
ylabel('sum error square')
subplot(2,1,2)
plot(landmark_error(2,:),'color','b');
title('landmark error for y direction')
xlabel('iteration')
ylabel('sum error square')
figure()
subplot(3,1,1)
plot(robot_error(1,:),'color','b'); 
title('robot error x')
subplot(3,1,2)
plot(robot_error(2,:),'color','b'); 
title('robot error y')
subplot(3,1,3)
plot(abs(robot_error(3,:)),'color','b');
title('robot error theta')