clc;clear;close all
%SLAM-EKF testing bench
iteration = 200;
loop = 1;
%%%%%%%%%%%INITIALIZATION
%%%TIME INTERVAL INIT
dt = 0.1;

%%%ROBOT STATES
states_ekf = zeros(iteration,3);
states_ukf = zeros(iteration,3);
states_expect = zeros(iteration,3);
%%%LANDMARKS INIT
landmark_number = 20; %number of landmarks
map_length = 10; %map size
figure(1); % create figure
cla % clear axes
axis([-map_length map_length -map_length map_length]) % set axes limits
axis square
%%%LANDMARKS GENERATION
random_landmark = true;%true; %true for random_landmark, false for fixed landmark
L = landmarks_generate(map_length,landmark_number,random_landmark); %landmark location are generated,size of(2,landmark_number)
% L_estimate_use_CI_sigma = zeros(2,landmark_number);
% L_estimate = L_estimate_use_CI_sigma;
L_filter_free = zeros(2,landmark_number);

%%%Constraints init
constraints = struct;
constraints.edges = zeros(iteration,2);
constraints.z = zeros(iteration,1);

%%%ODOMETRY approx
odometry = struct;
odometry.odom = zeros(iteration,3);
odometry.unfiltered_pose = zeros(iteration,3);

%%%NOISE COV
q = 3*[.01;pi/500]; % control noise
Q = diag(q.^2);

v = 2*[.2;1*pi/180];% measurement noise
V = diag(v.^2);
%%%ROBOT INTPUT
u_v = 1; %line speed
u_w = pi/10; %angular velocity
u = [u_v ; u_w]; %inputs
u_n = [u_v ; u_w]; %noised inputs
%%% UKF parameter
alpha = 0.55; 
kappa = 5;
beta = 2; %influence the Weight of mean covariance
alpha2 = 0.45; 
kappa2 = 10;
beta2 = 2; %influence the Weight of mean covariance
sigma_param = struct('alpha', alpha,'kappa',kappa,'beta',beta);
sigma_param2 = struct('alpha', alpha2,'kappa',kappa2,'beta',beta2);


%%%%%%%%%%%PLOT SETUP
landmark_error_ekf = zeros(2,iteration);
landmark_error_eukf = zeros(2,iteration);
robot_error_ekf = zeros(3,iteration);
robot_error_eukf= zeros(3,iteration);

landmark_error_ekf_cumulate = zeros(2,iteration);
landmark_error_eukf_cumulate = zeros(2,iteration);
robot_error_ekf_cumulate = zeros(3,iteration);
robot_error_eukf_cumulate= zeros(3,iteration);



for loop_i = 1:loop
loop_i
%%%ROBOT INIT
X_Robot = zeros(3,1); %robot states init, (x,y,theta)
% X_Robot = [-2;-2;0]
X_Landmark = zeros(2*landmark_number,1); %landmarks states init, L_x and L_y
%%%THE MAP INIT
X1 = [X_Robot ; X_Landmark]; %the map states init
X2 = X1;
%robot states index
r = 1:3;
state_number = 3+2*landmark_number;
P1 = zeros(state_number,state_number);
P1(r,r) = 0.1*diag([1 1 1]);
P2 = 1*diag([1 1 1]);

%observed landmarks index
landmark_obsved = [];
Y = zeros(2,landmark_number); %landmark observation
Y_v = zeros(2*landmark_number,1);

% first_obs = true;
%%%expected trajectory
X_expect = X_Robot;
Tri_shape0 = [0.4 0.1 0.1 0.4;
             0 pi*2/3 -pi*2/3 0]; %polar value [r phi] for triangle shape
Tri_shape_expect = polar2cart(X_expect,Tri_shape0);

%%%robot plts
robot_expect_plot =  plt_init(Tri_shape_expect,'r','-','none');
robot_plot_EKF =  plt_init(Tri_shape_expect,'b','-','none');
robot_plot_EUKF =  plt_init(Tri_shape_expect,'k','-','none');
%%%landmark plts
landmark_plot_real = plt_init(L,'r','none','+');
landmark_plot_EKF = plt_init(L,'b','none','+');
landmark_plot_EUKF = plt_init(L,'k','none','+');
landmark_filter_free = plt_init(L,'g','none','+');
%observe sequence generate
obsv_sequence = randperm(landmark_number,landmark_number);
states_obsved = [1 2 3];    

for j = 1:iteration
    lm_left = find(obsv_sequence);
    if lm_left 
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
        
        %%%EKF add new landmark
%           [X1, P1] = UKF_newLM(X_expect,X1,P1,V,L_obsv,states_obsved,L_index,sigma_param);
         [X1, P1] = EKF_newLM(X_expect,X1,P1,V,L_obsv,states_obsved,L_index);
         [X2, P2] = UKF_newLM(X_expect,X2,P2,V,L_obsv,states_obsved,L_index,sigma_param);
%          [X2, P2] = EKF_newLM(X_expect,X2,P2,V,L_obsv,states_obsved,L_index);
    end
    %expect trajectory, for plot
    X_expect = robot_motion(X_expect,u,dt); 
    X_expect(3) = wrapToPi(X_expect(3));

    %control input noise
    p_noise = q.*randn(2,1);
    u_n(1)=u_v+p_noise(1);
    u_n(2)=u_w+p_noise(2); 
    
    %unfiltered pose
    if(j >1)
        odometry.unfiltered_pose(j,r) = robot_motion((odometry.unfiltered_pose(j-1,r))',u_n,dt);
        odometry.unfiltered_pose(j,3) = wrapToPi(odometry.unfiltered_pose(j,3));
    end
    %landmark measurment noise added
    for i = landmark_obsved
        M = landmarks_obsv(X_expect,L(:,i))+ v.*randn(2,1);
        if M(1) <0
            M(1) = 0;
        end
        Y(:,i) = M;
        Y_v(2*(i-1)+1:2*(i-1)+2) = M;
    end
    
    %%%%%%%%EKF Prediction
      [X1,P1] = EKF_prediction(X1,P1,u_n,dt,states_obsved);
%         [X1,P1] = UKF_prediction_original(X1,P1,u_n,Q,dt,sigma_param,states_obsved,0);
%          [X1,P1] = UKF_prediction(X1,P1,u_n,Q,dt,sigma_param,states_obsved,0);
         [X2,P2] = UKF_prediction(X2,P2,u_n,Q,dt,sigma_param,states_obsved,0);
%         [X2,P2] = EKF_prediction(X2,P2,u_n,dt,states_obsved);
        %%%%%%%%EKF update
        [X1,P1,L_ekf] = EKF_update(X1,P1,Y,V,landmark_obsved,landmark_number);
%           [X1,P1,L_ekf] = UKF_update2(X1,P1,Y_v,v,sigma_param,landmark_obsved,landmark_number,states_obsved);
%               [X1,P1,L_ekf] = UKF_update(X1,P1,Y_v,v,sigma_param,landmark_obsved,landmark_number,states_obsved);
%         [X2,P2,L_eukf] = EKF_update(X2,P2,Y,V,landmark_obsved,landmark_number);
         [X2,P2,L_eukf] = UKF_update2(X2,P2,Y_v,v,sigma_param,landmark_obsved,landmark_number,states_obsved);

    %%%states update
    states_ekf(j,r) = X1(r);
    states_ukf(j,r) = X2(r);
    states_expect(j,r) = X_expect;
    for i = landmark_obsved
        L_filter_free(:,i) = inverse_landmark_obsv((odometry.unfiltered_pose(j,r))', Y(:,i));
    end
    %%%robot plot update
    robot_plot_update(X_expect,Tri_shape0,robot_expect_plot)
    robot_plot_update(X1(r),Tri_shape0,robot_plot_EKF)
    robot_plot_update(X2(r),Tri_shape0,robot_plot_EUKF)
    %%%landmarks plot update
%     landmark_plot_update(L,landmark_plot_real)
    landmark_plot_update(L_ekf,landmark_plot_EKF)
    landmark_plot_update(L_eukf,landmark_plot_EUKF)
    landmark_plot_update(L_filter_free,landmark_filter_free)
    
    %%%robot location mean square error
    robot_error_ekf(:,j) = robot_mse(X1(r),X_expect);
    robot_error_eukf(:,j) = robot_mse(X2(r),X_expect);
    %%%landmark location mean square error
    landmark_error_ekf(:,j) = landmark_sum_mse(L(:,landmark_obsved),L_ekf(:,landmark_obsved));
    landmark_error_eukf(:,j) = landmark_sum_mse(L(:,landmark_obsved),L_eukf(:,landmark_obsved));
    drawnow
end

% figure()
% subplot(2,1,1)
% plot(landmark_error_ekf(1,:),'color','b'); hold on;
% plot(landmark_error_eukf(1,:),'color','k'); hold off;
% title('landmark error for x direction')
% xlabel('iteration')
% ylabel('sum error square')
% subplot(2,1,2)
% plot(landmark_error_ekf(2,:),'color','b');  hold on;
% plot(landmark_error_eukf(2,:),'color','k');  hold off;
% title('landmark error for y direction')
% xlabel('iteration')
% ylabel('sum error square')
% 
% 
% figure()
% subplot(3,1,1)
% plot(robot_error_ekf(1,:),'color','b');  hold on;
% plot(robot_error_eukf(1,:),'color','k');  hold off;
% title('robot error x')
% subplot(3,1,2)
% plot(robot_error_ekf(2,:),'color','b');  hold on;
% plot(robot_error_eukf(2,:),'color','k');  hold off;
% subplot(3,1,3)
% plot(abs(robot_error_ekf(3,:)),'color','b');  hold on;
% plot(abs(robot_error_eukf(3,:)),'color','k');  hold off;

landmark_error_ekf_cumulate = landmark_error_ekf_cumulate +landmark_error_ekf;
landmark_error_eukf_cumulate = landmark_error_eukf_cumulate +landmark_error_eukf;
robot_error_ekf_cumulate = robot_error_ekf_cumulate + robot_error_ekf;
robot_error_eukf_cumulate = robot_error_eukf_cumulate + robot_error_eukf;
end
landmark_error_ekf_cumulate = landmark_error_ekf_cumulate/loop;
landmark_error_eukf_cumulate = landmark_error_eukf_cumulate/loop;
robot_error_ekf_cumulate = robot_error_ekf_cumulate/loop;
robot_error_eukf_cumulate = robot_error_eukf_cumulate/loop;

figure()
subplot(2,1,1)
plot(landmark_error_ekf_cumulate(1,:),'color','b'); hold on;
plot(landmark_error_eukf_cumulate(1,:),'color','k'); hold off;
title('landmark error for x direction')
xlabel('iteration')
ylabel('sum error square')
subplot(2,1,2)
plot(landmark_error_ekf_cumulate(2,:),'color','b');  hold on;
plot(landmark_error_eukf_cumulate(2,:),'color','k');  hold off;
title('landmark error for y direction')
xlabel('iteration')
ylabel('sum error square')


figure()
subplot(3,1,1)
plot(robot_error_ekf_cumulate(1,:),'color','b');  hold on;
plot(robot_error_eukf_cumulate(1,:),'color','k');  hold off;
title('robot error x')
subplot(3,1,2)
plot(robot_error_ekf_cumulate(2,:),'color','b');  hold on;
plot(robot_error_eukf_cumulate(2,:),'color','k');  hold off;
title('robot error y')
subplot(3,1,3)
plot(abs(robot_error_ekf_cumulate(3,:)),'color','b');  hold on;
plot(abs(robot_error_eukf_cumulate(3,:)),'color','k');  hold off;
title('robot error theta')

%trajectory plot
figure()
plot(states_ekf(:,1),states_ekf(:,2),'b'); hold on;
plot(states_ukf(:,1),states_ukf(:,2),'k'); hold on;
plot(odometry.unfiltered_pose(:,1),odometry.unfiltered_pose(:,2),'g'); hold on;
plot(states_expect(:,1),states_expect(:,2),'r'); hold off;

