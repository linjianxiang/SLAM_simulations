%%%%%%%READ ME%%%%%%%%%
%%%variables%%%
% %iteration : number of robot move steps
% %loop: number of experiments
% %dt: simulated discrete time interval
% %robot_full_states: Includes all robot states for testings
% %map: contains map information
% %observations: 
%               V: measurement noise covariance matrix
%               landmark_obsved: observed landmark sequence
%               Y: landmark measurement, noised
%               states_obsved: observed states, includes robot states and
%                                observed landmark states
%               L_index: observed landmark index, for update step of the kf
% %sigma_param: includes parameters for UKF
% %robot_info:
%               odom: noise free motion, which can represente the noised
%                       odometry
%               true_pose: robot true position, which is influenced by noises
% %errors: contains errors for estimated robot and landmark locations

%%%%%%%%%%%%%%%%%%%%%%%%



clc;clear;close all

%robot distance between wheels
global L;
L = 1;

%SLAM-EKF testing bench
iteration = 200;
loop = 1;
%%%%%%%%%%%INITIALIZATION
%%%TIME INTERVAL INIT
dt = 1;

%%%ROBOT STATES
robot_full_states = struct;
robot_full_states.states_testing1 = zeros(iteration,3);
robot_full_states.states_testing2 = zeros(iteration,3);
%%%LANDMARKS INIT
map = struct;
map.landmark_number = 20;  %number of landmarks
map.map_length = 10; %map size
map.random_landmark = true;
figure(1); % create figure
cla % clear axes
axis([-map.map_length map.map_length -map.map_length map.map_length]) % set axes limits
axis square
%%%LANDMARKS GENERATION
map.Landmark_location = landmarks_generate(map); %landmark location are generated,size of(2,map.landmark_number)
% L_estimate_use_CI_sigma = zeros(2,map.landmark_number);
% L_estimate = L_estimate_use_CI_sigma;
map.L_filter_free = zeros(2,map.landmark_number);


%%%ODOMETRY approx
robot_info = struct;
robot_info.odom = zeros(3,iteration); %noise free pose, we can see this as noised measurement
robot_info.true_pose = zeros(3,iteration);

%%%NOISE COV
q = 2*[.01;pi/180]; % control noise

v = 5*[.2;1*pi/180];% measurement noise

%%%OBSERVATION
observations = struct;
% observations.newlandmarks = struct;
observations.V = diag(v.^2); %measurement noise
%%%ROBOT INTPUT
control_input = struct;
u_v = 0.1; %line speed
u_w = pi/20; %angular velocity
control_input.u = [u_v ; u_w]; %inputs
control_input.u_noised = [u_v ; u_w]; %noised inputs
control_input.Q = diag(q.^2);
%%% UKF parameter
sigma_param = struct;
sigma_param.alpha = 0.55; 
sigma_param.kappa = 5;
sigma_param.beta = 2; %influence the Weight of mean covariance
sigma_param2 = struct;
sigma_param2.alpha = 0.45; 
sigma_param2.kappa = 10;
sigma_param2.beta = 2; %influence the Weight of mean covariance

%%%testing states 
X_Landmark = zeros(2*map.landmark_number,1); %landmarks states init, L_x and L_y
state_number = 3+2*map.landmark_number;
testing1 = struct;
testing2 = struct;
testing1.states = zeros(state_number,1);
testing2.states = zeros(state_number,1);
testing1.P = zeros(state_number,state_number);
testing2.P = zeros(state_number,state_number);
%%%%%%%%%%%PLOT SETUP
errors = struct; 

%error for each iteration
errors.landmark_error_testing1 = zeros(2,iteration); 
errors.landmark_error_testing2 = zeros(2,iteration);
errors.robot_error_testing1 = zeros(3,iteration);
errors.robot_error_testing2= zeros(3,iteration);
%sum of error for each iteration
errors.landmark_error_cumulate_testing1 = zeros(2,iteration);
errors.landmark_error_cumulate_testing2 = zeros(2,iteration);
errors.robot_error_cumulate_testing1 = zeros(3,iteration);
errors.robot_error_cumulate_testing2= zeros(3,iteration);

%loop i is for calculating average error
for loop_i = 1:loop
loop_i
%%%ROBOT INIT
X_Robot = [0 0 0]'; %robot states init, (x,y,theta)
% X_Robot = [-2;-2;0]

%%%THE MAP INIT
testing1.states = [X_Robot ; X_Landmark]; %the map states init
testing2.states = [X_Robot ; X_Landmark];
%robot states index
r = 1:3;

testing1.P = zeros(state_number,state_number);
testing1.P(r,r) = 0.01*diag([1 1 0.1]);
% testing2.P = 1*diag([1 1 1]);
testing2.P = zeros(state_number,state_number);
testing2.P(r,r) = 1*diag([1 1 0.1]);
%observed landmarks index
observations.landmark_obsved = [];
observations.Y = zeros(2,map.landmark_number); %landmark observation

% first_obs = true;
plots = struct;
temp = sqrt(2*L^2);
Tri_shape0 = [temp temp L L temp temp L;
             0.25*pi 0.75*pi 0.5*pi -0.5*pi -0.75*pi -0.25*pi -0.5*pi]; %polar value [r phi] for triangle shape
plots.Tri_shape_expect = polar2cart(X_Robot,Tri_shape0);

%%%robot plts
robot_plots.robot_expect_plot =  plt_init(plots.Tri_shape_expect,'r','-','none');
robot_plots.robot_plot_testing1 =  plt_init(plots.Tri_shape_expect,'b','-','none');
robot_plots.robot_plot_testing1_cov =  plt_init(X_Robot(1:2),'b','-','none');
robot_plots.robot_plot_testing2 =  plt_init(plots.Tri_shape_expect,'k','-','none');
robot_plots.robot_plot_testing2_cov =  plt_init(X_Robot(1:2),'k','-','none');
%%%landmark plts

landmark_plots.landmark_plot_real = plt_init(map.Landmark_location,'r','none','+');
landmark_plots.landmark_plot_testing1 = plt_init(map.Landmark_location,'b','none','+');
landmark_plots.landmark_plot_testing2 = plt_init(map.Landmark_location,'k','none','+');

for i = 1:map.landmark_number
    landmark_plots.landmark_plot_testing1_cov(i) = plt_init(map.Landmark_location(:,i),'b','-','none');
    landmark_plots.landmark_plot_testing2_cov(i) = plt_init(map.Landmark_location(:,i),'k','-','none');    
end

landmark_plots.landmark_filter_free = plt_init(map.Landmark_location,'g','none','+');
%observe sequence generate
obsv_sequence = randperm(map.landmark_number,map.landmark_number);
observations.states_obsved = [1 2 3];    

for j = 2:iteration
    lm_left = find(obsv_sequence);
    if lm_left 
        lm_number = obsv_sequence(lm_left(1));
        obsv_sequence(lm_left(1)) = 0;
        %increase states
        observations.L_index = 3+2*(lm_number-1)+1:3+2*(lm_number-1)+2;
        %robot states indices and observed landmark indeics
        observations.states_obsved = [observations.states_obsved, observations.L_index];
        %observed landmark indices
        observations.landmark_obsved = [observations.landmark_obsved,lm_number];
        %observe the current landmark range and angle [r phi] 
        L_obsv= landmarks_obsv(robot_info.true_pose(r,j),map.Landmark_location(:,lm_number))+ v.*randn(2,1);
        
        %%%EKF add new landmark
%           [testing1.states, testing1.P] = UKF_newLM(robot_info.true_pose(r,j),testing1,observations,L_obsv,sigma_param);
         [testing1.states, testing1.P] = EKF_newLM(robot_info.true_pose(r,j),testing1,observations,L_obsv);
         [testing2.states, testing2.P] = UKF_newLM(robot_info.true_pose(r,j),testing2,observations,L_obsv,sigma_param);
%          [testing2.states, testing2.P] = EKF_newLM(robot_info.true_pose(r,j),testing2,observations,L_obsv);;
    end

    %control input noise
    p_noise = q.*randn(2,1);
    control_input.u_noised(1)=u_v+p_noise(1);
    control_input.u_noised(2)=u_w+p_noise(2); 
    

    %true pose
    robot_info.true_pose(r,j) = robot_motion(robot_info.true_pose(r,j-1),control_input.u_noised,dt);
    robot_info.true_pose(3,j) = wrapToPi(robot_info.true_pose(3,j));

    %noise free pose, we can see this as noised measurement
    robot_info.odom(r,j) = robot_motion(robot_info.odom(r,j-1),control_input.u,dt);
    robot_info.odom(3,j) = wrapToPi(robot_info.odom(3,j));

    %landmark measurment noise added
    for i = observations.landmark_obsved
        M = landmarks_obsv(robot_info.true_pose(r,j),map.Landmark_location(:,i))+ v.*randn(2,1);
        if M(1) <0
            M(1) = 0;
        end
        observations.Y(:,i) = M;
    end
    
    %%%%%%%%EKF Prediction
      [testing1.states,testing1.P] = EKF_prediction(testing1,observations,control_input.u,dt);
%         [testing2.states,testing2.P] = UKF_prediction_original(testing2,observations,control_input,dt,sigma_param,0);
%          [testing1.states,testing1.P] = UKF_prediction(testing1,observations,control_input,dt,sigma_param,0);
         [testing2.states,testing2.P] = UKF_prediction(testing2,observations,control_input,dt,sigma_param,0);
%         [testing2.states,testing2.P] = EKF_prediction(testing2,observations,control_input.u,dt);
        %%%%%%%%EKF update
        [testing1.states,testing1.P,testing1.L_estimate_location] = EKF_update(testing1,observations,map.landmark_number);
%           [testing1.states,testing1.P,testing1.L_estimate_location] = UKF_update(testing1,observations,map.landmark_number,sigma_param);
%          [testing2.states,testing2.P,testing2.L_estimate_location] = UKF_update_testing(testing2,observations,map.landmark_number,sigma_param);
%         [testing2.states,testing2.P,testing2.L_estimate_location] = EKF_update(testing2,observations,map.landmark_number);
          [testing2.states,testing2.P,testing2.L_estimate_location] = UKF_update(testing2,observations,map.landmark_number,sigma_param);

    %%%states update
    robot_full_states.states_testing1(j,r) = testing1.states(r);
    robot_full_states.states_testing2(j,r) = testing2.states(r);

    for i = observations.landmark_obsved 
        map.L_filter_free(:,i) = inverse_landmark_obsv(robot_info.true_pose(r,j), observations.Y(:,i));
    end
    
    
    %%%robot plot update
    robot_plot_update(robot_info.true_pose(r,j),Tri_shape0,robot_plots.robot_expect_plot)
    robot_plot_update(testing1.states(r),Tri_shape0,robot_plots.robot_plot_testing1)
    PlotEllipse(testing1.states(r),testing1.P(r,r),robot_plots.robot_plot_testing1_cov,3)
    
    robot_plot_update(testing2.states(r),Tri_shape0,robot_plots.robot_plot_testing2)
    PlotEllipse(testing2.states(r),testing2.P(r,r),robot_plots.robot_plot_testing2_cov,3)
    %%%landmarks plot update
%     landmark_plot_update(L,landmark_plots.landmark_plot_real)
    landmark_plot_update(testing1.L_estimate_location,landmark_plots.landmark_plot_testing1)
    landmark_plot_update(testing2.L_estimate_location,landmark_plots.landmark_plot_testing2)
    PlotLandmarkEllipse(testing1.L_estimate_location,testing1.P,landmark_plots.landmark_plot_testing1_cov,3)
    PlotLandmarkEllipse(testing2.L_estimate_location,testing2.P,landmark_plots.landmark_plot_testing2_cov,3)
    landmark_plot_update(map.L_filter_free,landmark_plots.landmark_filter_free)
    
    %%%robot location mean square error
    errors.robot_error_testing1(:,j) = robot_mse(testing1.states(r),robot_info.true_pose(r,j));
    errors.robot_error_testing2(:,j) = robot_mse(testing2.states(r),robot_info.true_pose(r,j));
    %%%landmark location mean square error
    errors.landmark_error_testing1(:,j) = landmark_sum_mse(map.Landmark_location(:,observations.landmark_obsved),testing1.L_estimate_location(:,observations.landmark_obsved));
    errors.landmark_error_testing2(:,j) = landmark_sum_mse(map.Landmark_location(:,observations.landmark_obsved),testing2.L_estimate_location(:,observations.landmark_obsved));
    drawnow
end


errors.landmark_error_cumulate_testing1 = errors.landmark_error_cumulate_testing1 +errors.landmark_error_testing1;
errors.landmark_error_cumulate_testing2 = errors.landmark_error_cumulate_testing2 +errors.landmark_error_testing2;
errors.robot_error_cumulate_testing1 = errors.robot_error_cumulate_testing1 + errors.robot_error_testing1;
errors.robot_error_cumulate_testing2 = errors.robot_error_cumulate_testing2 + errors.robot_error_testing2;
end
errors.landmark_error_cumulate_testing1 = errors.landmark_error_cumulate_testing1/loop;
errors.landmark_error_cumulate_testing2 = errors.landmark_error_cumulate_testing2/loop;
errors.robot_error_cumulate_testing1 = errors.robot_error_cumulate_testing1/loop;
errors.robot_error_cumulate_testing2 = errors.robot_error_cumulate_testing2/loop;

figure()
subplot(2,1,1)
plot(errors.landmark_error_cumulate_testing1(1,:),'color','b'); hold on;
plot(errors.landmark_error_cumulate_testing2(1,:),'color','k'); hold off;
title('landmark error for x direction')
xlabel('iteration')
ylabel('sum error square')
subplot(2,1,2)
plot(errors.landmark_error_cumulate_testing1(2,:),'color','b');  hold on;
plot(errors.landmark_error_cumulate_testing2(2,:),'color','k');  hold off;
title('landmark error for y direction')
xlabel('iteration')
ylabel('sum error square')


figure()
subplot(3,1,1)
plot(errors.robot_error_cumulate_testing1(1,:),'color','b');  hold on;
plot(errors.robot_error_cumulate_testing2(1,:),'color','k');  hold off;
title('robot error x')
subplot(3,1,2)
plot(errors.robot_error_cumulate_testing1(2,:),'color','b');  hold on;
plot(errors.robot_error_cumulate_testing2(2,:),'color','k');  hold off;
title('robot error y')
subplot(3,1,3)
plot(abs(errors.robot_error_cumulate_testing1(3,:)),'color','b');  hold on;
plot(abs(errors.robot_error_cumulate_testing2(3,:)),'color','k');  hold off;
title('robot error theta')

%trajectory plot
figure()
plot(robot_full_states.states_testing1(:,1),robot_full_states.states_testing1(:,2),'b'); hold on;
plot(robot_full_states.states_testing2(:,1),robot_full_states.states_testing2(:,2),'k'); hold on;
plot(robot_info.odom(1,:),robot_info.odom(2,:),'g'); hold on;
plot(robot_info.true_pose(1,:),robot_info.true_pose(2,:),'r'); hold off;


%% graph based optimization 
%%%Constraints init
constraints = struct;
constraints.pose_pose_edges = zeros(iteration,2);
constraints.z = zeros(iteration,3);
optimization_iteration = 10;
odom = robot_info.odom;
loop_constriant_number = size(robot_info.true_pose,2);
c = zeros(optimization_iteration+1,1);
% %construct constraints for adjacent pose-pose
for i = 1:(loop_constriant_number-1)
    constraints.pose_pose_edges(i,:) = [i,i+1];
	pose_i = robot_full_states.states_testing1(i,:);
    pose_j = robot_full_states.states_testing1(i+1,:);
    constraints.z(i,:) = t2v(v2t(pose_i)\v2t(pose_j));
end
% manually add a loop closure constraint 
manual_add_number = 20; % manually added constrains
for i = 1: manual_add_number

% %approach1 , randomly choose two nodes and add true distance to constraint.z
%     constraint_left = randi([1,iteration]);
%     constraint_right = randi([1,iteration]);
%     constraints.pose_pose_edges(loop_constriant_number+i-1,:) = [constraint_left,constraint_right];
%     pose_i = robot_info.true_pose(:,constraint_left)';
%     pose_j = robot_full_states.states_testing1(constraint_right,:);
%     constraints.z(loop_constriant_number+i-1,:) = t2v(v2t(pose_i)\v2t(pose_j));

% % approach 2, sequently add constraints, 200-1 199-2 198-3
%     constraints.pose_pose_edges(loop_constriant_number+i-1,:) = [loop_constriant_number-i+1,i];
%     pose_i = robot_info.true_pose(:,loop_constriant_number-i+1)';
%     pose_j = robot_full_states.states_testing1(i,:);
%     constraints.z(loop_constriant_number+i-1,:) = t2v(v2t(pose_i)\v2t(pose_j));

% % apprach 3, add constraints between nodes and first node, 
% % % like nodes vs sub mpas, 200-1 199-1 198-1
%     constraints.pose_pose_edges(loop_constriant_number+i-1,:) = [loop_constriant_number-i+1,1];
%     pose_i = robot_info.true_pose(:,loop_constriant_number-i+1)';
%     pose_j = robot_full_states.states_testing1(1,:);
%     constraints.z(loop_constriant_number+i-1,:) = t2v(v2t(pose_i)\v2t(pose_j));

% % approach 4, randommly add constrains between nodes and first node
% %  like 5-1 175-1 33-1 ==> very good...
    constraint_left = randi([1,iteration]);
    constraints.pose_pose_edges(loop_constriant_number+i-1,:) = [constraint_left,1];
    pose_i = robot_info.true_pose(:,constraint_left)';
    pose_j = robot_full_states.states_testing1(1,:);
    constraints.z(loop_constriant_number+i-1,:) = t2v(v2t(pose_i)\v2t(pose_j));
end
%
figure()
plot(robot_full_states.states_testing1(:,1),robot_full_states.states_testing1(:,2),'b'); hold on;
plot(robot_full_states.states_testing2(:,1),robot_full_states.states_testing2(:,2),'k'); hold on;
plot(odom(1,:),odom(2,:),'g'); hold on;
plot(robot_info.true_pose(1,:),robot_info.true_pose(2,:),'r'); hold on;

%calcualate total error
c(1) = error_calculate(constraints, odom);
for i = 1:optimization_iteration
%optimization
dx = LS_optimization(constraints,odom);
dx = reshape(dx,[3,iteration]);
%new pose
odom =  odom + dx;
% new_pose =  new_pose + dx;
%error calculate
c(i+1) = error_calculate(constraints, odom);
display([' The constraint error converged from ',num2str(c(i)),'to ',num2str(c(i+1))]);
end

plot(odom(1,:),odom(2,:),'m'); hold off;
legend('ekf location','ukf location','pure odometry','true robot location','optimized location');
title(['with ' , num2str(iteration), ' number of steps']);

figure()
plot(c);
title('global cost');
xlabel('optimization steps');
ylabel('global cost');