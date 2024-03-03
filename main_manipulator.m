clear;
clc;
close all;

%% Predefine environment and load data
% load motion_data;
load('q2.mat');
load('q4.mat');

[x_ee, y_ee] = ForwardKinematics(q2, q4);
% x is position vection.
% x should be a row vector.
% x has a fixed sampling rate.
% x should have one min and max.
% x contains one period of desired motion.

% the first   vector is time
% the scond   vector is theta        look-up-table (for controller design)
% the third   vector is radius       look-up-table (for controller design)
% the fourth  vector is acceleration look-up-table (for controller design)
% the fifth   vector is pitch        look-up-table (for controller design)
% the sixth   vector is the fitted position  (to check quality of fitting)
% the seventh vector is the fitted velocity  (to check quality of fitting)

res      = 10000;    % res is resolution of the output look-up-table
T        = 1;        % T is period of x in second
sm_param = 0.999999; % sm_param is SmoothingSpline fitting parameter the higher the more bias

[~   ,tt,Radius_1,Acceleration_1,Pitch_1,X1,V1,Center_1,Transformation_1,S1] ...
      = ver_toolbox(q2,T,res,sm_param,1);


[~   ,~ ,Radius_2,Acceleration_2,Pitch_2,X2,V2,Center_2,Transformation_2,S2] ...
      = ver_toolbox(q4,T,res,sm_param,1);
%%
q0 = [-pi/4 -3*pi/4];
w0 = 0.*[(q2(2)-q2(1))/(T/length(q2)) (q4(2)-q4(1))/(T/length(q4))];
% w0 = [(q2(2)-q2(1))/(0.01), (q4(2)-q4(1))/(0.01)];
P  = 0*50;
% D  = 10;
D = 0; 
sr = 0.01;
en = 1;
%% Defining RL controller

Tf = 15;
Ts = 0.025;
% create observation specification
numObs =11;
obsInfo = rlNumericSpec([numObs 1]);
obsInfo.Name = 'observations';

% create action specification
numAct = 2;
actInfo = rlNumericSpec([numAct 1],'LowerLimit',-50,'UpperLimit',50);
actInfo.Name = 'foot_torque';


% Select and Create Agent for Training
AgentSelection = 'TD3';
switch AgentSelection
    case 'DDPG'
        agent = createDDPGAgent(numObs,obsInfo,numAct,actInfo,Ts);
    case 'TD3'
        agent = createTD3Agent(numObs,obsInfo,numAct,actInfo,Ts);
    otherwise
        disp('Enter DDPG or TD3 for AgentSelection')
end

% Specify Training Options and Train Agent
maxEpisodes = 5000;
maxSteps = floor(Tf/Ts);
trainOpts = rlTrainingOptions(...
    'MaxEpisodes',maxEpisodes,...
    'MaxStepsPerEpisode',maxSteps,...
    'ScoreAveragingWindowLength',250,...
    'Verbose',false,...
    'Plots','training-progress',...
    'StopTrainingCriteria','EpisodeCount',...
    'StopTrainingValue',maxEpisodes,...
    'SaveAgentCriteria','EpisodeCount',...
    'SaveAgentValue',maxEpisodes);

% trainOpts.UseParallel = true;
% trainOpts.ParallelizationOptions.Mode = 'async';
% trainOpts.ParallelizationOptions.StepsUntilDataIsSent = 32;
% trainOpts.ParallelizationOptions.DataToSendFromWorkers = 'Experiences';


%% Train the agent
% env = rlSimulinkEnv("panda", "panda/RL/RL Agent")
% trainingStats = train(agent,env,trainOpts);
%%

% Simulate Trained Agents
% rng(0)
% simOptions = rlSimulationOptions('MaxSteps',maxSteps);
% experience = sim(env,agent,simOptions);
% 
% comparePerformance('DDPGAgent','TD3Agent')


sim('panda');
% Limit Cycle 1
%
%%
close all;

figure
plot(pos1,vel1,'k','linewidth',1)
title('Desired Limit Cycle 1')
xlabel('Position [m]')
ylabel('Velocity [m/s]')
pbaspect([1 1 1])
hold on;
plot(X1,V1,'g','linewidth',2)
plot(q0(1),w0(1),'ro')
plot(Center_1,0,'ro')
grid on;

figure
plot(pos2,vel2,'k','linewidth',1)
title('Desired Limit Cycle 2')
xlabel('Position [m]')
ylabel('V [m/s]')
pbaspect([1 1 1])
hold on;
plot(X2,V2,'g','linewidth',2)
plot(q0(2),w0(2),'ro')
plot(Center_2,0,'ro')
grid on;


figure
title('Pitch Objective')
plot(time,pitch1)
hold on
plot(time,pitch2)
xlabel('Time [s]')
ylabel('Pitch')
legend({'A','B'})
grid on;

figure
plot(pos_ee(1:end,1),pos_ee(1:end,2),'k')
title('End Effector')
xlabel('x')
ylabel('y')
grid on;
pbaspect([1 1 1])
% xlim([0.3 1.1])
% ylim([-0.5 0.5])

figure
title('Pitch Objective')
plot(time,pitch_diff)
xlabel('Time [s]')
ylabel('Pitch Error')
legend({'A','B'})
grid on;
% xlim([0 5*T]) 
%
figure
title('Energy Objective 1')
plot(time,energy_diff1)
xlabel('Time [s]')
ylabel('Energy Error 1')
grid on;
% xlim([0 5*T])

figure
title('Energy Objective 2')
plot(time,energy_diff2)
xlabel('Time [s]')
ylabel('Energy Error 2')
grid on;
xlim([0 5*T])

figure
sgtitle('First Limit Cycle')
subplot(2,2,1)
plot(X1,V1,'k','linewidth',1)
pbaspect([1 1 1])
grid on;
xlabel('Position')
ylabel('Velocity')
hold on;
plot(S1(1),S1(2),'ro')
subplot(2,2,2)
plot(tt,Radius_1,'k','linewidth',1)
pbaspect([1 1 1])
grid on;
xlabel('Phase')
ylabel('Radius')
subplot(2,2,3)
plot(tt,Acceleration_1,'k','linewidth',1)
pbaspect([1 1 1])
grid on;
xlabel('Phase')
ylabel('Acceleration')
subplot(2,2,4)
plot(tt,Pitch_1,'k','linewidth',1)
pbaspect([1 1 1])
grid on;
xlabel('Phase')
ylabel('Pitch')

figure
sgtitle('Second Limit Cycle')
subplot(2,2,1)
plot(X2,V2,'k','linewidth',1)
pbaspect([1 1 1])
grid on;
xlabel('Position')
ylabel('Velocity')
hold on;
plot(S2(1),S2(2),'ro')
subplot(2,2,2)
plot(tt,Radius_2,'k','linewidth',1)
pbaspect([1 1 1])
grid on;
xlabel('Phase')
ylabel('Radius')
subplot(2,2,3)
plot(tt,Acceleration_2,'k','linewidth',1)
pbaspect([1 1 1])
grid on;
xlabel('Phase')
ylabel('Acceleration')
subplot(2,2,4)
plot(tt,Pitch_2,'k','linewidth',1)
pbaspect([1 1 1])
grid on;
xlabel('Phase')
ylabel('Pitch')
