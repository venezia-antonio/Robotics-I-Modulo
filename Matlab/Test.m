clear all
clc
close all
trj = [];
Ts = 0.05; % Sampling time
Tend = 15*1e3;
time = 0;
i = 1;
%% Esempio legge orario joint1
t = 0:0.1:10-Ts;
w = 5;
trj1 = w*t;
%% Connection to CoppeliaSim
vrep = remApi('remoteApi');
vrep.simxFinish(-1); %Pulisce connessioni aperte
clientID = vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

if(clientID > -1)
    disp("Connessione Effettuata");
    vrep.simxAddStatusbarMessage(clientID,'Connessione da Matlab',vrep.simx_opmode_oneshot);%invio messaggio a vrep
    [~,endEffector_h] = vrep.simxGetObjectHandle(clientID,'endEffector',vrep.simx_opmode_blocking);
    [~,J1_h] = vrep.simxGetObjectHandle(clientID,'IRB140_joint1',vrep.simx_opmode_blocking);
    [~,J2_h] = vrep.simxGetObjectHandle(clientID,'IRB140_joint2',vrep.simx_opmode_blocking);
    [~,J3_h] = vrep.simxGetObjectHandle(clientID,'IRB140_joint3',vrep.simx_opmode_blocking);
    [~,J4_h] = vrep.simxGetObjectHandle(clientID,'IRB140_joint4',vrep.simx_opmode_blocking);
    [~,J5_h] = vrep.simxGetObjectHandle(clientID,'IRB140_joint5',vrep.simx_opmode_blocking);
    [~,J6_h] = vrep.simxGetObjectHandle(clientID,'IRB140_joint6',vrep.simx_opmode_blocking);
    % Start Simulation
    
    [~] = vrep.simxSynchronous(clientID,true);
    [~] = vrep.simxSynchronousTrigger(clientID);
    [~] = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);
    tic
    while(i<length(t))
        
        [~]=vrep.simxSynchronousTrigger(clientID);
        tic
        % Get Joint Position
        [~,t1] = vrep.simxGetJointPosition(clientID,J1_h,vrep.simx_opmode_blocking);
        [~,t2] = vrep.simxGetJointPosition(clientID,J2_h,vrep.simx_opmode_blocking);
        [~,t3] = vrep.simxGetJointPosition(clientID,J3_h,vrep.simx_opmode_blocking);
        [~,t4] = vrep.simxGetJointPosition(clientID,J4_h,vrep.simx_opmode_blocking);
        [~,t5] = vrep.simxGetJointPosition(clientID,J5_h,vrep.simx_opmode_blocking);
        [~,t6] = vrep.simxGetJointPosition(clientID,J6_h,vrep.simx_opmode_blocking);
        [~,endEffector] = vrep.simxGetObjectPosition(clientID,endEffector_h,-1,vrep.simx_opmode_blocking);
        trj(:,i) = endEffector;
        joint(i,:) = [t1 t2 t3 t4 t5 t6];
        % Set Joint Position
%        Kinetics Control - remeber to set the model in non-dynamics      
%         [~] = vrep.simxSetJointPosition(clientID,J1_h,trj1(i),vrep.simx_opmode_blocking); %lag di 20ms circa costanti      
%         [~] = vrep.simxSetJointPosition(clientID,J2_h,1,vrep.simx_opmode_blocking);
%         [~] = vrep.simxSetJointPosition(clientID,J3_h,1,vrep.simx_opmode_blocking);
%         [~] = vrep.simxSetJointPosition(clientID,J4_h,1,vrep.simx_opmode_blocking);
%         [~] = vrep.simxSetJointPosition(clientID,J5_h,1,vrep.simx_opmode_blocking);
%         [~] = vrep.simxSetJointPosition(clientID,J6_h,i,vrep.simx_opmode_blocking);

%        Dynamics control 
          [~] = vrep.simxSetJointTargetPosition(clientID,J1_h,trj1(i),vrep.simx_opmode_oneshot);
%         [~] = vrep.simxSetJointTargetPosition(clientID,J2_h,0.1*i,vrep.simx_opmode_oneshot);
%         [~] = vrep.simxSetJointTargetPosition(clientID,J3_h,pi/2,vrep.simx_opmode_oneshot);
%         [~] = vrep.simxSetJointTargetPosition(clientID,J4_h,pi/2,vrep.simx_opmode_oneshot);
%         [~] = vrep.simxSetJointTargetPosition(clientID,J5_h,pi/2,vrep.simx_opmode_oneshot);
%         [~] = vrep.simxSetJointTargetPosition(clientID,J6_h,pi,vrep.simx_opmode_oneshot);
        time = vrep.simxGetLastCmdTime(clientID);
        i = i+1;
        %pause(0.005)
        toc
    end
    toc
    [~] = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot);
    
    
else
    disp("Fail");
end
 %%
figure(1)
view(3)
hold on, grid on
title('End Effector Trajectory')
plot3(trj(1,:),trj(2,:),trj(3,:),'*--')
xlabel('X [m]');ylabel('Y [m]');zlabel('Z [m]')
xlim([-2 2]);ylim([-2 2]);zlim([0 2])
% Angoli tra -pi +pi
figure(2)
subplot(321)
hold on,grid on
title('\theta_{1}')
plot(joint(:,1));xlabel('time [s]');
subplot(322)
hold on,grid on
title('\theta_{2}')
plot(joint(:,2));xlabel('time [s]');
subplot(323)
hold on,grid on
title('\theta_{3}')
plot(joint(:,3));xlabel('time [s]');
subplot(324)
hold on,grid on
title('\theta_{4}')
plot(joint(:,4));xlabel('time [s]');
subplot(325)
hold on,grid on
title('\theta_{5}')
plot(joint(:,5));xlabel('time [s]');
subplot(326)
hold on,grid on
title('\theta_{6}')
plot(joint(:,6));xlabel('time [s]');










