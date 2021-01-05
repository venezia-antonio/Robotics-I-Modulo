%%clear all
% clc
% close all
% load Traiettoria
%% Esempio legge orario joint1
trj1 = result;
%% Connection to CoppeliaSim
vrep = remApi('remoteApi');
vrep.simxFinish(-1); %Pulisce connessioni aperte
clientID = vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

if(clientID > -1)
    
    disp("Connessione Effettuata");
    vrep.simxAddStatusbarMessage(clientID,'Connessione da Matlab',vrep.simx_opmode_oneshot);%invio messaggio a vrep
    % Get Joint handle
    J_h = getJointHandle(vrep,clientID); 
    % Get bottle handle and initial position
    bottle_h = getBottleHandle(vrep,clientID);
    %bottlePos = getBottlePosition(vrep,clientID,bottle_h);
    bottlePos = [-0.3454  -0.5003   0.3311]';
    % Get glass handle and position
    glass_h = getGlassHandle(vrep,clientID);
    glassPos = getGlassPosition(vrep,clientID,glass_h);
    % Close grip
    %%%%%%closeGrip(vrep,clientID,bottle_h);
    % Set initial joint position
    setInitialJointPosition(vrep,clientID,trj1,J_h)    
    % Start Simulation
    startSimulation(vrep,clientID)
    [~]=vrep.simxSynchronousTrigger(clientID);
    pause(4)
    
    for i = 1:length(trj1)      
        [~]=vrep.simxSynchronousTrigger(clientID);
        tic
        % Get Joint angle
        jointAngle(i,1:8) = getJointPosition(vrep,clientID,J_h);
        % Get end effector position
        endEffectorPosition(:,i) = getEndEffectorPosition(vrep,clientID,J_h);
        % Dynamics control 
         setJointTargetPosition(vrep,clientID,trj1(i,:),J_h);
         
        % Grasp the bottle
        if(abs(endEffectorPosition(:,i) - bottlePos) <= 1e-03)
           closeGrip(vrep,clientID,bottle_h,J_h(7));
           disp('grip chiuso')
        end
    end
openGrip(vrep,clientID,bottle_h)
[~] = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot); 
else
    disp("Fail");
end
 %%
figure(1)
view(3)
hold on, grid on
title('End Effector Trajectory')
plot3(endEffectorPosition(1,:),endEffectorPosition(2,:),endEffectorPosition(3,:),'b*');
plot3(circonferenza(1,:),circonferenza(2,:),circonferenza(3,:),'ro');
legend('CoppeliaSim','IK Matlab')
xlabel('X [m]');ylabel('Y [m]');zlabel('Z [m]')
xlim([-2 2]);ylim([-2 2]);zlim([0 2]);axis equal

% Angoli tra -pi +pi
figure(2)
subplot(321)
hold on,grid on
title('\theta_{1}')
plot((1:length(jointAngle))*0.005,jointAngle(:,1));xlabel('time [s]');
subplot(322)
hold on,grid on
title('\theta_{2}')
plot((1:length(jointAngle))*0.005,jointAngle(:,2));xlabel('time [s]');
subplot(323)
hold on,grid on
title('\theta_{3}')
plot((1:length(jointAngle))*0.005,jointAngle(:,3));xlabel('time [s]');
subplot(324)
hold on,grid on
title('\theta_{4}')
plot((1:length(jointAngle))*0.005,jointAngle(:,4));xlabel('time [s]');
subplot(325)
hold on,grid on
title('\theta_{5}')
plot((1:length(jointAngle))*0.005,jointAngle(:,5));xlabel('time [s]');
subplot(326)
hold on,grid on
title('\theta_{6}')
plot((1:length(jointAngle))*0.005,jointAngle(:,6));xlabel('time [s]');

%% Functions
function J_h = getJointHandle(vrep,clientID)
    [~,endEffector_h] = vrep.simxGetObjectHandle(clientID,'endEffector',vrep.simx_opmode_blocking);
    [~,J1_h] = vrep.simxGetObjectHandle(clientID,'IRB140_joint1',vrep.simx_opmode_blocking);
    [~,J2_h] = vrep.simxGetObjectHandle(clientID,'IRB140_joint2',vrep.simx_opmode_blocking);
    [~,J3_h] = vrep.simxGetObjectHandle(clientID,'IRB140_joint3',vrep.simx_opmode_blocking);
    [~,J4_h] = vrep.simxGetObjectHandle(clientID,'IRB140_joint4',vrep.simx_opmode_blocking);
    [~,J5_h] = vrep.simxGetObjectHandle(clientID,'IRB140_joint5',vrep.simx_opmode_blocking);
    [~,J6_h] = vrep.simxGetObjectHandle(clientID,'IRB140_joint6',vrep.simx_opmode_blocking);
    [~,J8_h] = vrep.simxGetObjectHandle(clientID,'J1',vrep.simx_opmode_blocking);
    [~,J9_h] = vrep.simxGetObjectHandle(clientID,'J2',vrep.simx_opmode_blocking);
    J_h = [J1_h,J2_h,J3_h,J4_h,J5_h,J6_h,endEffector_h,J8_h,J9_h];
end
function bottle_h = getBottleHandle(vrep,clientID)
    [~,bottle_h] = vrep.simxGetObjectHandle(clientID,'bottle',vrep.simx_opmode_blocking);
end
function glass_h = getGlassHandle(vrep,clientID)
    [~,glass_h] = vrep.simxGetObjectHandle(clientID,'glass',vrep.simx_opmode_blocking);
end
function bottlePosition = getBottlePosition(vrep,clientID,bottle_h)
    [~,bottlePosition] = vrep.simxGetObjectPosition(clientID,bottle_h,-1,vrep.simx_opmode_blocking);
end
function glassPosition = getGlassPosition(vrep,clientID,glass_h)
    [~,glassPosition] = vrep.simxGetObjectPosition(clientID,glass_h,-1,vrep.simx_opmode_blocking);
end
function [] = startSimulation(vrep,clientID)
    [~] = vrep.simxSynchronous(clientID,true);
    [~] = vrep.simxSynchronousTrigger(clientID);
    [~] = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);
end
function [] = setInitialJointPosition(vrep,clientID,trj1,J_h)
    [~] = vrep.simxSetJointPosition(clientID,J_h(1),trj1(1,1),vrep.simx_opmode_blocking); %lag di 20ms circa costanti
    [~] = vrep.simxSetJointPosition(clientID,J_h(2),trj1(1,2)-pi/2,vrep.simx_opmode_blocking);
    [~] = vrep.simxSetJointPosition(clientID,J_h(3),trj1(1,3),vrep.simx_opmode_blocking);
    [~] = vrep.simxSetJointPosition(clientID,J_h(4),trj1(1,4),vrep.simx_opmode_blocking);
    [~] = vrep.simxSetJointPosition(clientID,J_h(5),trj1(1,5),vrep.simx_opmode_blocking);
    [~] = vrep.simxSetJointPosition(clientID,J_h(6),trj1(1,6),vrep.simx_opmode_blocking);
    [~] = vrep.simxSetJointPosition(clientID,J_h(7),deg2rad(0),vrep.simx_opmode_oneshot);
    [~] = vrep.simxSetJointPosition(clientID,J_h(8),deg2rad(0),vrep.simx_opmode_oneshot);
    pause(2)
end
function q = getJointPosition(vrep,clientID,J_h)
    [~,q1] = vrep.simxGetJointPosition(clientID,J_h(1),vrep.simx_opmode_blocking);
    [~,q2] = vrep.simxGetJointPosition(clientID,J_h(2),vrep.simx_opmode_blocking);
    [~,q3] = vrep.simxGetJointPosition(clientID,J_h(3),vrep.simx_opmode_blocking);
    [~,q4] = vrep.simxGetJointPosition(clientID,J_h(4),vrep.simx_opmode_blocking);
    [~,q5] = vrep.simxGetJointPosition(clientID,J_h(5),vrep.simx_opmode_blocking);
    [~,q6] = vrep.simxGetJointPosition(clientID,J_h(6),vrep.simx_opmode_blocking);
    [~,q7] = vrep.simxGetJointPosition(clientID,J_h(7),vrep.simx_opmode_blocking);
    [~,q8] = vrep.simxGetJointPosition(clientID,J_h(8),vrep.simx_opmode_blocking);
    q = [q1 q2 q3 q4 q5 q6 q7 q8];
end
function endEffector = getEndEffectorPosition(vrep,clientID,J_h)
    [~,endEffector] = vrep.simxGetObjectPosition(clientID,J_h(7),-1,vrep.simx_opmode_blocking);
end
function [] = setJointTargetPosition(vrep,clientID,trj1,J_h)
    [~] = vrep.simxSetJointTargetPosition(clientID,J_h(1),trj1(1,1),vrep.simx_opmode_oneshot);
    [~] = vrep.simxSetJointTargetPosition(clientID,J_h(2),trj1(1,2)-pi/2,vrep.simx_opmode_oneshot);
    [~] = vrep.simxSetJointTargetPosition(clientID,J_h(3),trj1(1,3),vrep.simx_opmode_oneshot);
    [~] = vrep.simxSetJointTargetPosition(clientID,J_h(4),trj1(1,4),vrep.simx_opmode_oneshot);
    [~] = vrep.simxSetJointTargetPosition(clientID,J_h(5),trj1(1,5),vrep.simx_opmode_oneshot);
    [~] = vrep.simxSetJointTargetPosition(clientID,J_h(6),trj1(1,6),vrep.simx_opmode_oneshot);
    [~] = vrep.simxSetJointTargetPosition(clientID,J_h(8),0,vrep.simx_opmode_oneshot);
    [~] = vrep.simxSetJointTargetPosition(clientID,J_h(9),0,vrep.simx_opmode_oneshot);
end
function [] = closeGrip(vrep,clientID,bottle_h,endEffector_h)  
    [~] = vrep.simxSetObjectParent(clientID,bottle_h,endEffector_h,true,false);
end
function [] = openGrip(vrep,clientID,bottle_h)  
    [~] = vrep.simxSetObjectParent(clientID,bottle_h,-1,true,false);
end

