vrep = remApi('remoteApi');
vrep.simxFinish(-1); %Pulisce connessioni aperte
clientID = vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

% % if(clientID > -1)
% %     
% % disp("Connessione Effettuata");
% % vrep.simxAddStatusbarMessage(clientID,'Connessione da Matlab',vrep.simx_opmode_oneshot);%invio messaggio a vrep
% % [~] = vrep.simxSynchronous(clientID,true)
% % vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking);
% % [~]=vrep.simxSynchronousTrigger(clientID);
% % [rec, sensorHandle]= vrep.simxGetObjectHandle(clientID, 'Vision_sensor', vrep.simx_opmode_blocking); 
% % [~]=vrep.simxSynchronousTrigger(clientID);
% % [rec, arr, image]= vrep.simxGetVisionSensorImage2( clientID, sensorHandle, 1 , vrep.simx_opmode_streaming);
% % [~]=vrep.simxSynchronousTrigger(clientID);
% % figure
% % imshow(image); 
% %     [~] = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot); 
% % else
% %     disp("Fail");
% % end

[returnCode,Vision_sensor]=vrep.simxGetObjectHandle(clientID,'screenshotSensor',vrep.simx_opmode_blocking);
[~] = vrep.simxGetVisionSensorImage2(clientID,Vision_sensor,1,vrep.simx_opmode_streaming);

    [returnCode,resolution,Image]=vrep.simxGetVisionSensorImage2(clientID,Vision_sensor,1,vrep.simx_opmode_buffer);
    if (returnCode==vrep.simx_return_ok)
        imshow(Image)
    end
