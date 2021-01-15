clear all
clc

vrep = remApi('remoteApi');
vrep.simxFinish(-1); %Pulisce connessioni aperte
clientID = vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

a = struct([]);
[returnCode,Vision_sensor]=vrep.simxGetObjectHandle(clientID,'Vision_sensor',vrep.simx_opmode_blocking);
vrep.simxGetVisionSensorImage2(clientID,Vision_sensor,0,vrep.simx_opmode_streaming);
i  = 1;
while (clientID~=-1)
    [number returnCode]=simxSetObjectIntParameter(clientID,Vision_sensor,visionintparam_render_mode,1017,vrep.simx_opmode_oneshot)
    [returnCode,resolution,Image]=vrep.simxGetVisionSensorImage2(clientID,Vision_sensor,0,vrep.simx_opmode_buffer);
    if (returnCode==vrep.simx_return_ok)
        imshow(Image)
%        a(i) = Image; 
%        i = i+1;
    end
%     break;
end


