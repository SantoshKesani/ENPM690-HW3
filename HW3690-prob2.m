vrep=remApi('remoteApi');
 vrep.simxFinish(-1); 
 clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
 scenter = 1;i=0;
 if (clientID>-1)
        disp('Connection is established');

while( scenter~='null' )
i=i+1;
%  Handle
    [returnCode,left_Motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking );
    [returnCode,Right_Motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking );
    [returnCode,front_Sensor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor5',vrep.simx_opmode_blocking );
    [returnCode,left_Sensor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor3',vrep.simx_opmode_blocking );
    [returnCode,right_Sensor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor7',vrep.simx_opmode_blocking );
    [returnCode,camera]=vrep.simxGetObjectHandle(clientID,'Vision_sensor',vrep.simx_opmode_blocking );

% Sensor Readings
    [returnCode,detectionStatecenter,scenter,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor,vrep.simx_opmode_streaming);
    [returnCode,detectionStateleft,sleft,~,~]=vrep.simxReadProximitySensor(clientID,left_Sensor,vrep.simx_opmode_streaming);
    [returnCode,detectionStateright,sright,~,~]=vrep.simxReadProximitySensor(clientID,right_Sensor,vrep.simx_opmode_streaming);
    
    scenter = norm(scenter);
    sleft = norm(sleft);
    sright = norm(sright);
     
    [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,1,vrep.simx_opmode_blocking);    
    [returnCode]=vrep.simxSetJointTargetVelocity( clientID,Right_Motor,1,vrep.simx_opmode_blocking); 
   

    if ( scenter<0.5 && scenter>0.001 || sleft<0.5 && sleft>0.001 || sright<0.5 && sright>0.001)
        if ( sleft<0.5 && sleft>0.001 && sright< 0.01 && scenter< 0.01)
            [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,0.5,vrep.simx_opmode_blocking); 
            [returnCode]=vrep.simxSetJointTargetVelocity( clientID,Right_Motor,-0.5,vrep.simx_opmode_blocking);
             pause(0.05)
            [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,0,vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetVelocity( clientID,Right_Motor,0,vrep.simx_opmode_blocking); 

        elseif (sright<0.5 && sright>0.001 && sleft< 0.01 && scenter< 0.01)
            [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,-0.5,vrep.simx_opmode_blocking); 
            [returnCode]=vrep.simxSetJointTargetVelocity( clientID,Right_Motor,0.5,vrep.simx_opmode_blocking);
             pause(0.05)
            [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,0,vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetVelocity( clientID,Right_Motor,0,vrep.simx_opmode_blocking);
        end
        
                [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,-0.5,vrep.simx_opmode_blocking); 
                [returnCode]=vrep.simxSetJointTargetVelocity( clientID,Right_Motor,0.5,vrep.simx_opmode_blocking);
                 pause(0.2)
                [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,0,vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointTargetVelocity( clientID,Right_Motor,0,vrep.simx_opmode_blocking);

    end
 
    
    
    [returnCode,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,0,vrep.simx_opmode_streaming);
    [returnCode,leftlinearvelocity, leftangularvelocity]=vrep.simxGetObjectVelocity(clientID,left_Motor,vrep.simx_opmode_streaming);
    [returnCode,rightlinearvelocity, rightangularvelocity]=vrep.simxGetObjectVelocity(clientID,Right_Motor,vrep.simx_opmode_streaming);

    [returnCode,leftlinearvelocity,leftangularvelocity]=vrep.simxGetObjectVelocity(clientID,left_Motor,vrep.simx_opmode_buffer);
    [returnCode,rightlinearvelocity, rightangularvelocity]=vrep.simxGetObjectVelocity(clientID,Right_Motor,vrep.simx_opmode_buffer);
    [returnCode,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,0,vrep.simx_opmode_buffer);
    imshow(image)
    disp(norm(scenter));
    disp(norm(sleft));
    disp(norm(sright));  
     
   end
  
    vrep.simxFinish(-1); 
 end
 
 vrep.delete();
 