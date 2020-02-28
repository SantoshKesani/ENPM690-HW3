vrep=remApi('remoteApi');
 vrep.simxFinish(-1); 
 clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

 if (clientID>-1)
        disp('Connection is established');
     motion= input('Control of Robot : Press W,A,S,D for Forward, Left, Reverse, Right, and Press y to end','s');
    while(motion~='y')
   if (motion=='W' ||motion=='w' || motion=='A' ||motion=='a' || motion=='S' ||motion=='s' || motion=='D' ||motion=='d') 
     
%   Handle
    [returnCode,left_Motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking );
    [returnCode,Right_Motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking );
    [returnCode,front_Sensor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor5',vrep.simx_opmode_blocking );
    [returnCode,camera]=vrep.simxGetObjectHandle(clientID,'Vision_sensor',vrep.simx_opmode_blocking );

%   Code controlling the action
   if( motion=='W' ||motion=='w')
    [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,0.5,vrep.simx_opmode_blocking);    
    [returnCode]=vrep.simxSetJointTargetVelocity( clientID,Right_Motor,0.5,vrep.simx_opmode_blocking);   
   elseif( motion=='A' ||motion=='a')
   [returnCode]=vrep.simxSetJointTargetVelocity( clientID,Right_Motor,0.5,vrep.simx_opmode_blocking);
   [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,0,vrep.simx_opmode_blocking);     
   elseif( motion=='S' ||motion=='s')
    [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,-0.5,vrep.simx_opmode_blocking);    
    [returnCode]=vrep.simxSetJointTargetVelocity( clientID,Right_Motor,-0.5,vrep.simx_opmode_blocking);
   elseif( motion=='D' ||motion=='d')
    [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,0.5,vrep.simx_opmode_blocking); 
    [returnCode]=vrep.simxSetJointTargetVelocity( clientID,Right_Motor,0,vrep.simx_opmode_blocking);

   end
    [returnCode,detectionState,detectedPoint,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor,vrep.simx_opmode_streaming);
    [returnCode,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,0,vrep.simx_opmode_streaming);
    [returnCode,leftlinearvelocity, leftangularvelocity]=vrep.simxGetObjectVelocity(clientID,left_Motor,vrep.simx_opmode_streaming);
    [returnCode,rightlinearvelocity, rightangularvelocity]=vrep.simxGetObjectVelocity(clientID,Right_Motor,vrep.simx_opmode_streaming);

    [returnCode,leftlinearvelocity,leftangularvelocity]=vrep.simxGetObjectVelocity(clientID,left_Motor,vrep.simx_opmode_buffer);
    [returnCode,rightlinearvelocity, rightangularvelocity]=vrep.simxGetObjectVelocity(clientID,Right_Motor,vrep.simx_opmode_buffer);
    [returnCode,detectionState,detectedPoint,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor,vrep.simx_opmode_buffer );
    [returnCode,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,0,vrep.simx_opmode_buffer);
    imshow(image) %showing the obstacle infornt of sensor
    %Printing the values of distances and velocities
    fprintf("Distance of the obstacle= %d m \n",norm(detectedPoint));
    fprintf("Linear Velocity of Left Wheel= %d m/s \n",leftlinearvelocity);
    fprintf("Linear Velocity of Right Wheel= %d m/s \n",rightlinearvelocity);
    fprintf("Angular Velocity of Left Wheel= %d rad/s \n",leftangularvelocity);
    fprintf("Angular Velocity of Right Wheel= %d rad/s \n",rightangularvelocity);
    
     pause(0.1);
     
  else
    [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,0,vrep.simx_opmode_blocking); 
    [returnCode]=vrep.simxSetJointTargetVelocity( clientID,Right_Motor,0,vrep.simx_opmode_blocking);    
     
   end
  motion = getkeywait(2);
  continue
  
     
    end
    vrep.simxFinish(-1); 
 end
 
 vrep.delete();
        