% function [DroneState, OptionData_collection, SequenceNumber] = ControlPannel (SequenceNumber, controlChannel, stateChannel, controlCommand, IfDrawAttitude)
% 
% ----------------------------- input ----------------------------- 
%  SequenceNumber : is the sequence number can be used instantly for the function
% 
%  controlCommand_duration : looks like as the following formmat
%           [controlCommand_1 duration_1_in_sec]
%           [controlCommand_2 duration_2_in_sec]
%           ....................................
%           ....................................
%           [controlCommand_N duration_N_in_sec]
%      if duration == 0 then the command will only be executed once.
%      if duration < 0, return
% 
% The function uses function [SequenceNumber] = MotionCommand(SequenceNumber, controlChannel,flag,LR_tilt,FB_tilt,VerticalVel,AngularVel)
%       to transmit command, controlCommand_i is the actual input of MontionCommand (except SequenceNumber and controlChannel).
%  for the detail of setting controlCommand, please refer to MotionCommand.m
% the size controlCommand should be N*6
% 
% controlChannel and stateChannel are UDP obj
% 
%  if (IfDrawAttitude == 1) then plot attitude of the drone
% 
% ----------------------------- output ----------------------------- 
%  DroneState returns the latest update of drone state, which is an array with 32 elements. (bit position [31, 30, 29,..., 2,1,0])
%  For the detail meaning of each bit, please refer to the Interpret_NavData function.
% 
%  OptionData_collection returns all inputs with timestamps and all available optionData (velocity, angular speed, height, etc.) with timestamps;
%  * each column of optionData = [battery in %
%                                pitch in angle
%                                roll
%                                yaw
%                                altitude in meter
%                                V_x in m/s
%                                V_y in m/s
%                                V_z in m/s];
% 
%  * if all values in the column are 0, then this column of optionData is invalid.
% 
%  One coloum of the OptionData_collection has the structure:
%                    [ command send time  ----------1
%                      Left_Right tilt
%                      Front_Back tilt
%                      Vertical velocity
%                      Angular Velocity
%                      state received time ---------6
%                      battery in %
%                      pitch in angle
%                      roll
%                      yaw                 ---------10
%                      altitude in meter
%                      V_x in m/s
%                      V_y in m/s
%                      V_z in m/s];        ---------14
% 
%        the first command send time will be set to 0, other timestamp will be set according to this timing benchmark
% 
% *************************************
% *  Authors:
%    Kun Zhang (dabiezu@gmail.edu)
%    Pieter J. Mosterman (pmosterman@yahoo.com) *
% *************************************
% 

function [DroneState, OptionData_collection, SequenceNumber] = ControlPannel (SequenceNumber, controlChannel, stateChannel, controlCommand, IfDrawAttitude)

DroneState = 0;
OptionData_collection = [];
[R, C] = size(controlCommand);

if C ~= 6
    return;
end

count = 1;
 t_0 = clock;
for R_i = 1:R
    duration = controlCommand(R_i,6);
    if duration < 0
        disp('para duration in ControlPannel is not correct...');
        return;
    end
    
    t_ = 0;
   t_start = clock;
    while(t_ <= duration)
        % send command
        SequenceNumber = MotionCommand(SequenceNumber,controlChannel,controlCommand(R_i,1),controlCommand(R_i,2),...
            controlCommand(R_i,3), controlCommand(R_i,4), controlCommand(R_i,5));
        OptionData_collection(1:5,count) = [etime(clock,t_0)
                                            controlCommand(R_i,2)
                                            controlCommand(R_i,3)
                                            controlCommand(R_i,4)
                                            controlCommand(R_i,5)];
        
        % ask for drone state and option data
        [droneState, OptionData, SequenceNumber] = Ask4DroneState (SequenceNumber, controlChannel, stateChannel, IfDrawAttitude);
        if length(OptionData) ~= 8
            OptionData = zeros(8,1);
        end
        OptionData_collection(6:14,count) = [etime(clock,t_0)
                                             OptionData];
        
        count = count + 1;
        % if the drone is in emergency state, then return this function
        if (droneState(1) == 1) % in emergency state
            return;
        end; 
        
        % check the time validity.
        t_ = etime(clock,t_start);
    end
    
end
OptionData_collection(1,:) = OptionData_collection(1,:) - OptionData_collection(1,1);
OptionData_collection(6,:) = OptionData_collection(6,:) - OptionData_collection(1,1);
end
    
    
    
    
    
    
    
    
    
    
    