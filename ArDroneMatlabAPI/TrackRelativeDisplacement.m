% function [SequenceNumber, coll] = TrackRelativeDisplacement ( SequenceNumber, controlChannel, stateChannel, Target, IfPlot,Indoor_Outdoor_Opt)
% 
%  --------------- input:
% 
%  4. Target = [target-1 timeout-1
%               target-2 timeout-2
%               target-3 timeout-3
%               ......   ......
%               target-N timeout-N]
%  and 
%     target-i = [relative-x, relative-y, yaw]; where yaw is the global orientation measured from the drone's compass chip.
%          * the yaw control will be developed later.
% 
%  user should specify the environment: "indoor" or "outdoor"
% ------------------ output:
%     SequenceNumber: the next Number that the drone can use;
%     coll : position, input comman, and speed in x,y direction.
% This function uses a simple controller to control the drone. 
%             The model the controller based on is 
%                   [X   ]    [ 1 DeltT 0 0]    [X   ]    [ 0    0]
%                   [V_x ] =  [ 0   1   0 0] *  [V_x ] +  [ k_1  0] * [FrontBack-Tilt] 
%                   [Y   ]    [0 0 1  DeltT]    [Y   ]    [ 0    0]   [RighLeft-Tilt ]
%                   [V_y ]    [0 0 0      1]    [V_y ]    [ 0  k_2]
% 
%                   where k_1 and k_2 can be acquired by doing a simple
%                   System ID roughly.
%                   In this case, the k_1 = 2; k_2 = 2.5;
%                   Tilt locates within [-1 1];
%                   The controller is self-explanatory in the code.
%
% *************************************
% *  Authors:
%    Kun Zhang (dabiezu@gmail.edu)
%    Pieter J. Mosterman (pmosterman@yahoo.com) *
% *************************************
% 
function [SequenceNumber, coll] = TrackRelativeDisplacement ( SequenceNumber, controlChannel, stateChannel, Target, IfPlot,Indoor_Outdoor_Opt)

indoor_time_req = 1;
outdoor_time_req = 0.8;
if strcmp(Indoor_Outdoor_Opt, 'outdoor')
    disp('outdoor environment...');
    indoor_time_req = outdoor_time_req;
else
    disp('indoor environment...');
end

coll = [];

size_Target = size(Target);
if size_Target(2) ~= 4 % [x, y, yaw, timeout]
    return;
end

PAL = 0.1;
YAL = 0.5;

RL_tilt=0;
FB_tilt=0;



for tp = 1:size_Target(1) % iterates throught all target points ---- loop 1
    fprintf('....%d-th target point ...\n',tp);
    pos_x = 0;
    pos_y = 0;
    t_0 = clock;
%   -----------------------------------------------------------------   
%   ********** 1st : move to displacement position in X-Y-plane
%   -----------------------------------------------------------------
dlt_t = 0.02;
    while 1 % ---- moving to (x,y) loop in loop of interating all targets
        t_start = clock;
        dlt_t_used = 0;
        if etime(clock,t_0) >= Target(tp,4)
            break;
        end
        [DroneState, OptionData, SequenceNumber] = Ask4DroneState (SequenceNumber, controlChannel, stateChannel, 0);
        if DroneState(1) == 1 % in emergency state
            return; 
        end
        
        %   1      2       3         4          5     
        %  flag,LR_tilt,FB_tilt,VerticalVel,AngularVel
        if length(OptionData) == 8
            pos_x = pos_x + dlt_t*OptionData(6);
            pos_y = pos_y + dlt_t*OptionData(7);
            dlt_t_used = 1;
            
            if (IfPlot==1)
                figure(1)
                hold on
                plot(pos_y,pos_x,'.');
            end
            
            Vt_x = atan(Target(tp,1) - pos_x);
            Vt_y = atan(Target(tp,2) - pos_y);
            FB_tilt = -(Vt_x-OptionData(6))/indoor_time_req/2;
            RL_tilt = (Vt_y-OptionData(7))/indoor_time_req/2.25;
            if abs(FB_tilt) > 1
                FB_tilt = FB_tilt/abs(FB_tilt);
            end
            
            if abs(RL_tilt) > 1
                RL_tilt = RL_tilt/abs(RL_tilt);
            end
            
            if abs(Target(tp,1) - pos_x) <= PAL && abs(Target(tp,2) - pos_y) <= PAL
                RL_tilt = 0;
                FB_tilt = 0;
            end
             
        % ======================
% this part is deleted...
             % add the heading control
%         OptionData(4) ---- measured yaw angle
%         Target(tp,3) ----- target yaw angle
%         if OptionData(4) < 0 
%             OptionData(4) = 360 + OptionData(4);
%         end
%         
%         angularV = 0.2*atan(Target(tp,3) - OptionData(4))/1.6;
%         if abs(Target(tp,3) - OptionData(4)) >180
%             angularV =-angularV;
%         end
%         if abs(Target(tp,3) - OptionData(4)) <= YAL
%             angularV = 0;
%         end
            
        angularV = 0;
        
            
            
            coll = [coll, [pos_x;pos_y;RL_tilt;FB_tilt;OptionData(6);OptionData(7)]];
            SequenceNumber = MotionCommand(SequenceNumber,controlChannel,...
                 1,RL_tilt,FB_tilt, 0, angularV);
            
        else
           SequenceNumber = MotionCommand(SequenceNumber,controlChannel,...
                1,RL_tilt,FB_tilt, 0, angularV);
        end
        if (dlt_t_used)
            dlt_t = etime(clock,t_start);
        else % dlt_t not used
            dlt_t = etime(clock,t_start) + dlt_t;
        end
        
        
        
        
    end

end % ---- loop 1 end
















