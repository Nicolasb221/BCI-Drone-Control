% function [SequenceNumber] = MotionCommand(SequenceNumber, controlChannel,flag,LR_tilt,FB_tilt,VerticalVel,AngularVel)
% Keep sending this command can maintain the control
%  e.g. : send this function with a rotate command once, the command will
%         last only for a while (maybe 0.1,0.2sec). The drone will rotate
%         just a little bit.
% 
% --------------------- input & requirements ---------------
% -----******** pls read carefully, breaking the requirements leads to
% failure execution of this command ******---------
% 
% 1. SequenceNumber 
% 2. controlChannel : UDP('192.168.1.1', 5556, 'LocalPort', 5556)
% 3. falg: flag enabling the use of progressive commands and/or the Combined Yaw mode (bitfield)
%     Always set the flag bit 0 to 1 to make the drone consider the other arguments. 
%     Setting it to 0 makes the drone enter hovering mode
%     (staying on top of the same point on the ground).
%   Thus, a recommended vaule for flag is 1 if flight control is desired.
% 4. LR_tilt: drone left-right tilt - floating-point value in range [-1..1]
%             A negative value makes the drone tilt to its left
% 5. FB_tilt: drone front-back tilt - floating-point value in range [-1..1]
%             A negative value makes the drone lower its nose
% 6. VerticalVel: drone vertical speed - floating-point value in range [-1..1]
%             A positive value makes the drone rise in the air
% 7. AngularVel: drone angular speed - floating-point value in range [-1..1]
%             A negative value makes the Drone spin left  


% --------------------- output ---------------------------
% The return value, SequenceNumber, is the next sequence user can use.
% --------------------- Note -----------------------------
% if you want to continuouly send this command, you need to mannually add pause or wait
% (e.g. wait(0.2)) in your loop. otherwise the UDP may cannot accept high
% frequence update.
% 
% 
% *************************************
% *  Authors:
%    Kun Zhang (dabiezu@gmail.edu)
%    Pieter J. Mosterman (pmosterman@yahoo.com) *
% *************************************
% 
function [SequenceNumber] = MotionCommand(SequenceNumber, controlChannel,flag,LR_tilt,FB_tilt,VerticalVel,AngularVel)
if (LR_tilt>=-1) && (LR_tilt<=1) && (FB_tilt>=-1) && (FB_tilt<=1) && (VerticalVel>=-1) && (VerticalVel<=1) && (AngularVel>=-1) && (AngularVel<=1)
Sig = sprintf('AT*PCMD=%d,%d,%d,%d,%d,%d\r',...
    SequenceNumber,flag,ARDrone_FloatArg2Int(LR_tilt),ARDrone_FloatArg2Int(FB_tilt),ARDrone_FloatArg2Int(VerticalVel),ARDrone_FloatArg2Int(AngularVel));
try
    fprintf(controlChannel, Sig);
catch 
    disp('Exception in MotionCommand()...');
end
SequenceNumber = SequenceNumber + 1;
end
