% function [droneState, OptionData, SequenceNumber] = Ask4DroneState (SequenceNumber, controlChannel, stateChannel, drawAttitude)
% if length(droneState) == 32
%   or length(OptionData) == 8
%  then the returned values are valid.
% a valid output is an 8*1 array in which the structure looks like:
% [batteryLevel unit: %        1
%  pitch        unit: angle    2
%  roll         unit: angle    3
%  yaw          unit: angle    4
%  altitude     unit: meter    5
%  Vx           unit: m/s      6
%  Vy           unit: m/s      7
%  Vz];         unit: m/s      8
%  --------------- input
%          drawAttitude == 1 ----> draw the animation of drone attitude 
% The input SequenceNumber is the Sequence for instance use;
%  the output SequenceNumber is the Sequence for instance use in the next
%  command.
% This command execution time is always bounded within 0.022 sec (after a successful connection)
%         detail: 
%               the first connection can take 0.13 sec
%                  after that, the return time can less than 0.007 sec.      
% 
% 
% The 3-D plotting part takes advantage of trajectory3.m function (author: Valerio Scordamaglia)
% ... from matlab files exchange.
% 
% *************************************
% *  Authors:
%    Kun Zhang (dabiezu@gmail.edu)
%    Pieter J. Mosterman (pmosterman@yahoo.com) *
% *************************************
% 
function [droneState, OptionData, SequenceNumber] = Ask4DroneState (SequenceNumber, controlChannel, stateChannel, drawAttitude)
% ---------------------------------

droneState = 0;
OptionData = 0;

stateChannel.Timeout = 0.001; % it seems the minimum UDP timeout that allowed in matlab is about 1 sec

% According to the documentation SDK 1_7
% to sync nav data stream
% first: send a some packet to NavData port
try
    fprintf(stateChannel, 1);
catch
end
% then Send the request for navdata_demo
try
    fprintf(controlChannel, ...
        sprintf('AT*CONFIG=%d,\"general:navdata_demo\",\"TRUE\"\r',SequenceNumber));
catch
end
SequenceNumber = SequenceNumber+1;

% only ask for once
try
    fprintf(controlChannel, sprintf('AT*COMWDG=%d\r',SequenceNumber));
catch
end
SequenceNumber = SequenceNumber + 1;
nav_data = fread(stateChannel,299,'uint8');
fclose(stateChannel); 

if(length(nav_data) >299)
    [droneState, OptionData] =  Interpret_NavData(nav_data);
end

% The reason that add the following loop is: 1. solve the conflicts when running this
%  ... function on different matlab version, and also on different os (Mac os & windows 7)
%  2. ...
% 
% running on matlab 2013b
% win 7 @ i5-2450 2.5GHZ
% this loop takes 0.5-1.5 ms (0.0005-0.0015 secondes) normally
% sometimes worst condition could be 10 ms (e.g. try to open the udp 3 times)
% 
% running on matlab 2013a 64 bit, Mac OS 10.8.3, 1.8GHz Intel Core i5
% this loop usually takes 1-2 ms (0.001-0.002 secondes)

mark = 1;
while mark
try
   fopen(stateChannel);
   mark = 0;
catch
   mark = 1;
end


if (drawAttitude == 1) && length(OptionData) >= 8
    trajectory3(0,0,OptionData(5),OptionData(2)/180*pi,OptionData(3)/180*pi,-OptionData(4)/180*pi,0.15,0,'747',[0.1 0 0]);
    pause(0.001);
end
    
end







