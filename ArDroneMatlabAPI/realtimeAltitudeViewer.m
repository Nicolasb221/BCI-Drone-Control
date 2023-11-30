% function realtimeAltitudeViewer (Duration_in_second)
% 
% This function opens the UDP ports to the drone, inquires drone states,
%               and then plots the Drone attitude in a 3-D model.
% Users can use the function to test connection between drone and local computer.
% 
% 
% *************************************
% *  Authors:
%    Kun Zhang (dabiezu@gmail.edu)
%    Pieter J. Mosterman (pmosterman@yahoo.com) *
% *************************************
% 


function realtimeAltitudeViewer (Duration_in_second)
if Duration_in_second <=0
    return
end
controlChannel = udp('192.168.1.1', 5556, 'LocalPort', 5556);
stateChannel = udp('192.168.1.1', 5554, 'LocalPort', 5554);
try
    fopen(controlChannel);
    fopen(stateChannel);
catch excp
    disp('failed to open udp channels.');
    disp(excp.message)
    return
end

try
    SequenceNumber = tic;
    t_ = 0;
    t_0 = clock;
    while(t_<Duration_in_second);
        [~, ~, SequenceNumber] = Ask4DroneState (SequenceNumber, controlChannel, stateChannel, 1);
        t_ = etime(clock,t_0);
    end

catch excp
    disp(excp.message)
    fclose(controlChannel);
    fclose(stateChannel);
end

fclose(controlChannel);
fclose(stateChannel);