% function [dronePort, localPort] = SetVideoReceiver
%  
% Execution of this function requires ffmpeg installed on your computer.
% It is recommended that user should look inside the function and revise
% the system command for specific computer OS and settings.
%  ...... The function is tested on Macbook OS 10.8.3
% 
% This function use local port 9438 to transimit the video from the drone
% to ffplay player (which is open source and included in ffmpeg)
% 
% The function also use the nc (netcat) to receive the video from the port.
% 
% The system command is : nc -l 127.0.0.1 9438 | /usr/local/bin/ffplay -f h264 -i - &
% 
% *************************************
% *  Authors:
%    Kun Zhang (dabiezu@gmail.edu)
%    Pieter J. Mosterman (pmosterman@yahoo.com) *
% *************************************
% 


function [dronePort, localPort] = SetVideoReceiver
Size = 2^15;
try
    system('nc -l 127.0.0.1 9438 | /usr/local/bin/ffplay -f h264 -i - &');
catch excp
    disp(excp.message)
    disp('required software tool: netcat(linux and mac TCP/UDP tool), ffmpeg(including ffplay)')
    dronePort = -1;
    localPort = -1;
    return
end
    
try
    dronePort = tcpip('192.168.1.1', 5555); % in_
    set(dronePort,'Timeout',10);
    set(dronePort,'InputBufferSize',Size);

    echotcpip('on',9438);
    localPort = tcpip('127.0.0.1', 9438); % out_
    set(localPort,'OutputBufferSize',Size);

    fopen(dronePort);
    fopen(localPort);
catch excp
    disp(excp.message)
    dronePort = -1;
    localPort = -1;
    try
    fclose(localPort);
    fclose(dronePort);
    echotcpip('off');
    catch
    end
end
  
end


