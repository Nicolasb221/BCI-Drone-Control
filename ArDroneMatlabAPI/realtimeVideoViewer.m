% function StartVideoView
% 
% The function executes a system command to play the video stream from the
% drone in real time.
% 
% Execution of this function requires ffmpeg installed on your computer.
% Currently a matlab decoder is not available.
% It is recommended that user should look inside the function and revise
% the system command for specific computer OS and settings.
%  ...... The function is tested on Macbook OS 10.8.3
% 
% *************************************
% *  Authors:
%    Kun Zhang (dabiezu@gmail.edu)
%    Pieter J. Mosterman (pmosterman@yahoo.com) *
% *************************************
% 

function realtimeVideoViewer
    system('/usr/local/bin/ffplay -f h264 http://192.168.1.1:5555 &');
end