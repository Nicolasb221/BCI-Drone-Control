% function [] = TransmitVideo2Player (dronePort, localPort)
% 
% *************************************
% *  Authors:
%    Kun Zhang (dabiezu@gmail.edu)
%    Pieter J. Mosterman (pmosterman@yahoo.com) *
% *************************************
% 
function [] = TransmitVideo2Player (dronePort, localPort)

if dronePort == -1 || localPort == -1
    disp('Video Receiver Setup failed.')
    return
end

try 
if (get( dronePort,'BytesAvailable' )==0)
    disp('try to reconnect the drone video channel...');
    fclose(dronePort);
    fopen(dronePort);
    fprintf(dronePort,'GET /');
end

while(get( dronePort,'BytesAvailable' )>0)
    DataReceived = fread(dronePort);
    fwrite(localPort,DataReceived);
end

catch
end

end