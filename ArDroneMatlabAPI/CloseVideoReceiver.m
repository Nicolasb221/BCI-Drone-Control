% function CloseVideoReceiver(dronePort, localPort)
% 
% *************************************
% *  Authors:
%    Kun Zhang (dabiezu@gmail.edu)
%    Pieter J. Mosterman (pmosterman@yahoo.com) *
% *************************************
%  
function CloseVideoReceiver(dronePort, localPort)
try
    fclose(localPort);
catch e
    disp(e.message)
end
try
    fclose(dronePort);
catch e
    disp(e.message)
end
try
    echotcpip('off');
catch e
    disp(e.message)
end
end