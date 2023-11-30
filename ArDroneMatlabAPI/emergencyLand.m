% emergency land
% If your matlab code get an error somewhere and stop running, but the
% drone still is hovering, you can run this code to land it. 

% Or you have to catch the drone and turn over it to stop its engines.

% controlChannel = udp('192.168.1.1', 5556, 'LocalPort', 5556);
% stateChannel = udp('192.168.1.1', 5554, 'LocalPort', 5554);
% 
% fopen(controlChannel);
% fopen(stateChannel);
% 
% 
% *************************************
% *  Authors:
%    Kun Zhang (dabiezu@gmail.edu)
%    Pieter J. Mosterman (pmosterman@yahoo.com) *
% *************************************
% 


%  normal landing
Sig = sprintf('AT*REF=%d,290717696\r',tic); %(00010001010101000000000000000000)

% emergency landing
% stop the drone engine
% Sig = sprintf('AT*REF=%d,290717952\r',tic); %(00010001010101000000000100000000)
fprintf(controlChannel, Sig);

