% function [SequenceNumber] = Land(SequenceNumber, controlChannel,stateChannel)
% This function makes the drone take a normal landing.
% 
%                   < input >
% 1. SequenceNumber is the sequence number that used in current operation.(integer > 0)
% 2. controlChannel is the UDP port for sending control command. (192.168.1.1:5556)
% 3. stateChannel is the UDP port for inquiring state. (192.168.1.1:5554)
%                   < output >
% 1. SequenceNumber is the sequence number that can be directly used by next operation
% 
% 
% *************************************
% *  Authors:
%    Kun Zhang (dabiezu@gmail.edu)
%    Pieter J. Mosterman (pmosterman@yahoo.com) *
% *************************************
% 
function [SequenceNumber] = Land(SequenceNumber, controlChannel,stateChannel)

disp('Try to land ...');
Sig = sprintf('AT*REF=%d,290717696\r',SequenceNumber);% (290717696: 00010001010101000000000000000000)
try
    fprintf(controlChannel, Sig);
catch
end

SequenceNumber = SequenceNumber + 1;
count = 0;
while count <= 5
    [DroneState, ~, SequenceNumber] = Ask4DroneState (SequenceNumber, controlChannel, stateChannel,0);
    if length(DroneState) == 32
        if DroneState(32) == 0
            disp('Land successfully ... ');
            break;
        else
            Sig = sprintf('AT*REF=%d,290717696\r',SequenceNumber);% (290717696: 00010001010101000000000000000000)
            try
                fprintf(controlChannel, Sig);
            catch
            end
            SequenceNumber = SequenceNumber + 1;
            disp('Try to land ...');
            pause(2);
        end
    end
    count = count + 1;
end

end