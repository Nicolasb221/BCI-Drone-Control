% function [SequenceNumber] = TakeOff(SequenceNumber, controlChannel, stateChannel)
% This function makes the drone take off.
%                   < input >
% 1. SequenceNumber is the sequence number that used in current operation.(integer > 0)
% 2. controlChannel is the UDP port for sending control command. (192.168.1.1:5556)
% 3. stateChannel is the UDP port for inquiring state. (192.168.1.1:5554)
%                   < output >
% 1. SequenceNumber is the sequence number that can be directly used by next operation
%    (* if it returns -1, that means the drone failed to take off. )
% 
% Operation content:
%       (1) Inquiring the drone state.
%       (2) If the drone responds, and it is in normal state, then send take off command.
%       (3) else, reset the emergency state to normal state, and then send take off command.
%       
%       !!!!!!!!!!!!!!!!!
%       (4) Note : before sending the take off command, the function
%           sends the horizontal-plane initialization command. This means, the
%           drone must be put on a horizontal plane (pitch = 0, roll = 0)
%           before taking off. Otherwise, the drone mistakenly sets the horizontal reference.
% 
% *************************************
% *  Authors:
%    Kun Zhang (dabiezu@gmail.edu)
%    Pieter J. Mosterman (pmosterman@yahoo.com) *
% *************************************
% 
function [SequenceNumber] = TakeOff(SequenceNumber, controlChannel, stateChannel)
disp('Try to take off...')
for i = 1:10 % ask for drone state for 10 times
    [DroneState, ~, SequenceNumber] = Ask4DroneState (SequenceNumber, controlChannel, stateChannel, 0);
    if length(DroneState) == 32
        break;
    end
end

if i >= 10 && length(DroneState)<32
    FailTakeOff;
    return
else
    Sig = sprintf('AT*FTRIM=%d\rAT*REF=%d,290718208\r',SequenceNumber, SequenceNumber+1);%(00010001010101000000001000000000)
    try
        fprintf(controlChannel, Sig);
    catch
    end
    SequenceNumber = SequenceNumber + 2;
    mark = 0;
    count = 1;
    while count<10
        [DroneState, ~, SequenceNumber] = Ask4DroneState (SequenceNumber,controlChannel, stateChannel,0);
        if length(DroneState) == 32
            if DroneState(32)==1 % in flying state
                disp('in flight state');
                mark = 1;
                break;
            else
                if DroneState(1) == 1 % in ground and in emergency state
                    disp('In emergency state...')
                    disp('Set to normal state...')
                    disp('If the drone does not take off, pls clear out emergency cause, and then reconnect the drone battery.')
                    Sig = sprintf('AT*REF=%d,290718464\r',SequenceNumber);%(00010001010101000000001100000000)
                    try
                        fprintf(controlChannel, Sig);
                    catch
                    end
                    pause(1);
%                     mark = 1;
%                     break;
                else % in ground but in normal state
                    disp('try to take off again...')
                    Sig = sprintf('AT*REF=%d,290718208\r',SequenceNumber);%(00010001010101000000001000000000)
                    try
                        fprintf(controlChannel, Sig);
                    catch
                    end
                end
                SequenceNumber = SequenceNumber + 1;
            end
        end
        count = count + 1;
    end
     
if mark == 1
    disp('Take off successfully...');
else
    FailTakeOff;
end

end






function FailTakeOff
try
   fprintf(controlChannel, sprintf('AT*REF=%d,290717696\r',SequenceNumber));% take a normal land
catch
end
SequenceNumber = -1;
disp('Wireless connection problem...');
disp('Cannot take off...(re-connect the drone power probably can solve the problem).');
end



end

