% function intValue = ARDrone_FloatArg2Int(floatValue)
% Based on ARDrone SDK 1.7 Documentation. 
%  This function may be replaced by some existing matlab command ...
% float parameter is sent in int format
% For example:
% The number -0.8 is stored in memory as a 32-bit word, which is BF4CCCCD(0x) according to the IEEE-754 format.
% This 32-bit word is sent as a 32-bit integer value -1085485875.
% 
% *************************************
% *  Authors:
%    Kun Zhang (dabiezu@gmail.edu)
%    Pieter J. Mosterman (pmosterman@yahoo.com) *
% *************************************
function intValue = ARDrone_FloatArg2Int(floatValue)
hex = sprintf('%tx',floatValue);
for i=1:length(hex)
    if hex(i)=='f'
        bin((i*4)-3:i*4)=[1 1 1 1];
    elseif hex(i)=='e'
        bin((i*4)-3:i*4)=[1 1 1 0];
    elseif hex(i)=='d'
        bin((i*4)-3:i*4)=[1 1 0 1];
    elseif hex(i)=='c'
        bin((i*4)-3:i*4)=[1 1 0 0];
    elseif hex(i)=='b'
        bin((i*4)-3:i*4)=[1 0 1 1];
    elseif hex(i)=='a'
        bin((i*4)-3:i*4)=[1 0 1 0];
    elseif hex(i)=='9'
        bin((i*4)-3:i*4)=[1 0 0 1];
    elseif hex(i)=='8'
        bin((i*4)-3:i*4)=[1 0 0 0];
    elseif hex(i)=='7'
        bin((i*4)-3:i*4)=[0 1 1 1];
    elseif hex(i)=='6'
        bin((i*4)-3:i*4)=[0 1 1 0];
    elseif hex(i)=='5'
        bin((i*4)-3:i*4)=[0 1 0 1];
    elseif hex(i)=='4'
        bin((i*4)-3:i*4)=[0 1 0 0];
    elseif hex(i)=='3'
        bin((i*4)-3:i*4)=[0 0 1 1];
    elseif hex(i)=='2'
        bin((i*4)-3:i*4)=[0 0 1 0];
    elseif hex(i)=='1'
        bin((i*4)-3:i*4)=[0 0 0 1];
    elseif hex(i)=='0'
        bin((i*4)-3:i*4)=[0 0 0 0];
    end
end
intValue = 0;
for i = 2: length(bin)
    intValue = intValue + bin(i)*(2^(length(bin)-i));
end
if bin(1) == 1
    intValue = intValue - 2^(length(bin)-1);
end
intValue = int32(intValue);
end



