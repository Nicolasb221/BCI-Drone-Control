% [status, output] = Interpret_NavData(NDS)
% This function interprets the NavData
% the interpretation is performed for firmware 2.3.3 platform
% if you downgrade or upgrade the Drone firmware, you need to make sure if the structure of
% NavData changes.
% --------------------- input & output -------------------
% NDS (Nav Data Sample) is a response from Drone. It must be a 12*1,24*1 or 500*1 array. 
% the function returns [status, output]
% if status == 0, then the status is invalid
% if output == 0, then the output is invalid
% 
% a valid status is an arry consisting of 32 elements in the index order
% [31,30,29,28,...1,0]
% meaning of each element can be found at the end of the comment.
%  e.g. : status(32) ---> indicates landing/flying state
%
% a valid output is an 8*1 array in which the structure looks like:
% [batteryLevel unit: %        1
%  pitch        unit: angle    2
%  roll         unit: angle    3
%  yaw          unit: angle    4
%  altitude     unit: meter    5
%  Vx           unit: m/s      6
%  Vy           unit: m/s      7
%  Vz];         unit: m/s      8
%
%----------------------------------------------------------
% // Define masks for ARDrone state
% // 31 0
% // x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x -> state
% // | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | |
% //00 | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | FLY MASK : (0) ardrone is landed, (1) ardrone is flying
% //01 | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | VIDEO MASK : (0) video disable, (1) video enable
% //02 | | | | | | | | | | | | | | | | | | | | | | | | | | | | | VISION MASK : (0) vision disable, (1) vision enable
% //03 | | | | | | | | | | | | | | | | | | | | | | | | | | | | CONTROL ALGO : (0) euler angles control, (1) angular speed control
% //04 | | | | | | | | | | | | | | | | | | | | | | | | | | | ALTITUDE CONTROL ALGO : (0) altitude control inactive (1) altitude control active
% //05 | | | | | | | | | | | | | | | | | | | | | | | | | | USER feedback : Start button state
% //06 | | | | | | | | | | | | | | | | | | | | | | | | | Control command ACK : (0) None, (1) one received
% //07 | | | | | | | | | | | | | | | | | | | | | | | | Trim command ACK : (0) None, (1) one received
% //08 | | | | | | | | | | | | | | | | | | | | | | | Trim running : (0) none, (1) running
% //09 | | | | | | | | | | | | | | | | | | | | | | Trim result : (0) failed, (1) succeeded
% //10 | | | | | | | | | | | | | | | | | | | | | Navdata demo : (0) All navdata, (1) only navdata demo
% //11 | | | | | | | | | | | | | | | | | | | | Navdata bootstrap : (0) options sent in all or demo mode, (1) no navdata options sent
% //12 | | | | | | | | | | | | | | | | | | | | Motors status : (0) Ok, (1) Motors Com is down
% //13 | | | | | | | | | | | | | | | | | |
% //14 | | | | | | | | | | | | | | | | | Bit means that there's an hardware problem with gyrometers
% //15 | | | | | | | | | | | | | | | | VBat low : (1) too low, (0) Ok
% //16 | | | | | | | | | | | | | | | VBat high (US mad) : (1) too high, (0) Ok
% //17 | | | | | | | | | | | | | | Timer elapsed : (1) elapsed, (0) not elapsed
% //18 | | | | | | | | | | | | | Power : (0) Ok, (1) not enough to fly
% //19 | | | | | | | | | | | | Angles : (0) Ok, (1) out of range
% //20 | | | | | | | | | | | Wind : (0) Ok, (1) too much to fly
% //21 | | | | | | | | | | Ultrasonic sensor : (0) Ok, (1) deaf
% //22 | | | | | | | | | Cutout system detection : (0) Not detected, (1) detected
% //23 | | | | | | | | PIC Version number OK : (0) a bad version number, (1) version number is OK
% //24 | | | | | | | ATCodec thread ON : (0) thread OFF (1) thread ON
% //25 | | | | | | Navdata thread ON : (0) thread OFF (1) thread ON
% //26 | | | | | Video thread ON : (0) thread OFF (1) thread ON
% //27 | | | | Acquisition thread ON : (0) thread OFF (1) thread ON
% //28 | | | CTRL watchdog : (1) delay in control execution (> 5ms), (0) control is well scheduled // Check frequency of control loop
% //29 | | ADC Watchdog : (1) delay in uart2 dsr (> 5ms), (0) uart2 is good // Check frequency of uart2 dsr (com with adc)
% //30 | Communication Watchdog : (1) com problem, (0) Com is ok // Check if we have an active connection with a client
% //31 Emergency landing : (0) no emergency, (1) emergency
% 
% *************************************
% *  Authors:
%    Kun Zhang (dabiezu@gmail.edu)
%    Pieter J. Mosterman (pmosterman@yahoo.com) *
% *************************************
% 
function [status, output] = Interpret_NavData(NDS)

%%
% programming...
% several fixed offset

DroneStateOffSet = 5;
SequenceNoOffSet = 9;
% ---- temporarily comment the following 2 lines ...
% HeadOffSet = 1;
% VisionFlagOffSet = 13;

% the first option block follows exactly the VisionFlag block
% ---- temporarily comment the following 2 lines ...
% Option_id_OS = 17; % OS is OffSet
% Option_size_OS = 19;

% from byte #20 - #24: the meaning is unknow.
% then: the data part follows 
data_length = 4;
Battery_OS  = 25; % data element has a length of 4 bytes
Pitch_OS    = 29;
Roll_OS     = 33;
Yaw_OS      = 37;
Altitude_OS = 41;
Vx_OS       = 45;
Vy_OS       = 49;
Vz_OS       = 53;


if length(NDS) == 24
    % navData.Mode.BOOTSTRAP
    status = detectDroneState(NDS(DroneStateOffSet:SequenceNoOffSet-1));
    output = 0;
elseif length(NDS) >= 299
    % navData.MOde.DEMO
    % ------------------- start -----------------------
    
    % process battery info:
      batteryLevel = reArrange_and_decodeValue(NDS(Battery_OS:Battery_OS+data_length-1),1); % unit: %
      pitch = reArrange_and_decodeValue(NDS(Pitch_OS:Pitch_OS+data_length-1),0)/1000; % unit: deg
      roll = reArrange_and_decodeValue(NDS(Roll_OS:Roll_OS+data_length-1),0)/1000; % unit: deg
      yaw = reArrange_and_decodeValue(NDS(Yaw_OS:Yaw_OS+data_length-1),0)/1000; % unit: deg
      altitude = reArrange_and_decodeValue(NDS(Altitude_OS:Altitude_OS+data_length-1),1)/1000; % unit: meter
      Vx = reArrange_and_decodeValue(NDS(Vx_OS:Vx_OS+data_length-1),0)/1000; % unit: meter/s
      Vy = reArrange_and_decodeValue(NDS(Vy_OS:Vy_OS+data_length-1),0)/1000; % unit: meter/s
      Vz = reArrange_and_decodeValue(NDS(Vz_OS:Vz_OS+data_length-1),0)/1000; % unit: meter/s
      output = [batteryLevel
                 pitch
                 roll
                 yaw
                 altitude
                  Vx
                  Vy
                  Vz];
      status = detectDroneState(NDS(DroneStateOffSet:SequenceNoOffSet-1));
    % ------------------- end -----------------------
else
    % the length could be 12 
    % neglect this data
    status = 0;
    output = 0;
end

function res = reArrange_and_decodeValue(input, mark)
% if mark = 1; then output a int
% else output the float value which is defined by SDK description.
% the input is an array (1*4) 
% e.g. : NDS(Battery_OS:Battery_OS+data_length-1)
    hex_value = dec2hex(input);
    hex_value = [hex_value(4,:),hex_value(3,:),hex_value(2,:),hex_value(1,:)];
    if mark ==1
        res = hex2dec(hex_value);
    else
        res = typecast(uint32(hex2dec(hex_value)),'single');
    end
end

function bin = detectDroneState(input)
    hex = dec2hex(input);
    % put the hex_value in the order [31, 30......0]
    hex = [hex(4,:),hex(3,:),hex(2,:),hex(1,:)];
for i=1:length(hex)
    if hex(i)=='F'
        bin((i*4)-3:i*4)=[1 1 1 1];
    elseif hex(i)=='E'
        bin((i*4)-3:i*4)=[1 1 1 0];
    elseif hex(i)=='D'
        bin((i*4)-3:i*4)=[1 1 0 1];
    elseif hex(i)=='C'
        bin((i*4)-3:i*4)=[1 1 0 0];
    elseif hex(i)=='B'
        bin((i*4)-3:i*4)=[1 0 1 1];
    elseif hex(i)=='A'
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

end

end
