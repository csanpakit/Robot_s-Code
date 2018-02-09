function [new_right, new_left, stateR, stateL] = encoder_func(mypi,Right_count,Left_count,stateR,stateL)

%obtain encoder readings
R_Encoder = readDigitalPin(mypi,21);
L_Encoder = readDigitalPin(mypi,20);

if R_Encoder == 1 && stateR == 1
    new_right = Right_count+1; %track total counter for encoder
    stateR = 0;
elseif R_Encoder == 0
    stateR = 1;%set state to one to allow measurement of next tick
    new_right = Right_count;
else
    new_right = Right_count;
    stateR = stateR;%we assign these in case return values are not assigned
end

if L_Encoder == 1 && stateL == 1
    new_left = Left_count+1;
    stateL = 0;
elseif L_Encoder == 0
    stateL = 1;
    new_left = Left_count;
else
    new_left = Left_count;
    stateL = stateL;
end

end