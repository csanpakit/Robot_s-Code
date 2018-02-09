function [distance_traveled orientation_traveled] = turn(desired_orientation,current_orientation,mypi, m,cam)
%TURN TURN TURN
%error angle is difference between the +x
%we are turning LEFT with this setup, meaning we record right wheel

%set initial states
voltage_input = 1;
voltK = .001;

cam_state = 0;
state = 0;
stateR = 0;
stateL = 0;
step_distance = [];
step_orientation = [];


counterR_input = 0;
counterL_input = 0;

DiameterEncoder = .02640; %meters %26.40mm encoders
DiameterWheel = .06434;

if current_orientation > desired_orientation
    %required_angle = abs(deg2rad(360 - current_orientation + desired_orientation));
    current_orientation
    desired_orientation
    required_angle = deg2rad(current_orientation - desired_orientation)
    DistL = (DiameterWheel/2)*required_angle;
    display('TURN RIGHT')
    deltaTicR = 4*(DistL*20)/(2*pi*(DiameterWheel/2));
    state = 1; %used this to spin left wheel
    
else
    current_orientation
    desired_orientation
    required_angle = abs(deg2rad(current_orientation - desired_orientation))
    DistR = (DiameterWheel/2)*required_angle;
    deltaTicR = 4*(DistR*20)/(2*pi*(DiameterWheel/2));
    display('TURN LEFT')
    
    
    %changed this to have robot turn the shorter way
end



%state
%decide which wheel to spin, open-loop control


%%%
while 1
    tic
    %measures encoder tics per unit time
    [counterR_input, counterL_input, stateR, stateL] = encoder_func(mypi,counterR_input,counterL_input,stateR,stateL);
    counterR_input %reads left wheel
    counterL_input %reads right wheel
    if state == 1
        writePWMVoltage(mypi,27,voltage_input); %spins left wheel
        writePWMVoltage(mypi,23,0);
        counterR_input;
        [counterR_input, counterL_input, stateR, stateL] = encoder_func(mypi,counterR_input,counterL_input,stateR,stateL);
        
    else
        writePWMVoltage(mypi,23,voltage_input); %spins right wheel
        writePWMVoltage(mypi,27,0);
        [counterR_input, counterL_input, stateR, stateL] = encoder_func(mypi,counterR_input,counterL_input,stateR,stateL);
        
    end
    %%%%%%%%%%%%%%%%Guarantees to increment until there is w
    deltaTicR_move = counterR_input; %- counterR; %calculate delta tick; used to find total distance traveled
    deltaTicL_move = counterL_input; % - counterL;
    
    %grab measurements for x time frame
    DistR = 2*pi*(DiameterWheel/2)*deltaTicR_move/20; %D = 2*pi*R*DeltaTick/N
    DistL = 2*pi*(DiameterWheel/2)*deltaTicL_move/20;
    
    %measure angular velocity
    Wr = DistR/(DiameterWheel/2*toc);
    Wl = DistL/(DiameterWheel/2*toc);
    
    [counterR_input, counterL_input, stateR, stateL] = encoder_func(mypi,counterR_input,counterL_input,stateR,stateL);
    
    %we try to slowly excite the motor to move
    if deltaTicR_move < 2 & state == 1
        voltage_input = voltage_input +.012;
    elseif deltaTicL_move < 2
        voltage_input = voltage_input + .012;
    end
    
    if voltage_input > 2.2
        voltage_input = 1.8;
    end
    
    %%%%%%%%%%%%%%%%
    %use camera to check for any instance of target
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    cam_state = DetectObject(cam);
    [counterR_input, counterL_input, stateR, stateL] = encoder_func(mypi,counterR_input,counterL_input,stateR,stateL);
    
    if cam_state == 1
        writePWMVoltage(mypi,23,0);
        writePWMVoltage(mypi,27,0);
        
        m.Data = uint8(2);
        
        error('TARGET FOUND')
    end
    
    if m.Data == 2
        writePWMVoltage(mypi,23,0);
        writePWMVoltage(mypi,27,0);
        error('TARGET FOUND BY OTHER BOT')
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %if ticks match, then we are done
    if counterL_input >= round(deltaTicR) || counterR_input >= round(deltaTicR)
        
        writePWMVoltage(mypi,23, 0);
        writePWMVoltage(mypi,27, 0);
        deltaTicR = counterR_input; %- counterR; %calculate delta tick; used to find total distance traveled
        deltaTicL = counterL_input; % - counterL;
        
        %grab measurements for x time frame
        DistR = 2*pi*(DiameterWheel/2)*deltaTicR/20; %D = 2*pi*R*DeltaTick/N
        DistL = 2*pi*(DiameterWheel/2)*deltaTicL/20;
        
        %measure angular velocity
        Wr = DistR/(DiameterWheel/2*toc);
        Wl = DistL/(DiameterWheel/2*toc);
        
        
        %calculate distance + orientation
        distance_traveled = (DistR + DistL)/2;
        orientation_traveled = rad2deg((DistL - DistR)/.13246);        
        
        break
    end
    [counterR_input, counterL_input, stateR, stateL] = encoder_func(mypi,counterR_input,counterL_input,stateR,stateL);
    
    
    
end


end
%%%
