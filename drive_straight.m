function [distance_traveled, orientation_traveled] = drive_straight(mypi,distance,m,cam)
configurePin(mypi,23,'PWM');
configurePin(mypi,27,'PWM');
display('DRIVE DRIVE DRIVE')
kL = .02;
kR = .002;
Ki = .1;
%start at zero to not measure until first tick is hit
cam_state = 0;
stateR = 0;
stateL = 0;
%start total tick counts at 0
counterR_input = 0;
counterL_input = 0;
counterR_old = 0;
counterL_old = 0;
counterR_new = counterR_old;
counterL_new = counterL_old;

DiameterEncoder = .02640; %meters %26.40mm encoders
DiameterWheel = .06434;
perimeter = DiameterWheel * pi;
distance_traveled = 0;
DistAvg = 0;
step_distance = [];
step_orientation = [];
tic %begin timer

%set a velocity for right wheel
writePWMVoltage(mypi,23,1.4);
%writePWMVoltage(mypi,27,1.2);
writePWMVoltage(mypi, 27, 2);

Rinput_voltage = 1;
while 1
    counterR_input = 0;
    counterL_input = 0; %reset count
    while 1
        %measures encoder tics per unit time
        [counterR_input, counterL_input, stateR, stateL] = encoder_func(mypi,counterR_input,counterL_input,stateR,stateL);
        [counterR_input, counterL_input, stateR, stateL] = encoder_func(mypi,counterR_input,counterL_input,stateR,stateL);
        
        %use camera to check for any instance of target
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        cam_state = DetectObject(cam);
        if cam_state == 1 %write to memory to show that robot has found the target
            writePWMVoltage(mypi,23,0);
            writePWMVoltage(mypi,27,0);
            m.Data = uint8(2);
            error('TARGET FOUND')
        end
        %check to see if other robot found the target
        if m.Data == 2
            writePWMVoltage(mypi,23,0);
            writePWMVoltage(mypi,27,0);
            error('TARGET FOUND BY OTHER BOT')
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %new ticks become old ticks
        if toc >= .1 %higher number adds stability; measure new total tick count every x seconds
            counterR_new = counterR_input + counterR_old;
            counterL_new = counterL_input + counterL_old;
            deltaTicR = counterR_new - counterR_old; %- counterR; %calculate delta tick; used to find total distance traveled
            deltaTicL = counterL_new - counterL_old; % - counterL;
            
            %grab measurements for x time frame
            DistR = 2*pi*(DiameterWheel/2)*deltaTicR/20; %D = 2*pi*R*DeltaTick/N
            DistL = 2*pi*(DiameterWheel/2)*deltaTicL/20;
            DistAvg = (DistR + DistL)/2;
            orientation = rad2deg((DistL - DistR)/.14);
            
            step_distance = [step_distance DistAvg];
            step_orientation = [step_orientation orientation];
            %We sum total distance traveled here
            distance_traveled = sum(step_distance);
            orientation_traveled = sum(step_orientation);
            %%%%%%%%%%%%%%HERE%%%%%%%
            
            %measure rotational velocity of left wheel
            Wr = DistR/(DiameterWheel/2*toc);
            Wl = DistL/(DiameterWheel/2*toc);
            
            
            error_WH = Wl - Wr;
            integral_errorWH =  toc*error_WH;
            
            %slave controller
            if abs(error_WH) > .1 & Rinput_voltage > 1.5
                Rinput_voltage = Rinput_voltage + error_WH*kR; + integral_errorWH*Ki;
                writePWMVoltage(mypi, 23, Rinput_voltage);
            end
            
            
            counterR_old = counterR_new; %new becomes old ticks
            counterL_old = counterL_new;
            cam_state = DetectObject(cam)
            if cam_state == 1 %write to memory to show that robot has found the target
                writePWMVoltage(mypi,23,0);
                writePWMVoltage(mypi,27,0);
                m.Data = uint8(2);
                error('TARGET FOUND')
            end
            %check to see if other robot found the target
            if m.Data == 2
                writePWMVoltage(mypi,23,0);
                writePWMVoltage(mypi,27,0);
                error('TARGET FOUND BY OTHER BOT')
            end
            
            tic;
            break
        end
        
        
        
    end
    
    if distance_traveled >= distance/4
        writePWMVoltage(mypi,23,0);
        writePWMVoltage(mypi,27,0);
        break
    end
    
end


end