%mypi = raspi('10.10.10.101','pi','raspberry') %for Maxwell
clear
cam = 0;

mypi = raspi('10.10.10.102','pi','chirawat') %for Elliot
cam = cameraboard(mypi,'Resolution','160x120');

%%%create memory map
myData = uint8(1)';
fileID = fopen('state.dat','w');
fwrite(fileID, myData,'uint8');
fclose(fileID);

m = memmapfile('state.dat');
m.Writable = true;
%found = m.Data
%%%

orientation = 0;
x_current = .4;
y_current = .4;
theta_current = 90;
distance_traveled = 0;
%20 ticks on an encoder
%66.56 mm wheel diameter
Diameter = .06434; %meters
Length_Btw_Wheels = .14529; %meters

%GPIO for Right motor 27 and 22
configurePin(mypi,27,'PWM')

%GPIO for Left Motor GPIO 23 and 24
configurePin(mypi,23,'PWM')

%147.93 mmlength between wheels
%GPIO for Right (YELLOW WIRE) Encoder is 21
configurePin(mypi,21,'DigitalInput')
%GPIO for Left (GREEN WIRE) Encoder is 20
configurePin(mypi,20,'DigitalInput')


hold on
%%%%%%%%%%%%%%%%%%%%%BEGIN ALGORITHM HERE%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%queue starting location
all_samples = [];
all_samples = [all_samples; x_current y_current];
%immediately generate two random points
%rotation =[cosd(theta_current) -sind(theta_current); sind(theta_current) cosd(theta_current)];
sampled_point = [rand(2,1)*.055 rand(2,1)*.055];%*rotation
x_sample = x_current + sampled_point(:,1); %add samples to current position
y_sample = y_current + sampled_point(:,2);
sample = [x_sample y_sample];
all_samples = [all_samples; sample]
counter = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while 1
    %%%%%%%%%%%%rotate to node based on cost function%%%%%%%%%%%%%%%%%%%%%%%%%%
    %calculate Euclidean Distance as cost function
    A = sqrt((all_samples(end-2,1) - all_samples(end-1,1)).^2 + (all_samples(end-2,2) - all_samples(end-1,1)).^2);
    B = sqrt((all_samples(end-2,1) - all_samples(end,1)).^2 + (all_samples(end-2,2) - all_samples(end,1)).^2);
    if A>B
        %find angle to rotate to
        desired_angle = atan2d(all_samples(end-2,2) - all_samples(end,2),all_samples(end-2,1) - all_samples(end,1))
        distance_to_travel = B;
    else
        desired_angle = atan2d(all_samples(end-2,2) - all_samples(end-1,2),all_samples(end-2,1) - all_samples(end-1,1));
        distance_to_travel = A;
    end
    [distance_traveled_turn, theta_current_turn] = turn(desired_angle + 180,theta_current,mypi,m,cam);
    pause(2) %add pause for stability
    %%%%%%%%%%%%%%%%drive_straight until destination reached%%%%%%%%%%%%%%%%%%%
    
    [distance_current, theta_current_drive] = drive_straight(mypi,distance_to_travel,m,cam);
    pause(2)
    %%%%%%%%%%%%%%%%%%pass in correct orientation%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    theta_current = theta_current + -theta_current_turn;% + theta_current_drive;
    if theta_current >=360
        theta_current = theta_current - 360;
    elseif theta_current <= 0
        theta_current = theta_current + 360;
    end
    %record current position
    [x_current, y_current, theta_current] = position(x_current,y_current,theta_current,distance_current);
    %%%%notice that adding current pose then the two points mimics starting
    %%%%conditions
    all_samples = [all_samples; x_current y_current]; %update sampled list with current pos
    %circle represents the robot's current position
%%%%%%%%%%%%%%%%%%%%%%%%%%DRAW ROBOT POSE%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    %h2 = circle(x_current,y_current,.14)
    %drawnow;
    %delete(h2)
    
    grid on
    axis([0 .600 0 .600])
    %run RRT to select traversable nodes
%%%%%%%%%%%%%%%%%%%%%%%%%%RUN RRT%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [x_new y_new] = RRT_Drive(x_current,y_current,theta_current,all_samples);
    all_samples = [all_samples; x_new y_new]; %update sampled list
    %plot the searched area (convex hull) for visualization
    h3 = plot(all_samples(convhull(all_samples),1),all_samples(convhull(all_samples),2),'r-');
    h3.XDataSource = 'all_samples(convhull(all_samples),1)';
    h3.YDataSource = 'all_samples(convhull(all_samples),2)';
    refreshdata
    
    if counter == 8
        break
    end
    counter = counter + 1;
    
end


writePWMDutyCycle(mypi, 27, 0);
writePWMDutyCycle(mypi, 23, 0);
