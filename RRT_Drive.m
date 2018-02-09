function [x_nodes y_nodes] = RRT_Drive(x_current,y_current,theta_current,all_samples)

%size of the space 750 mm x 750 mm
counter = 0;
x_nodes = 0;
y_nodes = 0;
boundary_x = .5;
boundary_y = .5;
num_nodes = 2; %set number of points to sample
%rotation matrix rotates points CCW by theta degree, %check this
%rotation is ccw if +theta , cw if -theta
if theta_current > 180
    theta_current = -theta_current;
end
rotation =[cosd(theta_current) -sind(theta_current); sind(theta_current) cosd(theta_current)];

%generate samples
sampled_point = [rand(num_nodes,1)*.055 rand(num_nodes,1)*.055]*rotation;


x_sample = x_current + sampled_point(:,1); %add samples to current position
y_sample = y_current + sampled_point(:,2);
sample = [x_sample y_sample];
%resample if sample is already in traversed region or out of bounds
%we first check if sample is within the convex hull (already searched
%region)

%check for possible points
while 1
    sample_check = inpolygon(sample(:,1),sample(:,2),all_samples(convhull(all_samples),1),all_samples(convhull(all_samples),2));
    points_possible = [];
    for ii = 1:length(sample_check)
        if sample_check(ii,1) == 0 & (sample(ii,1) > 0 & sample(ii,1) < boundary_x) & (sample(ii,2) > 0 & sample(ii,2) < boundary_y) %needs to be within bounds and out of searched region
            points_possible(ii,:) = sample(ii,:);
        end
    end
    %if length > 1, we found possible node
    if length(points_possible) > 0
        break
    else %else we resample
        %must stay within top half bounds
        sample = [rand(1,1)*boundary_x (.50 - .4).*rand(1,1)+ .4];
        %sample = [rand(1,1)*boundary_x (.4 - .1).*rand(1,1)+ .1];

        counter = counter + 1;
    end
    
end

%if we had to resample, use resampled point, else, continue with original
%used counter to check

%assign to return values
if counter > 0
    x_nodes = points_possible(1,1); %completely new region
    y_nodes = points_possible(1,2);
else
    x_nodes = points_possible(:,1);
    y_nodes = points_possible(:,2);
end
%give waypoints for robot to traverse to
%hold on
%plot(x_nodes,y_nodes,'b*','LineWidth',2)


end