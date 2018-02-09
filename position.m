function [xprime yprime thetaprime] = position(x_current,y_current,theta_current,D_traveled)

xprime = x_current + D_traveled*cosd(theta_current);
yprime = y_current + D_traveled*sind(theta_current);
thetaprime = theta_current;
end