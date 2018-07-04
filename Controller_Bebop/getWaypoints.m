function [ waypoint ] = getWaypoints(fi,A_p,A_v)
%GETWAYPOINTS Generates a list of waypoints for the ARDrone
% Each waypoint is a column vector that contains the desired position of 
% the drone, desired heading angle, and velocity desired x and y channel. The list of waypoints
% is created combining the column vectors. 
    t = 0:0.2:4*pi;
    x=A_p*cos(t+fi);
    y=A_p*sin(t+fi);
    vx=-A_v*sin(t+fi);
    vy=A_v*cos(t+fi);
    plot(x,y,'*r');
    figure();
    plot(t,vx,'b');hold on;
    plot(t,vy,'g')
    h=1.2;  % metric
    yaw=0; % rad
    nPoints=size(t,2);
    waypoint = zeros(7,nPoints);
    for i=1:nPoints
        waypoint(1,i)=i;
        waypoint(2,i)=x(i);
        waypoint(3,i)=y(i);
        waypoint(4,i)=h;
        waypoint(5,i)=yaw;
        waypoint(6,i)=vx(i);
        waypoint(7,i)=vy(i);
    end
    
    waypoint(6,1)=0;
    waypoint(7,1)=0;
    waypoint(6,nPoints)=0;
    waypoint(7,nPoints)=0;
    



   
end

