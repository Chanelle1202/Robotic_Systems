function [ angle, distance ] = get_angle_dist( start_point, end_point, external_boundaries, robot_size )
% Function which calculates and returns the length and angle of the path
% between points waypoints p1(x1, y1) and p2(x2, y2). This allows the
% robot's estimated path from start_point to end_point to be identified, so
% that the required translation and rotation can be performed by the robot.

%% Estimate a path from a given start point to a given end point
%tic % time taken to estimate a path
inflated_boundaries = boundary_inflation(external_boundaries, robot_size);
waypoint_coordinates = pathfinder(start_point, end_point, inflated_boundaries);
%toc

    x1 = waypoint_coordinates(1,1);
    y1 = waypoint_coordinates(1,2);
    x2 = waypoint_coordinates(2,1);
    y2 = waypoint_coordinates(2,2);
   
    angle = atan2d(y2-y1,x2-x1);
    angle = (angle * pi / 180);
    while angle > pi*2
        angle = angle - (pi*2);
    end
     while angle < 0
        angle = angle + (pi*2);
    end
   
    distance = sqrt ( ((y2-y1)^2) + ((x2-x1)^2) );
    if distance > 10
        distance = distance / 2;
    end
    
end

