function inflatedBoundaries = boundary_inflation(map, robotSize)
% Function returns a map with inflated internal boundaries using robotSize

%% First we need to initialise empty arrays to create the inflated boundaries
midpoint_x = zeros(1, size(map,1));
midpoint_y = zeros(1, size(map,1));
shiftedMidpoint_x_a = zeros(1, size(map,1));
shiftedMidpoint_y_a = zeros(1, size(map,1));
shiftedMidpoint_x_b = zeros(1, size(map,1));
shiftedMidpoint_y_b = zeros(1, size(map,1));
insidePoint_x = zeros(1, size(map,1));
insidePoint_y = zeros(1, size(map,1));
orientation = zeros(1, size(map,1));
inflatedBoundaries = zeros(size(map,1), 2);

%% Next we need to extract vertices from adjacent map walls
for currentWall_index = 1:1:size(map,1)    
    if currentWall_index < size(map,1)        
        % we need to extract the 2 points where the ends of the walls meet       
        point_1 = [map(currentWall_index,1);map(currentWall_index,2)];
        x_1 = point_1(1);
        y_1 = point_1(2);
        
        point_2 = [map(currentWall_index+1,1);map(currentWall_index+1,2)];
        x_2 = point_2(1);
        y_2 = point_2(2);
        
    elseif currentWall_index == size(map,1)    
        point_1 = [map(currentWall_index,1);map(currentWall_index,2)];
        x_1 = point_1(1);
        y_1 = point_1(2);
        
        point_2 = [map(1,1);map(1,2)];
        x_2 = point_2(1);
        y_2 = point_2(2);        
    end
    
    % Next we calculate the mindpoints
    midpoint_x(currentWall_index) = (x_1 + x_2)/2;
    midpoint_y(currentWall_index) = (y_1 + y_2)/2;
    
    % Then we compute the current pose of the map wall
    orientation(currentWall_index) = atan( (y_2 - y_1)/(x_2 - x_1) );
    
    % Next we add the inflated midpoints of the boundaries to the current
    % walls of the map
    shiftedMidpoint_x_a(currentWall_index) = midpoint_x(currentWall_index) + robotSize*cos(orientation(currentWall_index) + pi/2);
    shiftedMidpoint_y_a(currentWall_index) = midpoint_y(currentWall_index) + robotSize*sin(orientation(currentWall_index) + pi/2);
    shiftedMidpoint_x_b(currentWall_index) = midpoint_x(currentWall_index) + robotSize*cos(orientation(currentWall_index) - pi/2);
    shiftedMidpoint_y_b(currentWall_index) = midpoint_y(currentWall_index) + robotSize*sin(orientation(currentWall_index) - pi/2);
    
    % Next we need to ensure that the inflated boundaries are not ontop or
    % outside the map walls
    [IN_a ON_a] = inpolygon(shiftedMidpoint_x_a(currentWall_index), shiftedMidpoint_y_a(currentWall_index), map(:,1),map(:,2));
    [IN_b ON_b] = inpolygon(shiftedMidpoint_x_b(currentWall_index), shiftedMidpoint_y_b(currentWall_index), map(:,1),map(:,2));
    
    
    
    if IN_a == 1 && ON_a == 0 && IN_b == 1 && ON_b == 0 % if the inflated midpoints are inside the walls                  
        observer_state = [midpoint_x(currentWall_index); midpoint_y(currentWall_index)];
        current_target_node_a = [shiftedMidpoint_x_a(currentWall_index);shiftedMidpoint_y_a(currentWall_index)];
        current_target_node_b = [shiftedMidpoint_x_b(currentWall_index);shiftedMidpoint_y_b(currentWall_index)];
        
        % Next we check if the inflated midpoint is visible from the map
        % wall midpoint 
        visibility_a = line_of_sight(observer_state, current_target_node_a, map);
        visibility_b = line_of_sight(observer_state, current_target_node_b, map);
        
        % Now we discard any midpoints which dont have a direct line of
        % sight of the wall
        if visibility_a == 1            
            insidePoint_x(currentWall_index) = shiftedMidpoint_x_a(currentWall_index);
            insidePoint_y(currentWall_index) = shiftedMidpoint_y_a(currentWall_index);         
        elseif visibility_b == 1            
            insidePoint_x(currentWall_index) = shiftedMidpoint_x_b(currentWall_index);
            insidePoint_y(currentWall_index) = shiftedMidpoint_y_b(currentWall_index);            
        end
        
    elseif IN_a == 1 && ON_a == 0 % If inflated midpoint, a, is isnide, yet not ontop of the wall        
        insidePoint_x(currentWall_index) = shiftedMidpoint_x_a(currentWall_index);
        insidePoint_y(currentWall_index) = shiftedMidpoint_y_a(currentWall_index);        
    elseif IN_b == 1 && ON_b == 0        
        insidePoint_x(currentWall_index) = shiftedMidpoint_x_b(currentWall_index);
        insidePoint_y(currentWall_index) = shiftedMidpoint_y_b(currentWall_index);        
    end    
end

%% Finally, we can calculate the vertices of he inflated boundaries
for currentWall_index = 1:1:size(map,1)    
    if currentWall_index < size(map,1)        
        x_a = insidePoint_x(currentWall_index);
        y_a = insidePoint_y(currentWall_index);
        theta_a = orientation(currentWall_index);
        
        x_b = insidePoint_x(currentWall_index + 1);
        y_b = insidePoint_y(currentWall_index + 1);
        theta_b = orientation(currentWall_index + 1);        
    elseif currentWall_index == size(map,1)        
        x_a = insidePoint_x(currentWall_index);
        y_a = insidePoint_y(currentWall_index);
        theta_a = orientation(currentWall_index);
        
        x_b = insidePoint_x(1);
        y_b = insidePoint_y(1);
        theta_b = orientation(1);        
    end
    
    r_b_numerator = y_b - y_a - (x_b - x_a)*tan(theta_a);
    r_b_denominator = cos(theta_b)*tan(theta_a) - sin(theta_b);
    
    inflatedBoundaries(currentWall_index,1) = x_b + r_b_numerator/r_b_denominator*cos(theta_b);
    inflatedBoundaries(currentWall_index,2) = y_b + r_b_numerator/r_b_denominator*sin(theta_b);
    
end
