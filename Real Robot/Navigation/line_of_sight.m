function visibility = line_of_sight(observerState, currentTargetNode, modifiedMap)
% Functions that checks if the currentTargetNode is visible from the current obcserver 

%% First we need to initialise an empty array for the distance
distance_array = zeros(1,size(modifiedMap,1));
i = 0;

%% from the given endpoint we can extract the x and y coordinates
currentTarget_X = currentTargetNode(1);
currentTarget_Y = currentTargetNode(2);

%% from the given obser node we can extract the x and y coordinates
observerX = observerState(1);
observerY = observerState(2);

% We then need to loop through the modified map
for k = 1:1:size(modifiedMap,1)
    
    if k < size(modifiedMap,1)        
        % below we retrieve the two points where the wall ends
        point_1 = [modifiedMap(k,1);modifiedMap(k,2)];
        x_1 = point_1(1);
        y_1 = point_1(2);      
        point_2 = [modifiedMap(k+1,1);modifiedMap(k+1,2)];
        x_2 = point_2(1);
        y_2 = point_2(2);        
    elseif k == size(modifiedMap,1)
        point_1 = [modifiedMap(k,1);modifiedMap(k,2)];
        x_1 = point_1(1);
        y_1 = point_1(2);
        point_2 = [modifiedMap(1,1);modifiedMap(1,2)];
        x_2 = point_2(1);
        y_2 = point_2(2);        
    end 
    
    %% Next we need to compute the beam direction vector along with the
    % direction vecotr of the wall
    beamDirectionV = [(currentTarget_X - observerX) ; (currentTarget_Y - observerY)]; % beam direction vector
    wallDirectionV = [(x_2-x_1); (y_2-y_1)]; % wall direction vector
    
    
    % Then we need to look for any line intersections
    intersectionCheck = (dot(beamDirectionV,wallDirectionV))/( norm(beamDirectionV)*norm(wallDirectionV));
    
    
    
    % no intersections occur if the lines themselves are parallel
    if intersectionCheck ~= 1 && intersectionCheck ~= -1 
        
        
        
        % Next we find distance, p, from the observer to the wall 
        p_calculationNumerator = (x_2 - x_1)*(y_1 - observerY) - (y_2 - y_1)*(x_1 - observerX);
        p_calculationDenominator = (x_2 - x_1)*(currentTarget_Y - observerY) - (y_2 - y_1)*(currentTarget_X - observerX);
        p = p_calculationNumerator/p_calculationDenominator;

        
       
        if p >= 0   % if this distance p is >= 0 then the observer is indeed facing a wall            
            if (y_2 - y_1) == 0 % if the wall is horizontal   
                % we need to find the intersection, q, with the wall vector
                q = ( observerX - x_1 + p*(currentTarget_X - observerX) )/(x_2 - x_1);         
            elseif (x_2 - x_1) == 0 % else if the wall is vertical
                % We need to determine q along the wall vector
                q = ( observerY - y_1 + p*(currentTarget_Y - observerY ) )/(y_2 - y_1);          
            else % if the wall is not vertical or horizontal                
                q = ( observerY - y_1 + p*(currentTarget_Y - observerY ) )/(y_2 - y_1);                 
            end            

            if q >= 0 && q <= 1 %if intersections occur where all ends meet
                i = i + 1;
                distance_array(i) = p;                
            end            
        end        
    end    
end

%% extract the minimum p value from the distance array
p_min = min(distance_array(1:i));


if p_min == 0 % if the observer is positioned on a wall
    nonzero_entry_index = (distance_array(1:i) ~= 0); 
    p_min = min(distance_array(nonzero_entry_index)); 
end

if p_min < 1 % If wall is between observer and current target
    visibility = 0; % no direct line_of_sight
else % compute the midpoint coordinates (x and y) of the line connectine the observer and the target   
    midpoint_X = 0.5*(observerX + currentTarget_X);
    midpoint_Y = 0.5*(observerY + currentTarget_Y);   
    
    % we also need to ensure the computed midpoint is within the boundaries
    % of the map
    [IN ON] = inpolygon(midpoint_X,midpoint_Y,modifiedMap(:,1),modifiedMap(:,2));
    
    
    % if the path is indeed within the boundaries
    if IN == 1 || ON == 1        
        visibility = 1; % we have can see the target node         
    else        
        visibility = 0; % we cannot see the target node
    end
    
end
