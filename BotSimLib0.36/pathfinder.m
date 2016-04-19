function visibilityPath = pathfinder(start_point, end_point, modifiedMap)

% Function which finds the shortest path from a start point to an end point 
% in a 2D arena whilst avoiding obstacles. This function uses Visibility
% Graphs elements to implement the Dijkstra's algorithm.

%% Initialize empty arrays
i_combinedNodes = zeros(size(modifiedMap,1)+2,3);

%% Create the inital set of unvisited nodes
i_combinedNodes(1,1:2) = [start_point(1), start_point(2)];
i_combinedNodes(2:size(modifiedMap,1)+1,1:2) = modifiedMap;
i_combinedNodes(size(modifiedMap,1)+2,1:2) = [end_point(1), end_point(2)];

%% Assign infinte tentative distances to all nodes excpet the initial node
i_combinedNodes(:,3) = Inf*ones(size(i_combinedNodes,1),1);

%% Determine which nodes are visible from the initial node / starting point
visibleNodesID = zeros(1,size(i_combinedNodes,1));

%% Compile a library of all neighbouring nodes which are visible from the current node
% First, initialise this library as an empty 3D array 
visibleNeighboursLibrary =  zeros(size(i_combinedNodes,1),size(i_combinedNodes,2),size(i_combinedNodes,1));

% Set all the Index values of visible node to 0
visibleIndex = 0;

for referenceNodeID = 1:size(i_combinedNodes,1)
    combinedNodes = i_combinedNodes;
    for target_ID = 1:size(i_combinedNodes,1)
        % Initialise the observer and target (endpoint) nodes
        observerState = i_combinedNodes(referenceNodeID,:);
        currentTargetNode = i_combinedNodes(target_ID,:);
        
        % This checks whether or not the target node is visible from the observer node 
        visibility = line_of_sight(observerState, currentTargetNode, modifiedMap);
        % if it is visible, visibility is set to 1
        if visibility == 1 
            % The ID of the visible node is then recorded
            visibleIndex = visibleIndex + 1;
            visibleNodesID(visibleIndex) = target_ID;
            combinedNodes(target_ID,3) = sqrt((currentTargetNode(1) - observerState(1))^2 + (currentTargetNode(2) - observerState(2))^2);
        end
    end
    
    % Compile a 3D library of all the visible neighbouring nodes with respect to a reference node
    visibleNeighboursLibrary(:,:,referenceNodeID) = combinedNodes;
end

%% Next, the unvisited nodes are intialised
unvisitedNodes = zeros(1,size(i_combinedNodes,1));

%% Next, a list of all the unvisisted nodes is compiled so that they may be assigned to the unvisited node set
for index = 1:size(i_combinedNodes)
    unvisitedNodes(index) = index;
end
shortestPath = visibleNeighboursLibrary(:,:,1);

%% We intialise the starting node to 1
shortestPath(:,4) = 1;

%% We now need to remove the starting node from the univisted node set
current_node = 1;
unvisitedNodes = setdiff(unvisitedNodes, current_node);

%% the loop below represents the reallocation of the starting node and the removal of the 
% visited nodes from the univsisted node set until the target node has been visited or the 
% conditions have been met
while size(unvisitedNodes,2) > 0
    
    % The next starting node / current node is node with the smallest cumulative distance
    cumulative_distances = shortestPath(unvisitedNodes,3);
    [~, currentNodeID_index] = min(cumulative_distances); 
    
    % The current node is then extraced
    currentNodeID = unvisitedNodes(currentNodeID_index);
    
    % This line of code removes the current node from the set of unvisited nodes
    unvisitedNodes = setdiff(unvisitedNodes, currentNodeID);
    
    for unvisited_node_index = 1:size(unvisitedNodes,2)
        
        % An univisited node is set as the next target node
        target_node_ID = unvisitedNodes(unvisited_node_index);
        
        % if the distance between the target node and the current node is less than inifinty, then visibility is assumed
        if visibleNeighboursLibrary(target_node_ID,3,currentNodeID) < Inf
            
            % the cumulative or tentaive distance for the target node is now computed
            cumulativeNode_distance = shortestPath(currentNodeID,3);
            cumulativeNodeToTarget_distance = visibleNeighboursLibrary(target_node_ID,3,currentNodeID);
            newCumulative_distance = cumulativeNode_distance + cumulativeNodeToTarget_distance;
            
            % In order to compare this value to the previous, the previous value has to be determined
            previousCumulativetoTarget_distance = shortestPath(target_node_ID,3);
            
            % if a s maller distance value  is found from the comparison
            if newCumulative_distance < previousCumulativetoTarget_distance
                
                % then we need to replace the previous value with the smaller one
                shortestPath(target_node_ID,3) = newCumulative_distance;
                shortestPath(target_node_ID,4) = currentNodeID;
                
            end
            
        end
        
    end
    
end


%% Finally, now we can identify the shortest path 
path = zeros(1,size(shortestPath,1));
% We need to save the path starting with the target node
path_index = 1;
path(path_index) = size(i_combinedNodes,1);

while path(path_index) > 1 % this is repeated until the starting node is reached
    
    path_index = path_index + 1;
    path(path_index) = shortestPath(path(path_index-1),4);
    
end

%% Now we can reverse the order of the saved path so that a path is printing 
path = fliplr(path(1:path_index));

%% Now for the final part we can extract the waypoint coordinates from this newly 
% constructed path array to be used in the rest of the code
visibilityPath(:,1) = i_combinedNodes(path,1);
visibilityPath(:,2) = i_combinedNodes(path,2);
end 
