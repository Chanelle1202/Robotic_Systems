function visibilityPath = pathfinder(start_point, end_point, modifiedMap)

% Function which finds the shortest path from a start point to an end point 
% in a 2D arena whilst avoiding obstacles. This function uses Visibility
% Graphs elements to implement the Dijkstra's algorithm.

%% Initialize empty arrays
i_combinedNodes = zeros(size(modifiedMap,1)+2,3);

%% Create initial unvisited nodes array
i_combinedNodes(1,1:2) = [start_point(1), start_point(2)];
i_combinedNodes(2:size(modifiedMap,1)+1,1:2) = modifiedMap;
i_combinedNodes(size(modifiedMap,1)+2,1:2) = [end_point(1), end_point(2)];

%% Assign placeholder/tentative distances to all nodes as infinity
i_combinedNodes(:,3) = Inf*ones(size(i_combinedNodes,1),1);

%% Identify nodes visible from starting point
visibleNodesID = zeros(1,size(i_combinedNodes,1));

%% Create a library listing the visible neighbours of all of the nodes and 
%  their distances with respect to the reference nodes 

% Initialize library as an empty three dimensional array
visibleNeighboursLibrary =  zeros(size(i_combinedNodes,1),size(i_combinedNodes,2),size(i_combinedNodes,1));

% Initialize visible nodes index to zero
visibleIndex = 0;

for referenceNodeID = 1:size(i_combinedNodes,1)
    
    % Copy the i_combinedNodes into a new combinedNodes entry
    % This will form a new 'page' for each of the reference nodes
    combinedNodes = i_combinedNodes;
    
    for target_ID = 1:size(i_combinedNodes,1)
        
        % Assign observer and target nodes
        observerState = i_combinedNodes(referenceNodeID,:);
        currentTargetNode = i_combinedNodes(target_ID,:);
        
        % Check visibility of target node from observer
        % visibility = line_of_sight(observerState, currentTargetNode, i_combinedNodes);
        visibility = line_of_sight(observerState, currentTargetNode, modifiedMap);
        
        if visibility == 1 %If target is visible
            
            % Record the visible node ID
            visibleIndex = visibleIndex + 1;
            visibleNodesID(visibleIndex) = target_ID;
            
            % Overwrite recorded distance
            combinedNodes(target_ID,3) = sqrt((currentTargetNode(1) - observerState(1))^2 + (currentTargetNode(2) - observerState(2))^2);
                        
        end
        
    end
    
    % Create a three dimensional array representing a library of the visible neighbours with respect to the reference node
    % The reference node ID is represented by the 'page' number
    visibleNeighboursLibrary(:,:,referenceNodeID) = combinedNodes;
    
end

%% Initialize unvisitedNodes
unvisitedNodes = zeros(1,size(i_combinedNodes,1));

%% Generate a list of all available nodes and assign them to the unvisited nodes set
for index = 1:size(i_combinedNodes)
    unvisitedNodes(index) = index;
end

%% Copy the first 'page' of the visibleNeighboursLibrary as the initial
%shortest path array
shortestPath = visibleNeighboursLibrary(:,:,1);

%% Initialize all of the precursor nodes to 1 (the starting point)
shortestPath(:,4) = 1;

%% Take out the starting point (node 1) from the unvisited nodes set
current_node = 1;
unvisitedNodes = setdiff(unvisitedNodes, current_node);

%% Repeat until the set of unvisited nodes is depleted
while size(unvisitedNodes,2) > 0
    
    % Find the node with the smallest nonzero cumulative distance to be assigned as the next current node
    cumulative_distances = shortestPath(unvisitedNodes,3);
    [~, currentNodeID_index] = min(cumulative_distances); %First output (~) is not used
    
    % Extract the current node
    currentNodeID = unvisitedNodes(currentNodeID_index);
    
    % Take out the current node from the set of unvisited nodes
    unvisitedNodes = setdiff(unvisitedNodes, currentNodeID);
    
    % Refer to the library to find the distance to visible (neighboring) and UNVISITED nodes
    for unvisited_node_index = 1:size(unvisitedNodes,2)
        
        % Assign one of the unvisited nodes as the target node
        target_node_ID = unvisitedNodes(unvisited_node_index);
        
        % Visibility is implied if the distance recorded between the current and target nodes (in the library) is less than infinity
        if visibleNeighboursLibrary(target_node_ID,3,currentNodeID) < Inf
            
            % Calculate the (tentative) cumulative distance for the target node
            cumulativeNode_distance = shortestPath(currentNodeID,3);
            cumulativeNodeToTarget_distance = visibleNeighboursLibrary(target_node_ID,3,currentNodeID);
            newCumulative_distance = cumulativeNode_distance + cumulativeNodeToTarget_distance;
            
            % Find the PREVIOUS cumulative distance to the target node
            previousCumulativetoTarget_distance = shortestPath(target_node_ID,3);
            
            % If a smaller cumulative distance from the starting node to the
            % target node, passing through the current node was obtained
            if newCumulative_distance < previousCumulativetoTarget_distance
                
                %Overwrite the previous cumulative distance to the smaller value
                shortestPath(target_node_ID,3) = newCumulative_distance;
                
                %Replace the last precursor node with currentNodeID
                shortestPath(target_node_ID,4) = currentNodeID;
                
            end
            
        end
        
    end
    
end


%% Extract the shortest path
% Initialize path array
path = zeros(1,size(shortestPath,1));

% Assign the end point ID as the first entry on the path array
path_index = 1;
path(path_index) = size(i_combinedNodes,1);

while path(path_index) > 1 % Repeat until the starting node is reached
    
    path_index = path_index + 1;
    path(path_index) = shortestPath(path(path_index-1),4);
    
end

%% Reverse order of path array so that it now points from the starting point to the end point
% Only consider the first path_index entries; the remaining are only placeholder values (zeros)
path = fliplr(path(1:path_index));

%% Extract the waypoints identified by the entries in the path array
visibilityPath(:,1) = i_combinedNodes(path,1);
visibilityPath(:,2) = i_combinedNodes(path,2);
end 
