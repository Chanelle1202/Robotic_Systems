%% This script uses various functions, which as a whole execute a Visibility 
% Graph navigation algorithm to determine a path from a given sart point to 
% any end point in a bounded area. 

close all
clear all
clc

%% Initialise specific start and end points
% start_point = [53;38.5];
% end_point = [95;70];
% Robot_location = start_point;

%% Initialise the map and size of the robot
map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  % representing external boundaries
robot_size = 6; % used to inflate internal boundaries, the size of the internal border

%% Let the external boundaries form a closed polygon for graphical purposes
external_boundaries_draw = map;
external_boundaries_draw(size(map,1)+1,:) = external_boundaries_draw(1,:);

hold on
axis equal

botSim = BotSim(map,[0,0,0]);  % sets up a botSim object with a map, and debug mode on.
botSim.drawMap();
drawnow;
botSim.randomPose(10); % puts the robot in a random position at least 10cm away from an external boundary / wall
target = botSim.getRndPtInMap(10);  % gets random endpoint / target.
end_point = target

%% Localisation function using Particle Filtering to estimate the current 
% location of the robot
tic % starts timer
print = 'particle filtering'
returnedBot = localise(botSim,map,target);
ghostBot = BotSim(map, [0,0,0]);
ghostBot.setBotPos(returnedBot.getBotPos());
botSim.setBotPos(returnedBot.getBotPos());
Robot_location = returnedBot.getBotPos();
resultsTime = toc % stops timer

%% Plot the boundaries, starting points and end points
hold off;
botSim.drawMap();

plot(external_boundaries_draw(:,1), external_boundaries_draw(:,2), 'Color', 'black') % plot the external map borders
plot(Robot_location(1), Robot_location(2), 'X', 'Color', 'green') % plot robot starting point
plot(Robot_location(1), Robot_location(2), 'O', 'Color', 'green') % encricle robot starting point
plot(end_point(1), end_point(2), 'X', 'Color', 'red') % plot robot goal / endpoint
plot(end_point(1), end_point(2), 'O', 'Color', 'red') % encircle robot goal / endpoint

%% Calculate new extended, internl borders
inflated_boundaries = boundary_inflation(map, robot_size); % execute function to draw boundary borders

%% Plot new boundaries
external_boundaries_shifted_draw = inflated_boundaries;
external_boundaries_shifted_draw(size(external_boundaries_shifted_draw,1)+1,:) = inflated_boundaries(1,:);
plot(external_boundaries_shifted_draw(:,1), external_boundaries_shifted_draw(:,2), 'Color', 'cyan')


threshold = 5;
%put the robot in another random position to test the ultrasound correction
botSim.randomPose(10);
Robot_location = botSim.getBotPos();
%% Plot the chosen path
while Robot_location(1) > end_point(1) + 0.002 || Robot_location(1) < end_point(1) - 0.002 || Robot_location(2) > end_point(2) + 0.002 || Robot_location(2) < end_point(2) - 0.002;
    [angle, distance] = get_angle_dist(Robot_location, end_point, map, robot_size);
    
    random_error = rand() / 10; % add a random error to represent robot movement errors
    angleRadian = (angle) * 3.14 / 180; 
    angleRadian_Error = (angle + (angle * random_error)) * 3.14 / 180;
    distance_Error = distance + (distance * random_error);    
    
    botSim.drawBot(5,'red');
    ghostBot.drawBot(5,'cyan');
   
    botSim.setBotAng(3.14 + angleRadian);
    ghostBot.setBotAng(3.14 + angleRadian);
    botSim.move(distance);
    ghostBot.move(distance);
    Robot_location = botSim.getBotPos();
    
    botScan = botSim.ultraScan();
    ghostBotScan = ghostBot.ultraScan();
    
    %calculate the difference between the ghost robot and the real robot
    difference = (sum(ghostBotScan-botScan)/6);
    
    %Run particle filter if the difference between the ultrasound values is
    %above the threshold
    if (abs(difference) > threshold)
        difference
        % location of the robot
        tic % starts timer
        print = 'particle filtering'
        returnedBot = localise(botSim,map,target);
        ghostBot = BotSim(map, [0,0,0]);
        ghostBot.setBotPos(returnedBot.getBotPos());
        botSim.setBotPos(returnedBot.getBotPos());
        Robot_location = returnedBot.getBotPos();
        resultsTime = toc % stops timer
    end
    
    
    
    pause(1) %in seconds  
    
end