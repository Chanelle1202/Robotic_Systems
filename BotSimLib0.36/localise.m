function [botSim] = localise(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

%% setup code
%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);

scans=30;
botSim.setScanConfig(botSim.generateScanConfig(scans));

robot_size = 6; % used to inflate internal boundaries, the size of the internal border

%% Particle Filter
maxNumOfIterations = 30;
numParticles = 500;

%botGhost is the estimated botSim
[botSim, botGhost] = ParticleFilter(botSim, modifiedMap,numParticles, maxNumOfIterations, scans);    

if botSim.debug()
    distance = sqrt(sum((botSim.getBotPos()-botGhost.getBotPos()).^2));
end
%% Path Planning

%% Calculate new extended, internl borders
inflated_boundaries = boundary_inflation(map, robot_size); % execute function to draw boundary borders

%% Plot new boundaries
external_boundaries_shifted_draw = inflated_boundaries;
external_boundaries_shifted_draw(size(external_boundaries_shifted_draw,1)+1,:) = inflated_boundaries(1,:);

if botSim.debug()
    plot(external_boundaries_shifted_draw(:,1), external_boundaries_shifted_draw(:,2), 'Color', 'cyan')
    plot(target(1), target(2), 'rX')

end


%% Plot the chosen path

end_point = target;

Robot_location = botGhost.getBotPos();
while Robot_location(1) > end_point(1) + 0.002 || Robot_location(1) < end_point(1) - 0.002 || Robot_location(2) > end_point(2) + 0.002 || Robot_location(2) < end_point(2) - 0.002;
    
    [angle, distance] = get_angle_dist(Robot_location, end_point, map, robot_size);
    Robot_direction = botGhost.getBotAng();

    if botSim.debug()
        botSim.drawBot(5,'red');
        botGhost.drawBot(5,'cyan');
    end
   
    botSim.turn(angle-Robot_direction);
    botGhost.turn(angle-Robot_direction);

    
    botScan = botSim.ultraScan();
    
    %Check the bot won't move through a wall, if so run particle filter
    %again
    if botScan(1)<= distance;
        [botSim, botGhost] = ParticleFilter(botSim, modifiedMap,numParticles, maxNumOfIterations, scans);
    else
        botSim.move(distance);
        botGhost.move(distance);
    end
    
    botScan = botSim.ultraScan();
    botGhostScan = botGhost.ultraScan();
    
    %calculate the difference between the ghost robot and the real robot
    difference = (sum(botGhostScan-botScan)/scans);
    threshold = 3;
    
    %Run particle filter if the difference between the ultrasound values is
    %above the threshold
    if (abs(difference) > threshold)
        [botSim, botGhost] = ParticleFilter(botSim, modifiedMap,numParticles, maxNumOfIterations, scans);
    end

    %Estimated robot location
    Robot_location = botGhost.getBotPos();
    
    if botSim.debug()
        pause(1);
    end
end

    
%% Drawing
%only draw if you are in debug mode or it will be slow during marking
if botSim.debug()
    figure(1)
    hold off; %the drawMap() function will clear the drawing when hold is off
    botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
    botSim.drawBot(30,'g'); %draw robot with line length 30 and green
    botGhost.drawBot(30,'r'); %draws the mean ghost bot with line length 30 and red
    drawnow;
end

end
