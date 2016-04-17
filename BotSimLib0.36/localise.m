function [botSim] = localise(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

%% setup code
%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);

scans=20;
botSim.setScanConfig(botSim.generateScanConfig(scans));

robot_size = 6; % used to inflate internal boundaries, the size of the internal border

%% Particle Filter
maxNumOfIterations = 50;
numParticles = 500;

[botSim, botGhost_mean, botGhost_mode] = ParticleFilter(botSim, modifiedMap,numParticles, maxNumOfIterations, scans);    

if botSim.debug()
    mean_distance = sqrt(sum((botSim.getBotPos()-botGhost_mean.getBotPos()).^2))
    mode_distance = sqrt(sum((botSim.getBotPos()-botGhost_mode.getBotPos()).^2))
end
%% Path Planning

%% Calculate new extended, internl borders
inflated_boundaries = boundary_inflation(map, robot_size); % execute function to draw boundary borders

%% Plot new boundaries
external_boundaries_shifted_draw = inflated_boundaries;
external_boundaries_shifted_draw(size(external_boundaries_shifted_draw,1)+1,:) = inflated_boundaries(1,:);

if botSim.debug()
    plot(external_boundaries_shifted_draw(:,1), external_boundaries_shifted_draw(:,2), 'Color', 'cyan')
end

threshold = 5;


%% Plot the chosen path

end_point = target;

Robot_location = botGhost_mean.getBotPos();
while Robot_location(1) > end_point(1) + 0.002 || Robot_location(1) < end_point(1) - 0.002 || Robot_location(2) > end_point(2) + 0.002 || Robot_location(2) < end_point(2) - 0.002;
    
    [angle, distance] = get_angle_dist(Robot_location, end_point, map, robot_size);
    Robot_direction = botGhost_mean.getBotAng();
%     random_error = rand() / 10; % add a random error to represent robot movement errors
     angleRadian = (angle) * 3.14 / 180; 
%     angleRadian_Error = (angle + (angle * random_error)) * 3.14 / 180;
%     distance_Error = distance + (distance * random_error);    

    if botSim.debug()
        botSim.drawBot(5,'red');
        botGhost_mean.drawBot(5,'cyan');
    end
   
    %botSim.setBotAng(3.14 + angleRadian);
    botSim.turn(3.14-Robot_direction+angleRadian);
    botGhost_mean.turn(3.14-Robot_direction+angleRadian);

    
    botScan = botSim.ultraScan();
    
    if botScan(1)<= distance;
        [botSim, botGhost_mean, botGhost_mode] = ParticleFilter(botSim, modifiedMap,numParticles, maxNumOfIterations);
    else
        botSim.move(distance);
        botGhost_mean.move(distance);
    end
    
    botScan = botSim.ultraScan();
    botGhost_meanScan = botGhost_mean.ultraScan();
    %calculate the difference between the ghost robot and the real robot
    difference = (sum(botGhost_meanScan-botScan)/6);
    
    %Run particle filter if the difference between the ultrasound values is
    %above the threshold
    if (abs(difference) > threshold)
        [botSim, botGhost_mean, botGhost_mode] = ParticleFilter(botSim, modifiedMap,numParticles, maxNumOfIterations);
    end

    Robot_location = botGhost_mean.getBotPos();
    
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
    botGhost_mean.drawBot(30,'r'); %draws the mean ghost bot with line length 30 and red
    botGhost_mode.drawBot(30,'b'); %draws the mode ghost bot with line length 30 and blue
    drawnow;
end

end
