function [botGhost] = localise(bot,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

%% setup code

robot_size = 10; % used to inflate internal boundaries, the size of the internal border
scans = 6;

%you can modify the map to take account of your robots configuration space
inflated_boundaries = boundary_inflation(map, robot_size); % execute function to draw boundary borders
modifiedMap = map;
%botSim.setMap(modifiedMap);


%% Particle Filter
maxNumOfIterations = 30;
numParticles = 500;

[bot, botGhost] = ParticleFilter(bot, modifiedMap,numParticles, maxNumOfIterations, scans, target);    

'PFL converveged estimate'
botGhost.getBotPos()


%% Path Planning

%% Calculate new extended, internl borders
inflated_boundaries = boundary_inflation(map, robot_size); % execute function to draw boundary borders

%% Plot new boundaries
external_boundaries_shifted_draw = inflated_boundaries;
external_boundaries_shifted_draw(size(external_boundaries_shifted_draw,1)+1,:) = inflated_boundaries(1,:);

plot(external_boundaries_shifted_draw(:,1), external_boundaries_shifted_draw(:,2), 'Color', 'cyan')

threshold = 10;


%% Plot the chosen path

end_point = target;

Robot_location = botGhost.getBotPos();
while Robot_location(1) > end_point(1) + 0.002 || Robot_location(1) < end_point(1) - 0.002 || Robot_location(2) > end_point(2) + 0.002 || Robot_location(2) < end_point(2) - 0.002;
    
    [angle, distance] = get_angle_dist(Robot_location, end_point, map, robot_size);
    Robot_direction = botGhost.getBotAng();
%     random_error = rand() / 10; % add a random error to represent robot movement errors
     angleRadian = toRadians('degrees', angle); 
%     angleRadian_Error = (angle + (angle * random_error)) * 3.14 / 180;
%     distance_Error = distance + (distance * random_error);    

%     if botSim.debug()
%         %botSim.drawBot(5,'red');
%         botGhost.drawBot(5,'cyan');
%     end
   
    %botSim.setBotAng(3.14 + angleRadian);
    
    turn = angleRadian - Robot_direction;
    
%     while turn > (2 * pi)
%        turn = turn - pi; 
%     end
%     
%     while turn < 0
%         turn = turn + pi;
%     end
    
    bot.turn(turn);
    botGhost.turn(turn);

    distanceInFront = bot.getDistance_cm();

    if (distanceInFront < distance)
         [bot, botGhost] = ParticleFilter(bot, modifiedMap,numParticles, maxNumOfIterations, scans, target);
    else
        bot.move(distance);
        botGhost.move(distance);
    end
    
    Robot_location = botGhost.getBotPos();
end

end
