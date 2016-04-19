function [bot, botGhost] = ParticleFilter(bot, modifiedMap,numParticles, maxNumOfIterations, scans, target)

%Particle Filter Localisation Function

    sensorNoise = 1.363160109; % from robot calibration - 0;%
    motionNoise = 0.012088592; % from robot calibration - 0;%
    turningNoise = toRadians('degrees', 5.444208795); % from robot calibration - 0;%
        turnBot =pi/4;
        
%generate some random particles inside the map
num =numParticles; % number of particles
particles(num,1) = BotSim; %how to set up a vector of objects
for i = 1:num 
    particles(i) = BotSim(modifiedMap, [ sensorNoise, motionNoise, turningNoise ], 0);  %each particle should use the same map as the botSim object
    particles(i).randomPose(10); %spawn the particles in random locations
    particles(i).setScanConfig(generateScanConfig(particles(i), scans));
end

n = 0;

while(n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    
    botScan = bot.ultraScan() %get a scan from the real robot.
    
    while (botScan < 0)  %Catch invalid scan results
        bot.turn(turnBot);
        
        for i=1:num
           particles(i).turn(turnBot); 
        end
        botScan = bot.ultraScan() %get a scan from the real robot.
    end
    
    %% Write code for updating your particles scans
    particlesScan = zeros(scans,num);
    difference = zeros(scans,num);
    weight = zeros(num,1);
    particle_weight = zeros(scans,1);
    var = 80;   %variance
    k = 0; %damping factor
    for i=1:num
        if particles(i).insideMap() ==0
            particles(i).randomPose(0);
        end
        particlesScan(:,i)= particles(i).ultraScan();
        for j=1:scans
            %% Write code for scoring your particles
            p = circshift(particlesScan(:,i),j); %shift the scans to allow for different orientations
            difference(j,i) = sqrt(sum((p-botScan).^2)); %difference is Euclidean distance between scan vectors
            particle_weight(j) = k + (1/sqrt(2*pi*var))*exp(-((difference(j,i))^2/(2*var)));
        end
        [max_weight, max_pos] = max(particle_weight);
        weight(i) = max_weight;
        particles(i).turn(max_pos*2*pi/scans); %Give the particle the best orientation
    end
        
    %now need to normalise
    weights = weight./sum(weight);
    
    %% Write code for resampling your particles
    
    newParticleLocations = zeros(num, 3);
    
    for i = 1:num
        j = find(rand() <= cumsum(weights),1);
        newParticleLocations(i, 1:2) = particles(j).getBotPos();
        newParticleLocations(i, 3) = particles(j).getBotAng();
    end
     

  
    for i=1:num
       
        particles(i).setBotPos([newParticleLocations(i,1), newParticleLocations(i,2)]);
        particles(i).setBotAng(newParticleLocations(i,3));
    end
                 
    %% Estimating particle position   
    
    % obtain particle positions and angles
    positions = zeros(num, 2);   
    angles = zeros(num,1);
   
    for i = 1:num
        positions(i,:) = particles(i).getBotPos();
        angles(i)=particles(i).getBotAng();
    end
   

    
    %Set the mean estimate
    botGhost_mean = BotSim(modifiedMap, [ sensorNoise, motionNoise, turningNoise ], 0);
    botGhost_mean.setScanConfig(botGhost_mean.generateScanConfig(scans));
    botGhost_mean.setBotPos(mean(positions));
    botGhost_mean.setBotAng(mean(angles));
    
    %Set the mode estimate
    botGhost_mode = BotSim(modifiedMap, [ sensorNoise, motionNoise, turningNoise ], 0);
    botGhost_mode.setScanConfig(botGhost_mode.generateScanConfig(scans));
    botGhost_mode.setBotPos(mode(round(positions)));
    botGhost_mode.setBotAng(mode(round(angles)));
    
        figure(3)
        hold off; %the drawMap() function will clear the drawing when hold is off
        particles(1).drawMap(); %drawMap() turns hold back on again, so you can draw the botsn
        for i =1:num
            particles(i).drawBot(3); %draw particle with line length 3 and default color
        end
        botGhost_mean.drawBot(30, 'r');
        botGhost_mode.drawBot(30, 'b');
        drawnow;
    
    %% Write code to check for convergence   
    
    convergence_threshold = 5;
    
    % compute standard deviations of particle positions
    stdev = std(positions);
   
    % particle filter has converged if standard deviations are below convergence threshold
    if stdev < convergence_threshold
        break; %particle filter has converged so break out of while loop immediately before any movement
    end
    %% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness)	
    
    mutation_rate=0.01;
    
    for i=1:mutation_rate*num
        particles(randi(num)).randomPose(0);
    end 
    
    %% Write code to decide how to move next
    % here they just turn in cicles as an example

    botScanFront = bot.getDistance_cm();

    while (botScanFront < 0)
         
        bot.turn(turnBot);
        for i=1:num
            particles(i).turn(turnBot);
        end
        
        botScanFront = bot.getDistance_cm();
    end

    if (botScanFront > 30)
        move = botScanFront*0.3; % potentially use small fixed increment
    else
        move = 0;
    end
        
    turn = pi/2;
    
    bot.move(move); %move the real robot. These movements are recorded for marking 
    bot.turn(turn);

    for i =1:num %for all the particles.
        particles(i).move(move); %move the particle in the same way as the real robot
        particles(i).turn(turn); %turn the particle in the same way as the real robot

    end

    
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    figure(3)
    hold off; %the drawMap() function will clear the drawing when hold is off
    particles(1).drawMap(); %drawMap() turns hold back on again, so you can draw the bots
    for i =1:num
        particles(i).drawBot(3); %draw particle with line length 3 and default color
    end
    plot(target(1),target(2),'Marker','o','Color','g');
    drawnow;    
end

%% More rigorous check of orientation

% botScan = bot.ultraScan();
difference_mean= zeros(360,1);
difference_mode= zeros(360,1);
% for i=1:360    
%     botGhost_meanScan = botGhost_mean.ultraScan();
%     botGhost_modeScan = botGhost_mode.ultraScan();
%     difference_mean(i) = norm(botGhost_meanScan-botScan);
%     difference_mode(i) = norm(botGhost_modeScan-botScan);
%     botGhost_mean.setBotAng(i*pi/180);
%     botGhost_mode.setBotAng(i*pi/180);
% end

%find the best orientation for the mean estimate
[min_diff_mean, min_pos_mean] = min(difference_mean);
botGhost_mean.setBotAng(min_pos_mean*pi/180); 

%find the best orientation for the mode estimate
[min_diff_mode, min_pos_mode]=min(difference_mode);
botGhost_mode.setBotAng(min_pos_mode*pi/180);


if min_diff_mean < min_diff_mode %pick best from mean or mode estimates
    botGhost = botGhost_mean;
else
    botGhost = botGhost_mode;
end

end

