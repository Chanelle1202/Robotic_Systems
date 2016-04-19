function [botSim, botGhost] = ParticleFilter(botSim, modifiedMap,numParticles, maxNumOfIterations, scans)
%Particle Filter Localisation Function

%generate some random particles inside the map
num =numParticles; % number of particles
particles(num,1) = BotSim; %how to set up a vector of objects
for i = 1:num
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particles(i).randomPose(10); %spawn the particles in random locations
    particles(i).setScanConfig(generateScanConfig(particles(i), scans));
    particles(i).setMotionNoise(2); %give the particles some motion noise
    particles(i).setTurningNoise(pi/10);
end

n = 0;
while(n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations   
    botScan = botSim.ultraScan(); %get a scan from the real robot.
    
    %% Write code for updating your particles scans
    particlesScan = zeros(scans,num);
    difference = zeros(scans,num);
    weight = zeros(num,1);
    particle_weight = zeros(scans,1);
    var = 80;   %variance
    k = 0.00000001; %damping factor
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

     
%     R=2;
%   
%     for i=1:num
%         t = 2*pi*rand();
%         r=R*sqrt(rand());
%         particles(i).setBotPos([newParticleLocations(i,1)+r.*cos(t), newParticleLocations(i,2) + r.*sin(t)]);
%         particles(i).setBotAng(newParticleLocations(i,3));
%     end
                 
    %% Estimating particle position   
    
    % obtain particle positions and angles
    positions = zeros(num, 2);   
    angles = zeros(num,1);
   
    for i = 1:num
        positions(i,:) = particles(i).getBotPos();
        angles(i)=particles(i).getBotAng();
    end
    
    if botSim.debug()
        botPos = botSim.getBotPos();
        pos_diffs = zeros(num,1);
        for i=1:num
            pos_diffs(i) = sqrt((positions(i,1)-botPos(1))^2 + (positions(i,2)-botPos(2))^2); 
        end
        figure(4)
        bar(pos_diffs, weight)
        figure(5)
        scatter(min(difference), weight)
    end
    
    %Set the mean estimate
    botGhost_mean = BotSim(modifiedMap);
    botGhost_mean.setScanConfig(botGhost_mean.generateScanConfig(scans));
    botGhost_mean.setBotPos(mean(positions));
    botGhost_mean.setBotAng(mean(angles));
    
    %Set the mode estimate
    botGhost_mode = BotSim(modifiedMap);
    botGhost_mode.setScanConfig(botGhost_mode.generateScanConfig(scans));
    botGhost_mode.setBotPos(mode(round(positions)));
    botGhost_mode.setBotAng(mode(round(angles, 2)));
    
    if botSim.debug()
        figure(1)
        hold off; %the drawMap() function will clear the drawing when hold is off
        botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        botSim.drawBot(30,'g'); %draw robot with line length 30 and green
        for i =1:num
            particles(i).drawBot(3); %draw particle with line length 3 and default color
        end
        botGhost_mean.drawBot(30, 'r');
        botGhost_mode.drawBot(30, 'b');
        drawnow;
    end 
    
    %% Write code to check for convergence   
    
    convergence_threshold = 2;
    
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
   
    if rand()<0.7 %most of the time move in the maximum direction
        [max_distance, max_index] = max(botScan); %find maximum possible distance
        turn = (max_index-1)*2*pi/scans; %orientate towards the max distance
        move = max_distance*0.8*rand(); %move a random amount of the max distance, but never the entire distance
    else %some of the time move in a random direction
        index=randi(scans); 
        turn = (index-1)*2*pi/scans;
        move= botScan(index)*0.8;
    end
        
    botSim.turn(turn);        
    botSim.move(move); %move the real robot. These movements are recorded for marking 

    for i =1:num %for all the particles.
          particles(i).turn(turn);
          particles(i).move(move);
    end

    
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    if botSim.debug()
        figure(1)
        hold off; %the drawMap() function will clear the drawing when hold is off
        botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        botSim.drawBot(30,'g'); %draw robot with line length 30 and green
        for i =1:num
            particles(i).drawBot(3); %draw particle with line length 3 and default color
        end
        drawnow;
    end
    
end

%% More rigorous check of orientation

botScan = botSim.ultraScan();
difference_mean= zeros(360,1);
difference_mode= zeros(360,1);
for i=1:360    %Check scans of mode and mean estimates at every angle
    botGhost_meanScan = botGhost_mean.ultraScan();
    botGhost_modeScan = botGhost_mode.ultraScan();
    difference_mean(i) = norm(botGhost_meanScan-botScan);
    difference_mode(i) = norm(botGhost_modeScan-botScan);
    botGhost_mean.setBotAng(i*pi/180);
    botGhost_mode.setBotAng(i*pi/180);
end

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

