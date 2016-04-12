function [botSim] = localise(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

%% setup code
%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);

%generate some random particles inside the map
num =300; % number of particles
particles(num,1) = BotSim; %how to set up a vector of objects
for i = 1:num
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particles(i).randomPose(0); %spawn the particles in random locations
end

%% Localisation code
maxNumOfIterations = 50;
n = 0;
converged =0; %The filter has not converged yet
while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    botScan = botSim.ultraScan(); %get a scan from the real robot.
    
    %% Write code for updating your particles scans
    particlesScan = zeros(6,n);
    weight = zeros(num,1);    
    k = 0.00001; %damping factor
    var =10;
    for i=1:num
        if particles(i).insideMap() ==1
            particlesScan(:,i)= particles(i).ultraScan();
            %% Write code for scoring your particles
            difference = norm((particlesScan(:,i))-(botScan));
            weight(i) = k + (1/sqrt(2*pi*var))*exp(-((difference)^2/(2*var))); 

        else
            weight(i)=0;
        end  
    end
    %now need to normalise
    weights = weight./sum(weight);
    
    positions = zeros(num, 2);
    pos_diffs = zeros(num, 1);
    
    for i = 1:num
        botPos = botSim.getBotPos();
        positions(i,:) = particles(i).getBotPos();
        pos_diffs(i) = sqrt((positions(i,1)-botPos(1))^2 + (positions(i,2)-botPos(2))^2);
    end
    
    figure(2)
    bar(pos_diffs, weight)
    
    %% Write code for resampling your particles
    
    newParticleLocations = zeros(num, 3);
    
    for i = 1:num
        j = find(rand() <= cumsum(weights),1);
        newParticleLocations(i, 1:2) = particles(j).getBotPos();
        newParticleLocations(i, 3) = particles(j).getBotAng();
    end
     
    R=2;
  
    for i=1:num
        t = 2*pi*rand();
        r=R*sqrt(rand());
        particles(i).setBotPos([newParticleLocations(i,1)+r.*cos(t), newParticleLocations(i,2) + r.*sin(t)]);
%         particles(i).setBotAng(newParticleLocations(i,3));
    end
               
    
    %% Write code to check for convergence   
    
    % TODO accept this as a parameter?
    convergencethreshold = 2;
   
    % obtain particle positions
    for j = 1:num
        positions(j,:) = particles(j).getBotPos();
    end
   
    % compute standard deviations of particle positions (in x and y
    % coordinates)
    stdev = std(positions);
   
    % particle filter has converged if standard deviations are below
    % convergence threshold
    if stdev < convergencethreshold
        converged = 1;
    end
    
    %% Estimating particle position

    angles = 0;
    for i=1:num
        angles=angles+particles(i).getBotAng;
    end
    
    angle=angles/num;
    
    particles_mean_est = BotSim(modifiedMap);
    particles_mean_est.setBotPos(mean(positions));
    particles_mean_est.setBotAng(angle);
    
    particles_mode_est = BotSim(modifiedMap);
    particles_mode_est.setBotPos(mode(round(positions)));
    particles_mode_est.setBotAng(angle);

    %% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness)	
    
    mutation_rate=0.02;
    
    mutation_index = ceil(num.*rand(mutation_rate*num,1));
    
    for i=1:mutation_rate*num
        particles(mutation_index(i)).randomPose(0);
    end 
    
    
    %% Write code to decide how to move next
    % here they just turn in cicles as an example
    turn = 0.5;
    move = 2;
    botSim.turn(turn); %turn the real robot.  
    botSim.move(move); %move the real robot. These movements are recorded for marking 
    for i =1:num %for all the particles. 
        particles(i).turn(turn); %turn the particle in the same way as the real robot
        particles(i).move(move); %move the particle in the same way as the real robot
    end
    
    particles_mean_est.turn(turn);
    particles_mean_est.move(move);
    
    particles_mode_est.turn(turn);
    particles_mode_est.move(move);
    
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
        particles_mean_est.drawBot(30, 'r');
        particles_mode_est.drawBot(30, 'b');
        drawnow;
    end

end
end
