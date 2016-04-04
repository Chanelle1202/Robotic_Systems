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
maxNumOfIterations = 30;
n = 0;
converged =0; %The filter has not converged yet
while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    botScan = botSim.ultraScan(); %get a scan from the real robot.
    
    %% Write code for updating your particles scans
    particlesScan = zeros(6,n);
    weight = zeros(num,1);    
    k = 0.1; %damping factor
    var =1;
    for i=1:num
        if particles(i).insideMap() ==1
            particlesScan(:,i)= particles(i).ultraScan();
            %% Write code for scoring your particles
            difference = (((sum(particlesScan(:,i)-botScan))/6).^2);
            weight(i) = k + (1/sqrt(2*pi*var))*exp(-(difference/(2*var))); 
        else
            weight(i)=0;
        end  
    end
    %now need to normalise
    weights = weight./sum(weight);
    
    %% Write code for resampling your particles
    
    for i = 1:num
        newParticleLocations(i, :) = particles(find(rand() <= cumsum(weights),1)).getBotPos();
    end
     
    R=2;
  
    for i=1:num
        t = 2*pi*rand();
        r=R*sqrt(rand());
        particles(i).setBotPos([newParticleLocations(i,1)+r.*cos(t), newParticleLocations(i,2) + r.*sin(t)]);
    end
               
    
    %% Write code to check for convergence   
    
    % TODO accept this as a parameter?
    convergencethreshold = 10;
   
    positions = zeros(num, 2);
   
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
    
    particles_est = mean(positions);

    %% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness)	
    
    mutation_rate=0.01;
    
    mutation_index = ceil(num.*rand(mutation_rate*num,1));
    
    for i=1:mutation_rate*num
        particles(mutation_index(i)).randomPose(0);
    end 
    
    
    %% Write code to decide how to move next
    % here they just turn in cicles as an example
    turn = 0.5;
    move = 3;
    botSim.turn(turn); %turn the real robot.  
    botSim.move(move); %move the real robot. These movements are recorded for marking 
    for i =1:num %for all the particles. 
        particles(i).turn(turn); %turn the particle in the same way as the real robot
        particles(i).move(move); %move the particle in the same way as the real robot
    end
    
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    if botSim.debug()
        hold off; %the drawMap() function will clear the drawing when hold is off
        botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        botSim.drawBot(30,'g'); %draw robot with line length 30 and green
        for i =1:num
            particles(i).drawBot(3); %draw particle with line length 3 and default color
        end
        plot(particles_est(1), particles_est(2),'.r','markersize',20);
        drawnow;
    end

end
end
