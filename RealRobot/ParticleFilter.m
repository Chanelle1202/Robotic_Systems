function [botSim, botGhost_mean, botGhost_mode] = ParticleFilter(botSim, modifiedMap,numParticles, maxNumOfIterations)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%generate some random particles inside the map
num =numParticles; % number of particles
particles(num,1) = BotSim; %how to set up a vector of objects
for i = 1:num
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particles(i).randomPose(0); %spawn the particles in random locations
end

n = 0;
converged =0; %The filter has not converged yet


    
    
    
while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    n = n+1 %increment the current number of iterations
    % rr = robot();
     %botScanRaw = survey(rr, 50, 60);
     %botScan = [botScanRaw(4,2) ; botScanRaw(5,2) ; botScanRaw(6,2) ; botScanRaw(1,2) ; botScanRaw(2,2) ; botScanRaw(3,2)]
     botScan = botSim.ultraScan()

    %botScan = botScanRaw(:,2);

    %botScan = botSim.ultraScan(); %get a scan from the real robot.
   
    %% Write code for updating your particles scans
    scans = 6;
    particlesScan = zeros(scans,n);
    weight = zeros(num,1);
    particle_weight = zeros(360,1);
    k = 0.00001; %damping factor
    var =10;
    for i=1:num
        if particles(i).insideMap() ==1  
            particlesScan(:,i)= particles(i).ultraScan();
            difference = norm((particlesScan(:,i))-(botScan));
            weight(i) = k + (1/sqrt(2*pi*var))*exp(-((difference)^2/(2*var)));
                       
%             for j=1:scans
%                 %% Write code for scoring your particles
%                 difference = norm((particlesScan(:,i))-(botScan));
%                 particle_weight(j) = k + (1/sqrt(2*pi*var))*exp(-((difference)^2/(2*var)));
%                 particles(i).turn(pi/scans);
%             end
%             [max_weight, max_pos] = max(particle_weight);
%             weight(i) = max_weight;
%             particles(i).turn(max_pos*pi/scans);
        else
            weight(i)=0;
        end  
    end
    %now need to normalise
    weights = weight./sum(weight);
    
    positions = zeros(num, 2);
    pos_diffs = zeros(num, 1);
    
%     for i = 1:num
%         botPos = botSim.getBotPos();
%         positions(i,:) = particles(i).getBotPos();
%         pos_diffs(i) = sqrt((positions(i,1)-botPos(1))^2 + (positions(i,2)-botPos(2))^2);
%     end
    
    %figure(2)
    %bar(pos_diffs, weight)
    
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
%         particles(i).setBotAng(newParticleLocations(i,3)+rand());
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

    angles = zeros(num,1);
    for i=1:num
        angles(i)=particles(i).getBotAng();
    end    
       
    particles_mean_est = BotSim(modifiedMap);
    particles_mean_est.setBotPos(mean(positions));
    particles_mean_est.setBotAng(mean(angles));
    
    particles_mode_est = BotSim(modifiedMap);
    particles_mode_est.setBotPos(mode(round(positions)));
    particles_mode_est.setBotAng(mode(round(angles, 2)));

    %% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness)	
    
    mutation_rate=0.01;
    
    mutation_index = ceil(num.*rand(mutation_rate*num,1));
    
    for i=1:mutation_rate*num
        particles(mutation_index(i)).randomPose(0);
    end 
    
    %% Write code to decide how to move next
    % here they just turn in cicles as an example
    turn = 0.5;
    %realTurn = -turn * (180 / pi);
    move = 2;
    botSim.turn(turn); %turn the real robot.  
    botSim.move(move); %move the real robot. These movements are recorded for marking 
    %rotate(rr, 50, realTurn);
    %translate(rr, 50, move/100);
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
    
    
    botGhost_mean = particles_mean_est;
    botGhost_mode = particles_mode_est;

end

     botScan = botSim.ultraScan();
     %botScanRaw = survey(rr, 50, 60);
    %botScan = [botScanRaw(4,2);  botScanRaw(5,2) ; botScanRaw(6,2) ; botScanRaw(1,2) ; botScanRaw(2,2) ; botScanRaw(3,2)]

    %botScan = botScanRaw(:,2);
    
difference_mean= [360,1];
difference_mode= [360,1];
for i=1:360    
    botGhost_meanScan = botGhost_mean.ultraScan();
    botGhost_modeScan = botGhost_mode.ultraScan();
    difference_mean(i) = norm(botGhost_meanScan-botScan);
    difference_mode(i) = norm(botGhost_modeScan-botScan);
    botGhost_mean.turn(pi/180);
    botGhost_mode.turn(pi/180);
end
[min_weight_mean, min_pos_mean] = min(difference_mean);
botGhost_mean.turn(min_pos_mean*pi/180); 
[min_weight_mode, min_pos_mode]=min(difference_mode);
botGhost_mode.turn(min_pos_mode*pi/180);
end

