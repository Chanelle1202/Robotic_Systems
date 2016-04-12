function [botSim] = localise(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

%% setup code
%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);

%% Localisation code
maxNumOfIterations = 50;
numParticles = 300;

[botSim, botGhost_mean, botGhost_mode] = ParticleFilter(botSim, modifiedMap,numParticles, maxNumOfIterations);    
    
    %% Write code to decide how to move next
%     % here they just turn in cicles as an example
%     turn = 0.5;
%     move = 2;
%     botSim.turn(turn); %turn the real robot.  
%     botSim.move(move); %move the real robot. These movements are recorded for marking 
%     for i =1:num %for all the particles. 
%         particles(i).turn(turn); %turn the particle in the same way as the real robot
%         particles(i).move(move); %move the particle in the same way as the real robot
%     end
%     
%     particles_mean_est.turn(turn);
%     particles_mean_est.move(move);
%     
%     particles_mode_est.turn(turn);
%     particles_mode_est.move(move);
    
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
