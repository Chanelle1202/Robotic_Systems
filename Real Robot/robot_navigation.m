%% This script uses various functions, which as a whole execute a Visibility 
% Graph navigation algorithm to determine a path from a given sart point to 
% any end point in a bounded area. 

close all
clear all
clc

%% Initialise the map and size of the robot
map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  % representing external boundaries

botSim = BotSim(map,[0,0,0], 0);  % sets up a botSim object with a map, and debug mode on.
target = botSim.getRndPtInMap(10);  % gets random endpoint / target.

bot = Bot();
%% Localisation function using Particle Filtering to estimate the current 
% location of the robot
tic % starts timer
print = 'particle filtering'
returnedBot = localise(bot,map,target);
bot.complete();
resultsTime = toc

resultsDis = distance(target, returnedBot.getBotPos())



