classdef Bot
    %ROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        RobotRadius_m = 0.064;
        WheelRadius_m = 0.022;
        WheelCircumference_m = 2*pi*Bot.WheelRadius_m;
        

    end
    %public properties
    properties
        pos;    %position of the robot
        ang;    %angle of the robot (radians)
        dir;    %angle of the robot (stored as 2d vector)
        unmodifiedMap;
        scanOffset
        map;    %map coordinates with a copy of the first coordiantes at the end
        mapLines;   %The map stored as a list of lines (for easy line interection)
        inpolygonMapformatX; %The map stored as a polygon for the insidepoly function
        inpolygonMapformatY; %The map stored as a polygon for the insidepoly function
    end
    
    properties
        Handle
        MotorA
        MotorB
        MotorC
        MotorsAB
    end
    
    methods

        %% constructor
        
        function bot = Bot(newMap)
            if nargin <1
                warning = 'map should be passed to Bot'                        
            else
                bot.unmodifiedMap = newMap;
                bot.inpolygonMapformatX = cat(1,newMap(:,1), newMap(1,1));
                bot.inpolygonMapformatY = cat(1,newMap(:,2), newMap(1,2));

                newMap(length(newMap)+1,:)= newMap(1,:);
                bot.map = newMap;
                bot.mapLines = zeros(length(bot.map)-1,4);  %each row represents a border of the map
                for i =1:size(bot.mapLines,1)
                    bot.mapLines(i,:) = [bot.map(i,:) bot.map(i+1,:)] ;
                end
                bot.pos = [0 0];
                bot.ang = 2*pi*rand(1);
                bot.dir = [cos(bot.ang) sin(bot.ang)];
            end
                    
            
            bot.Handle = COM_OpenNXT();
            COM_SetDefaultNXT(bot.Handle);
            
            bot.MotorA = NXTMotor('A');
            bot.MotorA.ActionAtTachoLimit = 'Brake';
            bot.MotorA.SmoothStart = true;
            bot.MotorA.SpeedRegulation = false;
            
            bot.MotorB = NXTMotor('B');
            bot.MotorB.ActionAtTachoLimit = 'Brake';
            bot.MotorB.SmoothStart = true;
            bot.MotorB.SpeedRegulation = false;
            
            bot.MotorC = NXTMotor('C');
            
            bot.MotorsAB = NXTMotor('AB');
            bot.MotorsAB.ActionAtTachoLimit = 'Brake';
            bot.MotorA.SmoothStart = true;
            bot.MotorsAB.SpeedRegulation = false;
            
            OpenSwitch(SENSOR_1);
            OpenUltrasonic(SENSOR_3);
            
            NXT_GetBatteryLevel()
        end
        
        %% destructor
        
        function delete(bot)
            bot.MotorA.Stop('off');
            bot.MotorB.Stop('off');
            bot.MotorC.Stop('off');
            bot.MotorsAB.Stop('off');

            CloseSensor(SENSOR_1);
            CloseSensor(SENSOR_3);
            
            COM_CloseNXT(bot.Handle);
        end
        
        function randomPose(bot)
            %moves the robot to a random position and orientation a minimum
            %distance away from the walls
            bot.pos = [0 0];
            bot.ang = 2*pi*rand(1);
            bot.dir = [cos(bot.ang) sin(bot.ang)];
            %bot.updateScanLines(0,1);
        end

        
        %% high-level methods (equivalent to botsim)
        
        function move(bot, distance_m)
           distance_cm = distance_m/100;
           bot.translate(50, distance_cm); 
        end
        
        function debug = debug(bot)
            debug = 1;
        end
        
        function turn(bot, radiant_deg)
            angle_deg = radiant_deg *(180 / pi); 
            bot.rotate(50, angle_deg);
        end
        
        function distances_cm = ultraScan(bot)
            % botSim defaults to 6 scan lines over 360*, hence angle is 60*
            distances_cm = bot.survey(25, 60);
            distances_cm = distances_cm(:,2);
        end
        
        %% low-level methods
       
        function distance_cm = getDistance_cm(bot)
            distance_cm = GetUltrasonic(SENSOR_3);
        end

        function touch = getTouch(bot)
            touch = GetSwitch(SENSOR_1);
        end
        
        function rotate(bot, power_pct, angle_deg)
            tachoLimit = int32((bot.RobotRadius_m/bot.WheelRadius_m)*abs(angle_deg));

            bot.MotorA.Power = sign(angle_deg)*power_pct;
            bot.MotorA.TachoLimit = tachoLimit;
                        
            bot.MotorA.SendToNXT();

            bot.MotorB.Power = -sign(angle_deg)*power_pct;            
            bot.MotorB.TachoLimit = tachoLimit;
            
            bot.MotorB.SendToNXT();
            
            bot.MotorA.WaitFor();
            bot.MotorB.WaitFor();
            
            bot.MotorA.Stop();
            bot.MotorB.Stop();
        end
       
        function rotateDistanceSensor(bot, power_pct, angle_deg)
            bot.MotorC.Power = sign(angle_deg)*power_pct;
            bot.MotorC.TachoLimit = abs(angle_deg);
                        
            bot.MotorC.SendToNXT();

            bot.MotorC.WaitFor();
            
            bot.MotorC.Stop();
        end
        
        % rotate anti-clockwise
        function distances_cm = survey(bot, power_pct, angle_deg)
            count = 360/angle_deg;
            distances_cm = zeros(count, 2);
            
            % set initial position
            totAngle_deg = 180;
            rotateDistanceSensor(bot, power_pct, totAngle_deg);
            
            for i = 1:count
                % obtain distance measurement
                distances_cm(i,:) = [ totAngle_deg, getDistance_cm(bot) ];
                
                if i < count
                    % update position 
                    rotateDistanceSensor(bot, power_pct, -angle_deg);
                    totAngle_deg = totAngle_deg - angle_deg;
                end
            end
            
            % reset position
            rotateDistanceSensor(bot, power_pct, -totAngle_deg);
            
            % add 360 to negative angles for angular range from 0* to 360*
            j = find(distances_cm(:, 1) < 0);
            distances_cm(j, 1) = distances_cm(j, 1) + 360;
            
            % sort distances by angle
            distances_cm = sortrows(distances_cm);
            
            % adjust distances according to results of calibration
            %distances_cm(:,2) = 1.0149*distances_cm(:,2) - 6.5156;
        end
        
        function translate(bot, power_pct, distance_m)
            bot.MotorsAB.Power = sign(distance_m)*power_pct;
            bot.MotorsAB.TachoLimit = int32(abs((distance_m/bot.WheelCircumference_m)*360));
            
            bot.MotorsAB.SendToNXT();
            
            bot.MotorsAB.WaitFor();
            
            bot.MotorsAB.Stop();
        end
        
         function setMap(bot,newMap)
            bot.unmodifiedMap = newMap;
            bot.inpolygonMapformatX = cat(1,newMap(:,1), newMap(1,1));
            bot.inpolygonMapformatY = cat(1,newMap(:,2), newMap(1,2));
            
            newMap(length(newMap)+1,:)= newMap(1,:);
            bot.map = newMap;
            bot.mapLines = zeros(length(bot.map)-1,4);  %each row represents a border of the map
            for i =1:size(bot.mapLines,1)
                bot.mapLines(i,:) = [bot.map(i,:) bot.map(i+1,:)] ;
            end
        end
        
         %% drawing functions
        function drawMap(bot)
            plot(bot.map(:,1),bot.map(:,2),'lineWidth',2,'Color','r'); % draws arena
            axis equal; %keeps the x and y scale the same
            hold on; %
        end
        
        function drawBot(bot,lineLength,col)
            if nargin <3
                col = 'b'; %default color
            end
            %             plot(bot.pos(1),bot.pos(2),'Marker','o','Color',col);
            plot(bot.pos(1),bot.pos(2),'o');
            plot([bot.pos(1) bot.pos(1)+bot.dir(1)*lineLength],[bot.pos(2) bot.pos(2)+bot.dir(2)*lineLength],col);
        end
        
        function drawScanConfig(bot)
            drawLines(bot.scanLines,'g');
        end
    end
end

