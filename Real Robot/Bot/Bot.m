classdef Bot
    %ROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        RobotRadius_m = 0.064;
        WheelRadius_m = 0.022;
        WheelCircumference_m = 2*pi*Bot.WheelRadius_m;
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
        
        function bot = Bot()
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
        
        %% high-level methods (equivalent to botsim)
        
        function complete(bot)
            NXT_PlayTone(440, 500);    
        end
        
        function move(bot, distance_cm)
           bot.translate(50, distance_cm/100); 
        end
        
        function turn(bot, angle_rad)
            bot.rotate(50, toDegrees('radians', angle_rad));
        end
        
        function distances_cm = ultraScan(bot)
% 6 scans
            survey = bot.survey(25, 60);
            
            distances_cm = [ ...
                survey(4, 2); ... % 0* 
                survey(5, 2); ... % 60*
                survey(6, 2); ... % 120*
                survey(1, 2); ... % 180*
                survey(2, 2); ... % 240*
                survey(3, 2) ];    % 300*

% 4 scans
%             survey = bot.survey(25, 90);
%             
%             distances_cm = [ ...
%                 survey(3, 2); ... % 0* 
%                 survey(4, 2); ... % 90*
%                 survey(1, 2); ... % 180*
%                 survey(2, 2) ];   % 270*
            
            if any(distances_cm(:) < 0)
                distances_cm = -1;
            end
        end
        
        %% low-level methods
       
        function calibratedDistance_cm = getDistance_cm(bot)
            distance_cm = GetUltrasonic(SENSOR_3);
            
            if (distance_cm > 0 && distance_cm < 200)
                calibratedDistance_cm = 1.0078 * distance_cm + 0.2267;
            else
                calibratedDistance_cm = -1;
            end
        end

        function touch = getTouch(bot)
            touch = GetSwitch(SENSOR_1);
        end
        
        function rotate(bot, power_pct, angle_deg)
            calibratedAngle_deg = 1.074 * angle_deg + 3.8933;
            tachoLimit = int32((bot.RobotRadius_m/bot.WheelRadius_m)*abs(calibratedAngle_deg));

            bot.MotorA.Power = sign(calibratedAngle_deg)*power_pct;
            bot.MotorA.TachoLimit = tachoLimit;
                        
            bot.MotorA.SendToNXT();

            bot.MotorB.Power = -sign(calibratedAngle_deg)*power_pct;            
            bot.MotorB.TachoLimit = tachoLimit;
            
            bot.MotorB.SendToNXT();
            
            bot.MotorA.WaitFor();
            bot.MotorB.WaitFor();
            
            bot.MotorA.Stop();
            bot.MotorB.Stop();
        end
       
        function rotateDistanceSensor(bot, power_pct, angle_deg)
            bot.MotorC.ActionAtTachoLimit = 'Brake';
            bot.MotorC.Power = sign(angle_deg)*power_pct;
            bot.MotorC.SmoothStart = false;
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
                distance_cm = getDistance_cm(bot)
                % obtain distance measurement
                distances_cm(i,:) = [ totAngle_deg, distance_cm ];
                
                if i < count
                    % update position 
                    rotateDistanceSensor(bot, power_pct, -angle_deg);
                    totAngle_deg = totAngle_deg - angle_deg;
                end
            end
            
            % reset position
            rotateDistanceSensor(bot, power_pct, -totAngle_deg);
        end
        
        function translate(bot, power_pct, distance_m)
            calibratedDistance_m = 1.0469 * distance_m + 0.001326;
            
            bot.MotorsAB.Power = sign(calibratedDistance_m)*power_pct;
            bot.MotorsAB.TachoLimit = int32(abs((calibratedDistance_m/bot.WheelCircumference_m)*360));
            
            bot.MotorsAB.SendToNXT();
            
            bot.MotorsAB.WaitFor();
            
            bot.MotorsAB.Stop();
        end
    end
end

