classdef robot
    %ROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        RobotRadius_m = 0.064;
        WheelRadius_m = 0.022;
        WheelCircumference_m = 2*pi*robot.WheelRadius_m;
    end
    
    properties
        Handle
        MotorA
        MotorB
        MotorC
        MotorsAB
    end
    
    methods
        
        function obj = robot()
            obj.Handle = COM_OpenNXT();
            COM_SetDefaultNXT(obj.Handle);
            
            obj.MotorA = NXTMotor('A');
            obj.MotorA.ActionAtTachoLimit = 'Brake';
            obj.MotorA.SmoothStart = true;
            obj.MotorA.SpeedRegulation = false;
            
            obj.MotorB = NXTMotor('B');
            obj.MotorB.ActionAtTachoLimit = 'Brake';
            obj.MotorB.SmoothStart = true;
            obj.MotorB.SpeedRegulation = false;
            
            obj.MotorC = NXTMotor('C');
            
            obj.MotorsAB = NXTMotor('AB');
            obj.MotorsAB.ActionAtTachoLimit = 'Brake';
            obj.MotorA.SmoothStart = true;
            obj.MotorsAB.SpeedRegulation = false;
            
            OpenSwitch(SENSOR_1);
            OpenUltrasonic(SENSOR_3);
            
            NXT_GetBatteryLevel()
        end
        
        function delete(obj)
            obj.MotorA.Stop('off');
            obj.MotorB.Stop('off');
            obj.MotorC.Stop('off');
            obj.MotorsAB.Stop('off');

            CloseSensor(SENSOR_1);
            CloseSensor(SENSOR_3);
            
            COM_CloseNXT(obj.Handle);
        end
        
        function rotate(obj, power, angle_deg)
            tachoLimit = int32((obj.RobotRadius_m/obj.WheelRadius_m)*abs(angle_deg));

            obj.MotorA.Power = sign(angle_deg)*power;
            obj.MotorA.TachoLimit = tachoLimit;
                        
            obj.MotorA.SendToNXT();

            obj.MotorB.Power = -sign(angle_deg)*power;            
            obj.MotorB.TachoLimit = tachoLimit;
            
            obj.MotorB.SendToNXT();
            
            obj.MotorA.WaitFor();
            obj.MotorB.WaitFor();
            
            obj.MotorA.Stop();
            obj.MotorB.Stop();
        end
        
        function translate(obj, power, distance_m)
            obj.MotorsAB.Power = sign(distance_m)*power;
            obj.MotorsAB.TachoLimit = int32(abs((distance_m/obj.WheelCircumference_m)*360));
            
            obj.MotorsAB.SendToNXT();
            
            obj.MotorsAB.WaitFor();
            
            obj.MotorsAB.Stop();
        end
        
        function distances_cm = survey(obj, power, angle_deg)
            count = abs(360/angle_deg);
            distances_cm = zeros(count, 2);
            
            % set initial position
            totAngle_deg = 180;
            rotateDistanceSensor(obj, power, totAngle_deg);
            
            for i = 1:count
                % obtain distance measurement
                distances_cm(i,:) = [ totAngle_deg, getDistance_cm(obj) ];
                
                if i < count
                    % update position 
                    rotateDistanceSensor(obj, power, angle_deg);
                    totAngle_deg = totAngle_deg + angle_deg;
                end
            end
            
            % reset position
            rotateDistanceSensor(obj, power, -totAngle_deg);
        end
        
        function rotateDistanceSensor(obj, power, angle_deg)
            obj.MotorC.Power = sign(angle_deg)*power;
            obj.MotorC.TachoLimit = abs(angle_deg);
                        
            obj.MotorC.SendToNXT();

            obj.MotorC.WaitFor();
            
            obj.MotorC.Stop();
        end
        
        function distance_cm = getDistance_cm(obj)
            distance_cm = GetUltrasonic(SENSOR_3);
        end

        function touch = getTouch(obj)
            touch = GetSwitch(SENSOR_1);
        end
    end
    
end

