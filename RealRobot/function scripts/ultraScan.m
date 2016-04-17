function [radii, angles] = ultraScan(motor,scanSpeed,samples)
% Function which given a ultrasound scan speed and sample rate, operates a
% motor ontop of which an ultrasound sensor is mounted. 

%motor.stop('Brake'); % cancels any previous movement that may be happening
motor.ActionAtTachoLimit = 'Brake'; % we want precise movment
motor.Power = -scanSpeed; % so it turns counterclockwise
motor.SmoothStart = false; % we want the scan to be as linear as possible
disp('scanning...');
angleIt = 360/samples;
angles = zeros(samples,1); % preallocate
radii = zeros(samples,1); % preallocate
for i = 0:samples-1
    radii(i+1) = GetUltrasonic(SENSOR_4);
    angles(i+1) = i*angleIt;
    motor.TachoLimit =angleIt;
    motor.SendToNXT(); % move motor
    motor.WaitFor();    
end

%% move the motor back to it's original position
% Note: a second second scan could be done at this point and merged with
% previous data. Alternatively, motor could be kept in this position as a 
% starting point for the next scan. In this case, the motor would have turn 
% in the opposite direction to prevent any cables from getting tangled. 

disp('returning to original position');
motor.Power = scanSpeed;
motor.TachoLimit = 360;
motor.SendToNXT(); %move motor
motor.WaitFor();
end
