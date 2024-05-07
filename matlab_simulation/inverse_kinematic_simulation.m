clc
clear
%% Arm Parameters
L1 = 450; % 2nd Axis to 3rd Axis Link Length (mm)
L2 = 322.765; % 3rd Axis to 4th/5th Axis Link Length (mm)
L3 = 230; % 4th/5th Axis to Gripper Tip Link Length (mm)
theta2_home = 60; % 2nd axis angle degrees, LIMIT SWITCH home (degrees)
theta3_home = -170; % 3rd axis angle degrees, LIMIT SWITCH home (degrees) 

% theta3 ACTUAL is the angle of theta3 relative to global coordinate:
% theta3_actual = theta3_home + theta2_home

% Encoder
AngleIncrement2nd = 360/8192;
AngleIncrement3rd = 360/8192;
encoder_count_2nd = 0; % ASSUMED POSITION MOVED DURING MANUAL CONTROL (degrees)
encoder_count_3rd = 0; % ASSUMED POSITION MOVED DURING MANUAL CONTROL (degrees)
current_theta2nd = theta2_home + AngleIncrement2nd*encoder_count_2nd; % Calculate current angle of 2nd axis encoder (degrees)
current_theta3rd = theta3_home + AngleIncrement3rd*encoder_count_3rd; % Calculate current angle of 3rd axis encoder (degrees)

% FORWARD KINEMATIC
% X-distance in XY plane  [ADD this without IMU leveling+ L3*cos(theta2nd + theta3rd + roll)] 
x = L1*cos(current_theta2nd*(pi/180)) + L2*cos((current_theta2nd + current_theta3rd)*(pi/180)); % x-plane distance (mm)
% Y-distance in XY plane  [ADD this without IMU leveling + L3*sin(theta2nd + theta3rd + roll))] 
y = L1*sin(current_theta2nd*(pi/180)) + L2*sin((current_theta2nd + current_theta3rd)*(pi/180)); % y-plane distance (mm)

% Define the start and end positions for x and y
startX = x; % Starting x-coordinate in mm
startY = y;   % Starting y-coordinate in mm
endX = startX + 300;   % Ending x-coordinate in mm
endY = startY + 0;    % Ending y-coordinate in mm

% Define number of steps for the movement
numSteps = 3000; % Resolution
x_values = linspace(startX, endX, numSteps); % Linearly spaced x values from start to end
y_values = linspace(startY, endY, numSteps); % Linearly spaced y values from start to end

%% Non-Animation Inverse Kinematic
c2 = (x*x + y*y - L1*L1 - L2*L2) / (2 * L1 * L2);
s2 = sqrt(1 - c2*c2);
target_3rdAngle = -atan2(s2, c2)*(180/pi); % Calculate target angle of 2nd axis IKE (degrees)
target_2ndAngle = (atan2(y,x) + atan2((L2*s2),(L1 + L2*c2)))*(180/pi); % Calculate target angle of 3rd axis IKE (degrees)

% Solving Error Differences
error2nd = target_2ndAngle - current_theta2nd; % Error between current reading of encoder vs target angle (degrees)
error3rd = target_3rdAngle - current_theta3rd; % Error between current reading of encoder vs target angle (degrees)
% Map Velocity
speed2nd = ((abs(error2nd) - 0) / (45 - 0))*(64-0) + 0; % Speed mapping for 2nd axis until reaches desired position
speed3rd = ((abs(error3rd) - 0) / (45 - 0))*(255-0) + 0; % Speed mapping for 3rd axis until reaches desired position 

%% Animation Setup
figure;
h1 = subplot(1, 2, 1);
hold on;
axis equal;
grid on;
xlabel('X (mm)');
ylabel('Y (mm)');
title('Robotic Arm Simulation');

% Initialize variables for plotting
armLine = plot(h1, [0, 0], [0, 0], '-o'); % Placeholder for the arm line
angleText = uicontrol('Style', 'text', 'String', 'Starting...', 'Position', [300 340 200 20], 'BackgroundColor', 'white');
positionText = uicontrol('Style', 'text', 'String', 'Starting...', 'Position', [300 320 200 20], 'BackgroundColor', 'white');
encoderText2nd = uicontrol('Style', 'text', 'String', 'Starting...', 'Position', [300 300 200 20], 'BackgroundColor', 'white');
encoderText3rd = uicontrol('Style', 'text', 'String', 'Starting...', 'Position', [300 280 200 20], 'BackgroundColor', 'white');

% Animation Loop
for k = 1:numSteps
    % Current target positions
    targetX = x_values(k);
    targetY = y_values(k);

    % Calculate joint angles using inverse kinematics
    [target_2ndAngle, target_3rdAngle] = inverseKinematics(targetX, targetY, L1, L2);
    target_2ndAngle = round(target_2ndAngle/AngleIncrement2nd)*AngleIncrement2nd; % Increment by 8192 resolution of encoder
    target_3rdAngle = round(target_3rdAngle/AngleIncrement3rd)*AngleIncrement3rd; % Increment by 8192 resolution of encoder

    % Calculate encoder reading
    encoder2nd = round((target_2ndAngle - theta2_home)/AngleIncrement2nd);
    encoder3rd = round((target_3rdAngle - theta3_home)/AngleIncrement3rd);

    % Calculate the arm's position based on angles
    armX = [0, L1 * cosd(target_2ndAngle), L1 * cosd(target_2ndAngle) + L2 * cosd(target_2ndAngle + target_3rdAngle)];
    armY = [0, L1 * sind(target_2ndAngle), L1 * sind(target_2ndAngle) + L2 * sind(target_2ndAngle + target_3rdAngle)];

    % Update the plot
    set(armLine, 'XData', armX);
    set(armLine, 'YData', armY);
    set(angleText, 'String', sprintf('Theta2 = %.2f°, Theta3 = %.2f°', target_2ndAngle, target_3rdAngle));
    set(positionText, 'String', sprintf('X = %.2f mm, Y = %.2f mm', armX(3), armY(3)));
    set(encoderText2nd, 'String', sprintf('Encoder 2nd = %.2f counts', encoder2nd));
    set(encoderText3rd, 'String', sprintf('Encoder 3rd = %.2f counts', encoder3rd));

    % Window Prompt Prinout
    fprintf('Step %d: Theta2 = %.2f, Theta3 = %.2f, Arm X = %.2f, Arm Y = %.2f\n', k, target_2ndAngle, target_3rdAngle, armX(3), armY(3));

    % Pause for the animation effect
    pause(0.001);
end

% Function for Inverse Kinematics
function [target_2ndAngle, target_3rdAngle] = inverseKinematics(x, y, L1, L2)
    x = x + 0; % CHANGE HERE FOR THE DISTANCE TRAVELED ALONG X (HORIZONTAL)
    y = y - 0; % CHANGE HERE FOR THE DISTANCE TRAVELED ALONG X (HORIZONTAL)
    c2 = (x*x + y*y - L1*L1 - L2*L2) / (2 * L1 * L2);
    s2 = sqrt(1 - c2*c2);
    target_3rdAngle = -atan2(s2, c2)*(180/pi); % Calculate target angle of 2nd axis IKE (degrees)
    target_2ndAngle = (atan2(y,x) + atan2((L2*s2),(L1 + L2*c2)))*(180/pi); % Calculate target angle of 3rd axis IKE (degrees)
end
