% Importing of the model 
drone = importrobot('Drone_Assembly_MATLAB.urdf');
showdetails(drone)

% Displaying the model
figure
show(drone, 'Visuals', 'on', 'Collisions', 'off');
title('Drone Assembly URDF Visualization');
axis equal
view(135, 20);

% Create a figure for animation
figure('Name','Drone Joint Animation','Color','w');
ax = show(drone, 'Visuals', 'on', 'PreservePlot', false);
view(135, 25);
axis equal
title('Animating Propeller Joints');
hold on

% Get home configuration
config = homeConfiguration(drone);

% Animation parameters
numFrames = 100;  % frames per rotation
rotSpeed = 2*pi;  % radians per second equivalent speed

% Loop through each joint
for i = 1:numel(config)
    disp(['Animating ', drone.BodyNames{i}, ' ...'])

    for theta = linspace(0, 2*pi, numFrames)
        config(i).JointPosition = theta;  % rotate current propeller
        show(drone, config, 'PreservePlot', false, 'Frames', 'off', 'Parent', ax);
        drawnow;
    end

    % Reset joint before moving to next
    config(i).JointPosition = 0;
end

title('All Propellers Verified ');

