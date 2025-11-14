%% setup_drone_complete.m
% Initializes variables for Drone_UAVScenario.slx
% FIXES: Strictly obeys the Block's Definition (No extra fields).

clear; clc;
disp('🚁 Initializing Drone Simulation...');

%% 1. Simulation Parameters
droneMass = 2.83;
initPos   = [0 0 0];
initYaw   = 0;

% PID Gains
Kp_pos = 2.0; Kd_pos = 1.5;
Kp_z   = 2.0; Kd_z   = 1.0;
Kp_yaw = 1.5; 

% SQUARE TRAJECTORY (NED Frame: -2 is Up)
waypoints = [0 0 -2; 10 0 -2; 10 10 -2; 0 10 -2; 0 0 -2];

%% 2. Load Robot
disp('Loading URDF...');
urdfFile = '/home/siddx/Documents/Drone_MATLAB_SIMULATION/Drone_Assembly_MATLAB_ready/urdf/Drone_Assembly_MATLAB.urdf';
if isfile(urdfFile)
    try
        robot = importrobot(urdfFile);
        robot.DataFormat = 'row';
        robot.Gravity = [0 0 -9.81];
        disp('✅ Custom URDF loaded.');
    catch
        robot = loadrobot('parrotMambo', 'DataFormat', 'row');
    end
else
    robot = loadrobot('parrotMambo', 'DataFormat', 'row');
end

%% 3. AUTO-GENERATE BUS DEFINITIONS
disp('Aligning Bus Definitions...');
modelName = 'Drone_UAVScenario'; 
if ~bdIsLoaded(modelName), load_system(modelName); end

% Find the block to get the official definitions
blk = find_system(modelName, 'BlockType', 'SubSystem', 'Name', 'Guidance Model');

if isempty(blk)
    error('❌ Guidance Model block not found!');
else
    % 1. Reset the block's internal definitions (Factory Reset)
    uav.sluav.internal.guidancemodel.mask.createUAVBuses(blk{1});
    
    % 2. Create structures FROM the definitions
    ctrlData = Simulink.Bus.createMATLABStruct('MultirotorGuidanceControlBus');
    envData  = Simulink.Bus.createMATLABStruct('MultirotorGuidanceEnvironmentBus');
    
    % 3. Populate Environment Data (STRICT MODE)
    % We only assign values if the BUS actually requested that field.
    
    % --- GRAVITY CHECK ---
    gravElem = MultirotorGuidanceEnvironmentBus.Elements(strcmp({MultirotorGuidanceEnvironmentBus.Elements.Name}, 'Gravity'));
    if isequal(gravElem.Dimensions, 1) || isequal(gravElem.Dimensions, [1 1])
        envData.Gravity = 9.81; % Scalar
        disp('ℹ️  Block requires Scalar Gravity.');
    else
        envData.Gravity = [0 0 9.81]; % Vector
        disp('ℹ️  Block requires Vector Gravity.');
    end
    
    % --- OPTIONAL FIELDS CHECK ---
    % Only add these if the block asked for them
    if isfield(envData, 'AirPressure')
        envData.AirPressure = 101325;
    end
    if isfield(envData, 'Temperature')
        envData.Temperature = 288.15;
    end
    if isfield(envData, 'MagneticField')
        envData.MagneticField = [0 0 0];
    end
    if isfield(envData, 'Wind')
        envData.Wind = [0 0 0];
    end
    
    disp('✅ Bus Definitions & Data aligned perfectly.');
end

%% 4. Display Required Wiring Order
disp('---------------------------------------------------');
disp('⬇️  VERIFY YOUR BUS CREATOR WIRING ORDER (Top->Bottom) ⬇️');
disp(fieldnames(ctrlData)); 
disp('---------------------------------------------------');
disp('🚀 READY TO RUN');