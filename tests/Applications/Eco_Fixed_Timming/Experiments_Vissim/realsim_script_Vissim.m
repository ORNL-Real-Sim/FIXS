% File: realsim_script.m

function realsim_script(settingDir, simModelName)
    % Clear workspace and console
    disp(['Current folder: ', pwd]);
    close all;
    clearvars -except settingDir simModelName; % Keep input variables
    clc;
    format compact;
    addpath(genpath('..\\..\\..\\VehicleDynamicsModels'))
    % addpath(genpath('..\\..\\..\\ICV'))
    % Initializations
    RealSimPath = '..\\..\\..\\..\\..\\..\\CommonLib';
    % Add path of RealSim tools
    addpath(genpath(RealSimPath));
    % Initialize RealSim for Simulink, Read yaml file
    configPath = [settingDir '\ecodriving_config_Vissim.yaml'];
    %print configPath
    fprintf('Configuration path: %s\n', configPath);
    [VehicleMessageFieldDefInputVec, VehDataBus, TrafficLayerIP, TrafficLayerPort] = RealSimInitSimulink(configPath);

    % Define RealSim parameters
    RealSimPara = struct;
    RealSimPara.warmupTime = 20;
    RealSimPara.speedInit = 0; % Initial speed of the ego vehicle when entering SUMO network
    RealSimPara.tLookahead = 0.1; % Use 0.1 for external control, recommend tLookahead >= 0.2 for SUMO driver
    RealSimPara.smoothWindow = 1; % Number of moving average data points, 1 means no moving average
    RealSimPara.speedSource = 3; % Use speedSource = 3 for dummy vehicle dynamics (simple transfer function)

    % Start RealSim Procedure
    fprintf('Starting RealSim procedure...\n');
    %tic;

    % Start Simulink model
    % Specify the following:
    % 1) Simulink model name
    % 2) Stop time of the Simulink model
    % Assign RealSimPara to the base workspace
    
    assignin('base', 'RealSimPara', RealSimPara);
    assignin('base', 'VehicleMessageFieldDefInputVec', VehicleMessageFieldDefInputVec);
    assignin('base', 'VehDataBus', VehDataBus);
    assignin('base', 'TrafficLayerIP', TrafficLayerIP);
    assignin('base', 'TrafficLayerPort', TrafficLayerPort);
    
    load_system(simModelName)
    VehicleOut = sim(simModelName); % alternatively can use 'sim' command
    % Load and run the Simulink model
    Time = VehicleOut.desired_speed.Time;                  
    DesiredSpeed = VehicleOut.desired_speed.Data;
    DesiredSpeed = DesiredSpeed(:);
    ActualSpeed = VehicleOut.actual_speed.Data;
    ActualSpeed = ActualSpeed(:);
    MPGe = VehicleOut.MPGe.Data;
    BatteryCurrent = VehicleOut.battery_current.Data;
    BatteryPower = VehicleOut.battery_power.Data;
    BatterySOC = VehicleOut.battery_soc.Data;
    MotorSpeed = VehicleOut.motor_speed.Data;
    MotorTorque = VehicleOut.motor_torque.Data;
    
    save('vehData', 'VehicleOut')
    T = table(Time, DesiredSpeed, ActualSpeed, MPGe, BatteryCurrent, BatteryPower, BatterySOC, MotorSpeed, MotorTorque,'VariableNames', {'Time', 'DesiredSpeed', 'ActualSpeed', 'MPGe', 'BatteryCurrent', 'BatteryPower','BatterySOC', 'MotorSpeed', 'MotorTorque'});
    writetable(T, [settingDir, '\ego_profile.csv']);

end
