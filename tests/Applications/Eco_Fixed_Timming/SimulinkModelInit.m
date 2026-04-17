% File: realsim_script.m


simModelName = 'CAVE_MachE_dSPACE_250912';% 

% Clear workspace and console
disp(['Current folder: ', pwd]);
close all;
clearvars -except settingDir simModelName config_file_name; % Keep input variables
clc;
format compact;

% Different Vehicle Model, if different, need change!!!!!

addpath(genpath('.\\Vehicles_24a'))
configPath = 'C:\Users\hg25079\Documents\GitHub\FIXS\tests\Applications\SUMO_CARLA_EcoDriving\MLK_Sumo_Scenario\config_Sumo.yaml';



% Initializations
RealSimPath = 'C:\Users\hg25079\Documents\GitHub\FIXS\CommonLib';
% Add path of RealSim tools
addpath(genpath(RealSimPath));
% Initialize RealSim for Simulink, Read yaml file
disp(['configPath: ', configPath]);
%print configPath
fprintf('Configuration path: %s\n', configPath);

[VehicleMessageFieldDefInputVec, VehDataBus, TrafficLayerIP, TrafficLayerPort] = RealSimInitSimulink(configPath);

% Define RealSim parameters
RealSimPara = struct;
RealSimPara.warmupTime = 0;
RealSimPara.speedInit = 1; % Initial speed of the ego vehicle when entering SUMO network
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


