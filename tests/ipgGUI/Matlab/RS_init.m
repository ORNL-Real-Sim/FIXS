addpath(genpath('YAMLMatlab_0.4.3'))
addpath('RS files')
spdlimMap = readmatrix('C:\CM_Projects\Super_RS\tests\I81Small\Simulation\spdLimMapNB.csv');
spdlimMap = spdlimMap(2:end,:);
spdlimVolvo = readmatrix('C:\CM_Projects\Super_RS\CM11_proj\src_tm4sl\ST3_Route3_speed_profile.xlsx');
spdlimMapVolvo = spdlimVolvo(7104:141930, 2:3);

configFilename = 'C:\CM_Projects\Super_RS\tests\I81Small\RS_config_release.yaml';
[VehicleMessageFieldDefInputVec, VehDataBus, TrafficLayerIP, TrafficLayerPort] = RealSimInitSimulink(configFilename);
RealSimPara = struct;
RealSimPara.warmupTime = 20;
RealSimPara.speedInit = 50; % Initial speed of the ego vehicle when entering SUMO network
RealSimPara.tLookahead = 0.1; % Use 0.1 for external control, recommend tLookahead >= 0.2 for SUMO driver
RealSimPara.smoothWindow = 1; % Number of moving average data points, 1 means no moving average
% RealSimPara.speedSource = 3; % Use speedSource = 3 for dummy vehicle dynamics (simple transfer function)
RealSimPara.speedSource = 2; % Use speedSource = 2 for real vehicle dynamics (simple transfer function)
load histRun
MFC_ACC_Script

ci = 3; v_ini = 31.3;
kp = lowKp(ci); ki = lowKi(ci); kf = lowKf(ci); 
v_cruise = v_ini;
v_ratio = 1;