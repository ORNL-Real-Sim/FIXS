% This is a script for Merging Control of ACM
clc
clear all
%close all
set(0,'DefaultAxesFontName', 'Times');
set(0,'DefaultAxesFontSize', 12);
set(0,'DefaultTextFontname', 'Times');
set(0,'DefaultTextFontSize', 12);

%% Traci setup 
import traci.constants
ScenarioPath = 'C:\Program Files (x86)\Eclipse\Sumo\tools\contributed\traci4matlab\ZJ_Learn_Matlab4Traci\CVXGEN_ACM\acm-main\ACM_SUMO_ZJ\ACM_5CARS_HDVPSG.sumocfg';

%% Import Real Packet Drop
load('PDR_All.mat')              % In total, 147247 groups of REAL RECOREDE packet drop at each step, with V2VC rate = 10ms;
load('PDR_All_Flags')     % Each group correponds to 3000 [1,0]: 1 received, 0 packet drop

% We do not use Model or Simulator, all the PDR are real-recorded;
% Therefore, we do not use Random number
% Instead, we need to have 4 PDR record since we have 4 followers
% Therefore, we can rotate the record for each vehicle 
% You can have 4*3*2*1 = 24 simulations in total for each PDR, all based on
% recorded REAL PDR.

% 90% Recved: 10180, 17063, 21800, 34799
% 80% Recved: 8400,  38590, 54783, 107800
% 70% Recved: 39080, 58740, 84140, 108300
% 60% Recved: 39460, 46240, 59130, 60910
% 50% Recved: 47060, 63430, 68410, 117500
% 40% Recved: 47740, 47780, 49536, 118106

Recved_90_1 = Recved_Flag_Combined(10180,:);   % Max_consecutive_zeros: 4
Recved_90_2 = Recved_Flag_Combined(17063,:);   % Max_consecutive_zeros: 16
Recved_90_3 = Recved_Flag_Combined(21800,:);   % Max_consecutive_zeros: 5
Recved_90_4 = Recved_Flag_Combined(34799,:);   % Max_consecutive_zeros: 5

Recved_80_1 = Recved_Flag_Combined(8400,:);    % Max_consecutive_zeros: 40
Recved_80_2 = Recved_Flag_Combined(38590,:);   % Max_consecutive_zeros: 22
Recved_80_3 = Recved_Flag_Combined(54783,:);   % Max_consecutive_zeros: 30
Recved_80_4 = Recved_Flag_Combined(107800,:);  % Max_consecutive_zeros: 46

Recved_70_1 = Recved_Flag_Combined(39080,:);   % Max_consecutive_zeros: 47
Recved_70_2 = Recved_Flag_Combined(58740,:);   % Max_consecutive_zeros: 49
Recved_70_3 = Recved_Flag_Combined(84140,:);   % Max_consecutive_zeros: 46
Recved_70_4 = Recved_Flag_Combined(108300,:);  % Max_consecutive_zeros: 46

Recved_60_1 = Recved_Flag_Combined(39460,:);   % Max_consecutive_zeros: 47
Recved_60_2 = Recved_Flag_Combined(46240,:);   % Max_consecutive_zeros: 39
Recved_60_3 = Recved_Flag_Combined(59130,:);   % Max_consecutive_zeros: 49
Recved_60_4 = Recved_Flag_Combined(60910,:);   % Max_consecutive_zeros: 49

Recved_50_1 = Recved_Flag_Combined(47060,:);   % Max_consecutive_zeros: 39
Recved_50_2 = Recved_Flag_Combined(63430,:);   % Max_consecutive_zeros: 46
Recved_50_3 = Recved_Flag_Combined(68410,:);   % Max_consecutive_zeros: 47
Recved_50_4 = Recved_Flag_Combined(117500,:);  % Max_consecutive_zeros: 44

Recved_40_1 = Recved_Flag_Combined(47740,:);   % Max_consecutive_zeros: 43
Recved_40_2 = Recved_Flag_Combined(47780,:);   % Max_consecutive_zeros: 43
Recved_40_3 = Recved_Flag_Combined(49536,:);   % Max_consecutive_zeros: 43
Recved_40_4 = Recved_Flag_Combined(118106,:);  % Max_consecutive_zeros: 47

%% Attention!!!!!!!!!!!!!
% For this Low-level control paper, we have changed the .route file, which
% guarantees: 
% 0). MP moved forward to avoid lateral collision, and removes XY
% transition
% 1). FIFO and thus the follower alwayas has a leader
% 2). Maybe reduce the simulation time and avoid collision

% Traci start: You must traci.start before using any Traci command
% We set basic setup step = 0.01s; It seems like actionStep is irrelated if we control via Traci
step_length = ' --step-length ';
T_SUMO = 0.01; T_ADE  = 0.01;
T_SUMO = num2str(T_SUMO);
step_SUMO = [step_length ' ' T_SUMO];

[traciVersion,sumoVersion] = traci.start(['sumo-gui -c ' '"' ScenarioPath '"' step_SUMO ' --route-steps 0' ' --start']);  % Forced all car released at t = 0;

%[traciVersion,sumoVersion] = traci.start(['sumo -c ' '"' ScenarioPath '"' step_SUMO ' --start']);

traci.vehicle.setSpeedMode('PSG_first',  32)               % 32 will remove right-of-way and acceleration limit, but the car can still be limited by CF model even without Traci control 
traci.vehicle.setSpeedMode('PSG_middle', 32)
traci.vehicle.setSpeedMode('PSG_end',    32)
traci.vehicle.setSpeedMode('HDV_first',  32)
traci.vehicle.setSpeedMode('HDV_last',   32)

traci.vehicle.setLaneChangeMode('PSG_first',  256)         % Disable SUMO internal lane change, but still collision avoidance, even with Traci
traci.vehicle.setLaneChangeMode('PSG_middle', 256)
traci.vehicle.setLaneChangeMode('PSG_end',    256)
traci.vehicle.setLaneChangeMode('HDV_first',  256)
traci.vehicle.setLaneChangeMode('HDV_last',   256)

%% Vehicle Dynamics
global UT
UT.Ts_Veh = 0.01;    % Basic Simulink period = Veh Dyn Simulation period = SUMO period = ULMPC; In practice, SUMO/ULMPC period can be set larger
                      
% Values based of CarSim D-Class Sedan
UT.lf  =  1.11;        %m        length from c.g to front
UT.lr  =  1.66622;     %m        length from c.g to rear
UT.Ls  =  0.775;       %m        half of the track width, w=2*Ls
UT.Rw  =  0.330;       %m        Wheel Radius (Effective rolling radius)
UT.ha  =  0.72;        %m        height that the aerodynamic resistance force acts
UT.Mv  =  1530;        %kg       total mass of the vehicle
UT.Ms  =  1370;        %kg       sprung mass of the vehicle
UT.g   =  9.81;        %m/s^2    gravitational constant
UT.h   =  0.54;        %m        height from ground to c.g
UT.Kss =  153*10^3;    %N/m      suspension stiffness
UT.Cs  =  2.9447*10^3; %N*s/m    suspension damping
UT.rho =  1.206;       %kg/m^3   air mass density
UT.Cd  =  0.28;        %         aerodynamic drag coeff (at zero slip angle)
UT.Awind = 2.51;       %m^2      projected area of the vehicle in direction of travel
UT.Vw  = 0;            %m/s      wind speed
UT.CRF = 0.03;    
UT.Iz  = 4192;         %kg*m^2   Moment of inertia around the z-axis for Z-torque
UT.Ix  = 606.1;        %kg*m^2   Moment of inertia around the x-axis for X-torque
UT.Iy  = 4192;         %kg*m^2   Moment of inertia around the y-axis for Y-torque
UT.Jw  = 0.9;          %kg*m^2   Polar Moment of inertia around the wheel center

% PSG/HDV dynamics high/low TRFC: Just for modeling and motivation
PSG_HighMu_P    = 75;     PSG_LowMu_P    = 48;
PSG_HighMu_I    = 125;    PSG_LowMu_I    = 64;
PSG_HighMu_D    = 15;     PSG_LowMu_D    = 12;  
PSG_HighMu_TRFC = 0.8;    PSG_LowMu_TRFC = 0.7;

HDV_HighMu_P    = 27;     HDV_LowMu_P    = 12;
HDV_HighMu_I    = 27;     HDV_LowMu_I    = 8;
HDV_HighMu_D    = 9;      HDV_LowMu_D    = 6;  
HDV_HighMu_TRFC = 0.7;    HDV_LowMu_TRFC = 0.6;

High_TRFC = 1;   % Will be a distance-to-MP function: have TRFC drop

% Initial conditions
% Speed set in SUMO rou
PSG_first_v0  = 13;   
PSG_middle_v0 = 13;
PSG_end_v0    = 13;
HDV_first_v0  = 13;
HDV_last_v0   = 13;

% TCC High-Level Commands (Needs to be designed by yourself)
MG_Seq          = 'HDV_first, HDV_last, PSG_first, PSG_middle, PSG_end';
DELTA_DR_MG_TCC = 15;     % Inter-vehicle distance at the MG point for ALL
VR_MG_TCC       = 15;     % Prescribed speed at the MG point for ALL

% General MPC Setup
Np_MPC      =  20;
HDV_AMAX    =  1.3;   % Conservative per Low TRFC case
HDV_AMIN    = -2.5; 
PSG_AMAX    =  3.2;   % Conservative per Low TRFC case
PSG_AMIN    = -6;
V_MAX       =  40;
V_MIN       =  0;
% For Reference Generation & its trigger
CtrlZone_Length = 250;  % Of course, less than the initial dis to MP 

% ULMPC Leader Speed Ctrl Only
BETA_HDV_Leader = 50;
Q_Leader = 1;
R_Leader = 20;

% Follower MPC Setup
DELTA_D_MIN             = 5;
BETA_HDV_Follower       = 100;  %50 not good :chatterring
BETA_PSG_Follower       = 150;

Qd_HDV_Follower         = 10;   Qd_PSG_Follower         = 10;
Qd_dot_HDV_Follower     = 2;    Qd_dot_PSG_Follower     = 2;
R_HDV_Follower          = 1;    R_PSG_Follower          = 1;
Rho_v_HDV_Follower      = 1e3;  Rho_v_PSG_Follower      = 1e3;
Rho_deltad_HDV_Follower = 1e3;  Rho_deltad_PSG_Follower = 1e3;  % Dynamic in practice

V2V_SamplingPeriod = 0.01;
%% SIM
sim('HDVPSG_DelayComp_NewRef_RealDrop_case1')

%% Fig_Parameters
lw = 2;
ftsz = 10;
pos_fig = [200 -200 1000 800];

% figure, plot(ans.ADE_Summary.Time, ans.ADE_Summary.Data(:,1))

%% Reference and Actual Speed DUE TO VEH DYNAMICS
% figure
% set(gcf,'paperpositionMode','auto','position',pos_fig)
% subplot(2,5,1)
% plot(ans.HDV_First_vx_des_real.Time, ans.HDV_First_vx_des_real.Data(:,1), 'linewidth', lw);
% grid on;
% hold on;
% plot(ans.HDV_First_vx_des_real.Time, ans.HDV_First_vx_des_real.Data(:,2), '-.', 'linewidth', lw);
% legend('Real Speed','Reference Speed')
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('HDV First Speed (m/s)','Interpreter', 'LaTeX','fontsize',ftsz)
% 
% subplot(2,5,6)
% plot(ans.HDV_First_vx_des_real.Time, ans.HDV_First_vx_des_real.Data(:,1) - ans.HDV_First_vx_des_real.Data(:,2), 'linewidth', lw);
% grid on;
% hold on;
% legend('HDV1 Speed Error (m/s)')
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% 
% subplot(2,5,2)
% plot(ans.HDV_Last_vx_des_real.Time, ans.HDV_Last_vx_des_real.Data(:,1), 'linewidth', lw);
% grid on;
% hold on;
% plot(ans.HDV_Last_vx_des_real.Time, ans.HDV_Last_vx_des_real.Data(:,2), '-.','linewidth', lw);
% legend('Real Speed','Reference Speed')
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('HDV Last Speed (m/s)','Interpreter', 'LaTeX','fontsize',ftsz)
% 
% subplot(2,5,7)
% plot(ans.HDV_Last_vx_des_real.Time, ans.HDV_Last_vx_des_real.Data(:,1) - ans.HDV_Last_vx_des_real.Data(:,2), 'linewidth', lw);
% grid on;
% hold on;
% legend('HDV2 Speed Error (m/s)')
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% 
% subplot(2,5,3)
% plot(ans.PSG_First_vx_des_real.Time, ans.PSG_First_vx_des_real.Data(:,1), 'linewidth', lw);
% grid on;
% hold on;
% plot(ans.PSG_First_vx_des_real.Time, ans.PSG_First_vx_des_real.Data(:,2), '-.', 'linewidth', lw);
% legend('Real Speed','Reference Speed')
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('PSG First Speed (m/s)','Interpreter', 'LaTeX','fontsize',ftsz)
% 
% subplot(2,5,8)
% plot(ans.PSG_First_vx_des_real.Time, ans.PSG_First_vx_des_real.Data(:,1) - ans.PSG_First_vx_des_real.Data(:,2), 'linewidth', lw);
% grid on;
% hold on;
% legend('PSG1 Speed Error (m/s)')
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% 
% subplot(2,5,4)
% plot(ans.PSG_Middle_vx_des_real.Time, ans.PSG_Middle_vx_des_real.Data(:,1), 'linewidth', lw);
% grid on;
% hold on;
% plot(ans.PSG_Middle_vx_des_real.Time, ans.PSG_Middle_vx_des_real.Data(:,2),'-.', 'linewidth', lw);
% legend('Real Speed','Reference Speed')
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('PSG Middle Speed (m/s)','Interpreter', 'LaTeX','fontsize',ftsz)
% 
% subplot(2,5,9)
% plot(ans.PSG_Middle_vx_des_real.Time, ans.PSG_Middle_vx_des_real.Data(:,1) - ans.PSG_Middle_vx_des_real.Data(:,2), 'linewidth', lw);
% grid on;
% hold on;
% legend('PSG2 Speed Error (m/s)')
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% 
% subplot(2,5,5)
% plot(ans.PSG_End_vx_des_real.Time, ans.PSG_End_vx_des_real.Data(:,1), 'linewidth', lw);
% grid on;
% hold on;
% plot(ans.PSG_End_vx_des_real.Time, ans.PSG_End_vx_des_real.Data(:,2), '-.','linewidth', lw);
% legend('Real Speed','Reference Speed')
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('PSG End Speed (m/s)','Interpreter', 'LaTeX','fontsize',ftsz)
% set(gcf,'color','w');
% 
% subplot(2,5,10)
% plot(ans.PSG_End_vx_des_real.Time, ans.PSG_End_vx_des_real.Data(:,1) - ans.PSG_End_vx_des_real.Data(:,2), 'linewidth', lw);
% grid on;
% hold on;
% legend('PSG3 Speed Error (m/s)')
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)

%% MPC_Predicted_d against PDR
figure
set(gcf,'paperpositionMode','auto','position',pos_fig)
subplot(2,2,1)
plot(ans.deltad_agpdr_HDV_Last.Time, ans.deltad_agpdr_HDV_Last.Data(:,1), 'linewidth', lw);
grid on
hold on
plot(ans.deltad_agpdr_HDV_Last.Time, ans.deltad_agpdr_HDV_Last.Data(:,2), 'r-.', 'linewidth', lw/2);
plot(ans.deltad_agpdr_HDV_Last.Time, ans.deltad_agpdr_HDV_Last.Data(:,3), 'g--', 'linewidth', lw/2);
legend('With estimation','Measurement', 'Real')
xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
ylabel('HDV Last $$\delta_d$$ (m)','Interpreter', 'LaTeX','fontsize',ftsz)
title('Case1')

subplot(2,2,2)
plot(ans.deltad_agpdr_PSGFirst.Time, ans.deltad_agpdr_PSGFirst.Data(:,1), 'linewidth', lw);
grid on
hold on
plot(ans.deltad_agpdr_PSGFirst.Time, ans.deltad_agpdr_PSGFirst.Data(:,2), 'r-.', 'linewidth', lw/2);
plot(ans.deltad_agpdr_PSGFirst.Time, ans.deltad_agpdr_PSGFirst.Data(:,3), 'g--', 'linewidth', lw/2);
legend('With estimation','Measurement', 'Real')
xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
ylabel('PSG First $$\delta_d$$ (m)','Interpreter', 'LaTeX','fontsize',ftsz)

subplot(2,2,3)
plot(ans.deltad_agpdr_PSGMiddle.Time, ans.deltad_agpdr_PSGMiddle.Data(:,1), 'linewidth', lw);
grid on
hold on
plot(ans.deltad_agpdr_PSGMiddle.Time, ans.deltad_agpdr_PSGMiddle.Data(:,2), 'r-.', 'linewidth', lw/2);
plot(ans.deltad_agpdr_PSGMiddle.Time, ans.deltad_agpdr_PSGMiddle.Data(:,3), 'g--', 'linewidth', lw/2);
legend('With estimation','Measurement', 'Real')
xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
ylabel('PSG Middle $$\delta_d$$ (m)','Interpreter', 'LaTeX','fontsize',ftsz)

subplot(2,2,4)
plot(ans.deltad_agpdr_PSGEnd.Time, ans.deltad_agpdr_PSGEnd.Data(:,1), 'linewidth', lw);
grid on
hold on
plot(ans.deltad_agpdr_PSGEnd.Time, ans.deltad_agpdr_PSGEnd.Data(:,2), 'r-.', 'linewidth', lw/2);
plot(ans.deltad_agpdr_PSGEnd.Time, ans.deltad_agpdr_PSGEnd.Data(:,3), 'g--', 'linewidth', lw/2);
legend('With estimation','Measurement', 'Real')
xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
ylabel('PSG End $$\delta_d$$ (m)','Interpreter', 'LaTeX','fontsize',ftsz)

set(gcf,'color','w');

%% vcmd and delta_vcmd 
% figure
% set(gcf,'paperpositionMode','auto','position',pos_fig)
% subplot(2,2,1)
% plot(ans.v_cmd_HDV_First.Time,  ans.v_cmd_HDV_First.Data, 'r-.', 'linewidth', lw);
% grid on
% hold on
% plot(ans.v_cmd_HDV_Last.Time,   ans.v_cmd_HDV_Last.Data, 'r--',  'linewidth', lw);
% plot(ans.v_cmd_HDV_First.Time,  ones(length(ans.v_cmd_HDV_First.Time), 1)*V_MAX, 'k', 'linewidth', lw-0.5);
% plot(ans.v_cmd_HDV_First.Time,  ones(length(ans.v_cmd_HDV_First.Time), 1)*V_MIN, 'k', 'linewidth', lw-0.5);
% legend('HDV First','HDV Last', 'Upper bound', 'Lower bound')
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('HDV reference speed (m/s)','Interpreter', 'LaTeX','fontsize',ftsz)
% 
% subplot(2,2,2)
% plot(ans.v_cmd_PSG_First.Time,  ans.v_cmd_PSG_First.Data, 'b-.', 'linewidth', lw);
% grid on
% hold on
% plot(ans.v_cmd_PSG_Middle.Time, ans.v_cmd_PSG_Middle.Data, 'b--',  'linewidth', lw);
% plot(ans.v_cmd_PSG_End.Time,    ans.v_cmd_PSG_End.Data,    'b:', 'linewidth', lw);
% plot(ans.v_cmd_PSG_First.Time,  ones(length(ans.v_cmd_PSG_First.Time), 1)*V_MAX, 'k', 'linewidth', lw-0.5);
% plot(ans.v_cmd_PSG_First.Time,  ones(length(ans.v_cmd_PSG_First.Time), 1)*V_MIN, 'k', 'linewidth', lw-0.5);
% legend('PSG First','PSG Middle','PSG End', 'Upper bound', 'Lower bound')
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('PSG reference speed (m/s)','Interpreter', 'LaTeX','fontsize',ftsz)
% 
% subplot(2,2,3)
% plot(ans.deltav_cmd_HDV_First.Time, ans.deltav_cmd_HDV_First.Data, 'r-.', 'linewidth', lw);
% grid on
% hold on
% plot(ans.deltav_cmd_HDV_Last.Time,   ans.deltav_cmd_HDV_Last.Data, 'r--',  'linewidth', lw);
% plot(ans.deltav_cmd_HDV_First.Time,  ones(length(ans.deltav_cmd_HDV_First.Time), 1)*HDV_AMAX*T_ADE, 'k', 'linewidth', lw-0.5);
% plot(ans.deltav_cmd_HDV_First.Time,  ones(length(ans.deltav_cmd_HDV_First.Time), 1)*HDV_AMIN*T_ADE, 'k', 'linewidth', lw-0.5);
% legend('HDV First','HDV Last', 'Upper bound', 'Lower bound')
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('HDV reference speed change (m/s)','Interpreter', 'LaTeX','fontsize',ftsz)
% 
% subplot(2,2,4)
% plot(ans.deltav_cmd_PSG_First.Time,  ans.deltav_cmd_PSG_First.Data,  'b-.', 'linewidth', lw);
% grid on
% hold on
% plot(ans.deltav_cmd_PSG_Middle.Time, ans.deltav_cmd_PSG_Middle.Data, 'b--', 'linewidth', lw);
% plot(ans.deltav_cmd_PSG_End.Time,    ans.deltav_cmd_PSG_End.Data,    'b:', 'linewidth', lw);
% plot(ans.deltav_cmd_PSG_First.Time,  ones(length(ans.deltav_cmd_PSG_First.Time), 1)*PSG_AMAX*T_ADE, 'k', 'linewidth', lw-0.5);
% plot(ans.deltav_cmd_PSG_First.Time,  ones(length(ans.deltav_cmd_PSG_First.Time), 1)*PSG_AMIN*T_ADE, 'k', 'linewidth', lw-0.5);
% legend('PSG First','PSG Middle','PSG End', 'Upper bound', 'Lower bound')
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('PSG reference speed change (m/s)','Interpreter', 'LaTeX','fontsize',ftsz)
% 
% set(gcf,'color','w');

%% Reference speed tracking & Errors
% figure
% set(gcf,'paperpositionMode','auto','position',pos_fig)
% subplot(2,5,1)
% plot(ans.v_vr_HDV_First.Time, ans.v_vr_HDV_First.Data(:,1), 'linewidth', lw);
% grid on
% hold on
% plot(ans.v_vr_HDV_First.Time, ans.v_vr_HDV_First.Data(:,2), 'r-.', 'linewidth', lw);
% legend('Actual Speed','Desired Speed')
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('HDV First Speed (m/s)','Interpreter', 'LaTeX','fontsize',ftsz)
% 
% subplot(2,5,6)
% plot(ans.v_vr_HDV_First.Time, ans.v_vr_HDV_First.Data(:,1) - ans.v_vr_HDV_First.Data(:,2), 'linewidth', lw);
% grid on
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('HDV First Speed Error (m/s)','Interpreter', 'LaTeX','fontsize',ftsz)
% 
% subplot(2,5,2)
% plot(ans.v_vr_HDV_Last.Time, ans.v_vr_HDV_Last.Data(:,1), 'linewidth', lw);
% grid on
% hold on
% plot(ans.v_vr_HDV_Last.Time, ans.v_vr_HDV_Last.Data(:,2), 'r-.', 'linewidth', lw);
% legend('Actual Speed','Desired Speed')
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('HDV Last Speed (m/s)','Interpreter', 'LaTeX','fontsize',ftsz)
% 
% subplot(2,5,7)
% plot(ans.v_vr_HDV_Last.Time, ans.v_vr_HDV_Last.Data(:,1) - ans.v_vr_HDV_Last.Data(:,2), 'linewidth', lw);
% grid on
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('HDV Last Speed Error (m/s)','Interpreter', 'LaTeX','fontsize',ftsz)
% 
% subplot(2,5,3)
% plot(ans.v_vr_PSG_First.Time, ans.v_vr_PSG_First.Data(:,1), 'linewidth', lw);
% grid on
% hold on
% plot(ans.v_vr_PSG_First.Time, ans.v_vr_PSG_First.Data(:,2), 'r-.', 'linewidth', lw);
% legend('Actual Speed','Desired Speed')
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('PSG First Speed (m/s)','Interpreter', 'LaTeX','fontsize',ftsz)
% 
% subplot(2,5,8)
% plot(ans.v_vr_PSG_First.Time, ans.v_vr_PSG_First.Data(:,1) - ans.v_vr_PSG_First.Data(:,2), 'linewidth', lw);
% grid on
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('PSG First Speed Error (m/s)','Interpreter', 'LaTeX','fontsize',ftsz)
% 
% subplot(2,5,4)
% plot(ans.v_vr_PSG_Middle.Time, ans.v_vr_PSG_Middle.Data(:,1), 'linewidth', lw);
% grid on
% hold on
% plot(ans.v_vr_PSG_Middle.Time, ans.v_vr_PSG_Middle.Data(:,2), 'r-.', 'linewidth', lw);
% legend('Actual Speed','Desired Speed')
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('PSG Middle Speed (m/s)','Interpreter', 'LaTeX','fontsize',ftsz)
% 
% subplot(2,5,9)
% plot(ans.v_vr_PSG_Middle.Time, ans.v_vr_PSG_Middle.Data(:,1) - ans.v_vr_PSG_Middle.Data(:,2), 'linewidth', lw);
% grid on
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('PSG Middle Speed Error (m/s)','Interpreter', 'LaTeX','fontsize',ftsz)
% 
% subplot(2,5,5)
% plot(ans.v_vr_PSG_End.Time, ans.v_vr_PSG_End.Data(:,1), 'linewidth', lw);
% grid on
% hold on
% plot(ans.v_vr_PSG_End.Time, ans.v_vr_PSG_End.Data(:,2), 'r-.', 'linewidth', lw);
% legend('Actual Speed','Desired Speed')
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('PSG End Speed (m/s)','Interpreter', 'LaTeX','fontsize',ftsz)
% 
% subplot(2,5,10)
% plot(ans.v_vr_PSG_End.Time, ans.v_vr_PSG_End.Data(:,1) - ans.v_vr_PSG_End.Data(:,2), 'linewidth', lw);
% grid on
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('PSG End Speed Error (m/s)','Interpreter', 'LaTeX','fontsize',ftsz)
% 
% set(gcf,'color','w');

%% Reference intervehicle distance tracking
figure
set(gcf,'paperpositionMode','auto','position',pos_fig)
subplot(2,4,1)
plot(ans.deltad_HDV_Last.Time, ans.deltad_HDV_Last.Data(:,2), 'linewidth', lw);
grid on
hold on
plot(ans.deltad_HDV_Last.Time, ans.deltad_HDV_Last.Data(:,1), 'r-.','linewidth', lw);
legend('Real Intervehicle Distance','Desired Intervehicle Distance')
xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
ylabel('Dist HDV Last to HDV First(m)','Interpreter', 'LaTeX','fontsize',ftsz)
title('Case1')

subplot(2,4,5)
plot(ans.deltad_HDV_Last.Time, ans.deltad_HDV_Last.Data(:,2) - ans.deltad_HDV_Last.Data(:,1), 'linewidth', lw);
grid on
xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
ylabel('HDV Last Dist Error(m)','Interpreter', 'LaTeX','fontsize',ftsz)

subplot(2,4,2)
plot(ans.deltad_PSG_First.Time, ans.deltad_PSG_First.Data(:,2), 'linewidth', lw);
grid on
hold on
plot(ans.deltad_PSG_First.Time, ans.deltad_PSG_First.Data(:,1), 'r-.','linewidth', lw);
legend('Real Intervehicle Distance','Desired Intervehicle Distance')
xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
ylabel('Dist PSG First to HDV Last(m)','Interpreter', 'LaTeX','fontsize',ftsz)

subplot(2,4,6)
plot(ans.deltad_PSG_First.Time, ans.deltad_PSG_First.Data(:,2) - ans.deltad_PSG_First.Data(:,1), 'linewidth', lw);
grid on
xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
ylabel('PSG First Dist Error(m)','Interpreter', 'LaTeX','fontsize',ftsz)

subplot(2,4,3)
plot(ans.deltad_PSG_Middle.Time, ans.deltad_PSG_Middle.Data(:,2), 'linewidth', lw);
grid on
hold on
plot(ans.deltad_PSG_Middle.Time, ans.deltad_PSG_Middle.Data(:,1), 'r-.','linewidth', lw);
legend('Real Intervehicle Distance','Desired Intervehicle Distance')
xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
ylabel('Dist PSG Middle to PSG First(m)','Interpreter', 'LaTeX','fontsize',ftsz)

subplot(2,4,7)
plot(ans.deltad_PSG_Middle.Time, ans.deltad_PSG_Middle.Data(:,2) - ans.deltad_PSG_Middle.Data(:,1), 'linewidth', lw);
grid on
xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
ylabel('PSG Middle Dist Error(m)','Interpreter', 'LaTeX','fontsize',ftsz)

subplot(2,4,4)
plot(ans.deltad_PSG_End.Time, ans.deltad_PSG_End.Data(:,2), 'linewidth', lw);
grid on
hold on
plot(ans.deltad_PSG_End.Time, ans.deltad_PSG_End.Data(:,1), 'r-.','linewidth', lw);
legend('Real Intervehicle Distance','Desired Intervehicle Distance')
xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
ylabel('Dist PSG End to PSG Middle(m)','Interpreter', 'LaTeX','fontsize',ftsz)

subplot(2,4,8)
plot(ans.deltad_PSG_End.Time, ans.deltad_PSG_End.Data(:,2) - ans.deltad_PSG_End.Data(:,1), 'linewidth', lw);
grid on
xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
ylabel('PSG End Dist Error(m)','Interpreter', 'LaTeX','fontsize',ftsz)
set(gcf,'color','w');

 %% Absolute Dist & Adaptive Rho_d
% figure
% set(gcf,'paperpositionMode','auto','position',pos_fig)
% subplot(2,2,1)
% yyaxis left
% plot(ans.Absd_HDV_Last.Time, ans.Absd_HDV_Last.Data, 'linewidth', lw);
% grid on
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('AbsDist HDV Last to HDV First (m)', 'Interpreter', 'LaTeX', 'fontsize', ftsz)
% yyaxis right
% plot(ans.Rhod_HDV_Last.Time, ans.Rhod_HDV_Last.Data, 'linewidth', lw);
% grid on
% ylabel('$$\rho_d$$ HDV Last','Interpreter', 'LaTeX','fontsize',ftsz)
% subplot(2,2,2)
% yyaxis left
% plot(ans.Absd_PSG_First.Time, ans.Absd_PSG_First.Data, 'linewidth', lw);
% grid on
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('AbsDist PSG First to HDV Last (m)', 'Interpreter', 'LaTeX', 'fontsize', ftsz)
% yyaxis right
% plot(ans.Rhod_PSG_First.Time, ans.Rhod_PSG_First.Data, 'linewidth', lw);
% grid on
% ylabel('$$\rho_d$$ PSG First', 'Interpreter', 'LaTeX','fontsize',ftsz)
% subplot(2,2,3)
% yyaxis left
% plot(ans.Absd_PSG_Middle.Time, ans.Absd_PSG_Middle.Data, 'linewidth', lw);
% grid on
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('AbsDist PSG Middle to PSG First (m)', 'Interpreter', 'LaTeX', 'fontsize', ftsz)
% yyaxis right
% plot(ans.Rhod_PSG_Middle.Time, ans.Rhod_PSG_Middle.Data, 'linewidth', lw);
% grid on
% ylabel('$$\rho_d$$ PSG Middle', 'Interpreter', 'LaTeX','fontsize',ftsz)
% subplot(2,2,4)
% yyaxis left
% plot(ans.Absd_PSG_End.Time, ans.Absd_PSG_End.Data, 'linewidth', lw);
% grid on
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('AbsDist PSG End to PSG Middle (m)', 'Interpreter', 'LaTeX', 'fontsize', ftsz)
% yyaxis right
% plot(ans.Rhod_PSG_End.Time, ans.Rhod_PSG_End.Data, 'linewidth', lw);
% grid on
% ylabel('$$\rho_d$$ PSG End', 'Interpreter', 'LaTeX','fontsize',ftsz)
% set(gcf,'color','w');

%% Distance to end
figure
set(gcf,'paperpositionMode','auto','position',pos_fig)
plot(ans.to_end_Summary.Time, ans.to_end_Summary.Data(:,4)-194, 'b', 'linewidth', lw);   % HDV First
grid on;
hold on;
plot(ans.to_end_Summary.Time, ans.to_end_Summary.Data(:,5)-194, 'b-.', 'linewidth', lw);  % HDV End
plot(ans.to_end_Summary.Time, ans.to_end_Summary.Data(:,1)-194, 'r',  'linewidth', lw);  % PSG First
plot(ans.to_end_Summary.Time, ans.to_end_Summary.Data(:,2)-194, 'r-.',  'linewidth', lw);  % PSG Middle
plot(ans.to_end_Summary.Time, ans.to_end_Summary.Data(:,3)-194, 'r--',  'linewidth', lw);  % PSG End
legend('HDV First','HDV End', 'PSG First', 'PSG Middle', 'PSG End')
xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
ylabel('Distance to MP (m)','Interpreter', 'LaTeX','fontsize',ftsz)
title('Case 1')
xlim([1.18, 28])
set(gcf,'color','w');

%% All CO2 and Fuel
figure
set(gcf,'paperpositionMode','auto','position',pos_fig)
subplot(1,2,1)
plot(ans.All_CO2.Time, ans.All_CO2.Data, 'b',  'linewidth', lw);  % HDV First
xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
ylabel('All CO2 (mg)','Interpreter', 'LaTeX','fontsize',ftsz)
title('Case1')
%xlim([15, 34])
subplot(1,2,2)
plot(ans.All_Fuel.Time, ans.All_Fuel.Data, 'b',  'linewidth', lw);  % HDV First
xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
ylabel('All Fuel (ml)','Interpreter', 'LaTeX','fontsize',ftsz)
%xlim([15, 34])
set(gcf,'color','w');

% %% Acceleration
% figure
% set(gcf,'paperpositionMode','auto','position',pos_fig)
% subplot(1,2,1)
% plot(ans.acc_Summary.Time, ans.acc_Summary.Data(:,4), 'b',  'linewidth', lw);  % HDV First
% grid on;
% hold on;
% plot(ans.acc_Summary.Time, ans.acc_Summary.Data(:,5), 'b-.',  'linewidth', lw);  % HDV End
% plot(ans.acc_Summary.Time, ones(length(ans.acc_Summary.Time), 1)*HDV_AMAX, 'k', 'linewidth', lw-0.5);
% plot(ans.acc_Summary.Time, ones(length(ans.acc_Summary.Time), 1)*HDV_AMIN, 'k', 'linewidth', lw-0.5);
% legend('HDV First','HDV End', 'Upper  Bound', 'Lower Bound')
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('HDV acceleration (m/s2)','Interpreter', 'LaTeX','fontsize',ftsz)
% %xlim([15, 34])
% 
% subplot(1,2,2)
% plot(ans.acc_Summary.Time, ans.acc_Summary.Data(:,1), 'r',    'linewidth', lw);  % PSG First
% grid on;
% hold on;
% plot(ans.acc_Summary.Time, ans.acc_Summary.Data(:,2), 'r-.',  'linewidth', lw);  % PSG Middle
% plot(ans.acc_Summary.Time, ans.acc_Summary.Data(:,3), 'r--',  'linewidth', lw);  % PSG End
% plot(ans.acc_Summary.Time, ones(length(ans.acc_Summary.Time), 1)*PSG_AMAX, 'k', 'linewidth', lw-0.5);
% plot(ans.acc_Summary.Time, ones(length(ans.acc_Summary.Time), 1)*PSG_AMIN, 'k', 'linewidth', lw-0.5);
% legend('PSG First', 'PSG Middle', 'PSG End', 'Upper  Bound', 'Lower Bound')
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('Acceleration (m/s2)','Interpreter', 'LaTeX','fontsize',ftsz)
% %xlim([15, 34])
% set(gcf,'color','w');

%% ADE
% figure
% set(gcf,'paperpositionMode','auto','position',pos_fig)
% subplot(1,2,1)
% plot(ans.DirectDiff_Summary.Time, ans.DirectDiff_Summary.Data(:,1), 'linewidth', lw);  % PSG First
% grid on;
% hold on;
% plot(ans.ADE_Summary.Time, ans.ADE_Summary.Data(:,1), 'linewidth', lw);                % PSG First
% legend('Estimated','Direct Diff')
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('PSG First: Derivative of speed (m/s2)','Interpreter', 'LaTeX','fontsize',ftsz)
% 
% subplot(1,2,2)
% plot(ans.DirectDiff_Summary.Time, ans.DirectDiff_Summary.Data(:,4), 'linewidth', lw);  % HDV First
% grid on;
% hold on;
% plot(ans.ADE_Summary.Time, ans.ADE_Summary.Data(:,4), 'linewidth', lw);                % HDV First
% legend('Direct Diff', 'Estimated')
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('HDV First: Derivative of speed (m/s2)','Interpreter', 'LaTeX','fontsize',ftsz)
% set(gcf,'color','w');
% 
% %% TRFC Conditions
% figure
% set(gcf,'paperpositionMode','auto','position',pos_fig)
% subplot(1,5,3)
% plot(ans.PSG_First_HighMu.Time, ans.PSG_First_HighMu.Data, 'linewidth', lw);
% grid on
% hold on
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('PSG First Road Condition','Interpreter', 'LaTeX','fontsize',ftsz)
% 
% subplot(1,5,4)
% plot(ans.PSG_Middle_HighMu.Time, ans.PSG_Middle_HighMu.Data, 'linewidth', lw);
% grid on
% hold on
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('PSG Middle Road Condition','Interpreter', 'LaTeX','fontsize',ftsz)
% 
% subplot(1,5,5)
% plot(ans.PSG_End_HighMu.Time, ans.PSG_End_HighMu.Data, 'linewidth', lw);
% grid on
% hold on
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('PSG End Road Condition','Interpreter', 'LaTeX','fontsize',ftsz)
% 
% subplot(1,5,1)
% plot(ans.HDV_First_HighMu.Time, ans.HDV_First_HighMu.Data, 'linewidth', lw);
% grid on
% hold on
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('HDV First Road Condition','Interpreter', 'LaTeX','fontsize',ftsz)
% 
% subplot(1,5,2)
% plot(ans.HDV_Last_HighMu.Time, ans.HDV_Last_HighMu.Data, 'linewidth', lw);
% grid on
% hold on
% xlabel('Time (s)','Interpreter', 'LaTeX','fontsize',ftsz)
% ylabel('HDV End Road Condition','Interpreter', 'LaTeX','fontsize',ftsz)
% set(gcf,'color','w');

