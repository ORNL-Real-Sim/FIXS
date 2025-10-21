
% This is a script for Merging Control of ACM
%clc
%clear all
%close all
% set(0,'DefaultAxesFontName', 'Times');
% set(0,'DefaultAxesFontSize', 12);
% set(0,'DefaultTextFontname', 'Times');
% set(0,'DefaultTextFontSize', 12);
load('speedProfile_Jinghui.mat');
load('speedProfile_Jinghui_Ref.mat');
ref_time = time;
ref_speed = speed;
%% Vehicle Dynamics
global UT
UT.Ts_Veh = 0.01;    % Basic Simulink period = Veh Dyn Simulation period = SUMO period = MFC;
                      
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
PSG_HighMu_P    = 105;     PSG_LowMu_P    = 48;
PSG_HighMu_I    = 150;    PSG_LowMu_I    = 64;
PSG_HighMu_D    = 20;     PSG_LowMu_D    = 12;  
PSG_HighMu_TRFC = 1;    PSG_LowMu_TRFC = 0.7;

High_TRFC = 1;   % Will be a distance-to-MP function: have TRFC drop

T_ADE = 0.01;
v_ini = 16+3;

v_cruise = 16+3; %m/s
% Load Leader Speed 
load('Front_car_speed.mat')

% Desired Int-Veh Dist
time_headway  = 1;
safety_margin = 2.5; 

Initial_dist = v_ini*time_headway+safety_margin;

%% Sim
% sim('MFC_ACC')
% 
% %% Plot
% figure, plot(ans.Int_dist_tracking.time, ans.Int_dist_tracking.signals.values(:,2))
% hold on
% plot(ans.Int_dist_tracking.time, ans.Int_dist_tracking.signals.values(:,1), 'r--')
% legend('Ref Distance','Real Distance');
% xlabel('Time (s)')
% ylabel('ACC Int Veh Dist')

