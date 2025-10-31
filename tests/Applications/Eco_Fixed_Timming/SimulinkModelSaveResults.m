% File: realsim_script.m

settingDir = 'C:\Users\hg25079\Documents\GitHub\FIXS\tests\Applications\Eco_Fixed_Timming\Experiments_Sumo\Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V3\MPR\0%_10Hz_w_vehDyn_29119.31\';

VehicleOut = out;
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
ActualAcc = VehicleOut.VehAcc.Data;


save('vehData', 'VehicleOut')
T = table(Time, DesiredSpeed, ActualSpeed, MPGe, BatteryCurrent, BatteryPower, BatterySOC, MotorSpeed, MotorTorque, ActualAcc,'VariableNames', {'Time', 'DesiredSpeed', 'ActualSpeed', 'MPGe', 'BatteryCurrent', 'BatteryPower','BatterySOC', 'MotorSpeed', 'MotorTorque', 'ActualAcc'});
% T = table(Time, DesiredSpeed, ActualSpeed, MPGe, BatteryCurrent, BatteryPower, BatterySOC, MotorSpeed, MotorTorque,'VariableNames', {'Time', 'DesiredSpeed', 'ActualSpeed', 'MPGe', 'BatteryCurrent', 'BatteryPower','BatterySOC', 'MotorSpeed', 'MotorTorque'});

writetable(T, [settingDir, '\ego_profile.csv']);

