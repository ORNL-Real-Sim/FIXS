%% =========================
%  prepare_measured_inputs.m
%  作用：
%  1) 读取 vehicle_data_1s.csv
%  2) 整理成 Simulink 可用的 timeseries
%  3) 放到 base workspace


clear; clc;

%% 1. CSV 文件完整路径
csvFile = 'C:\Users\yusun\Desktop\Yu Sun\MSC\Mache_measurement data\Measurement Data\0%_10Hz_w_vehDyn_29119.31_eco\vehicle_data_1.csv';

%% 2. 读取表格
data = readtable(csvFile);

%% 3. 看一下列名（可选）
disp('CSV column names:')
disp(data.Properties.VariableNames')

%% 4. 提取时间和信号
% 你这个表第一列是 time
t = data.time;
spd_mph = data.Vehicle_SpdDmd_mph;

% ===== 下面两行按你表里的真实列名写 =====
% 如果你的列名正好是下面这样，就不用改
gas_raw   = data.AccelPedal_CmdFinal_pct;
brake_raw = data.BrakePedalCmd_pct;

rpmLF = data.ActSpLF_rpm;
rpmRF = data.ActSpRF_rpm;
rpmLR = data.ActSpLR_rpm;
rpmRR = data.ActSpRR_rpm;

% 转向先设为 0
steer_raw = zeros(size(t));

%% 5. 检查时间是否从 0 开始（可选）
if t(1) ~= 0
    t = t - t(1);
end

%% 6. 如果气门/刹车是百分比 0~100，就先保留原始值
% 因为你模型里前面还有 0.01 增益
% 所以这里不要再除以 100
gas   = gas_raw;
brake = brake_raw;
steer = steer_raw;

%% 7. 生成为 Simulink 准备的 timeseries
gas_ts   = timeseries(gas, t);
brake_ts = timeseries(brake, t);
steer_ts = timeseries(steer, t);
spd_kmh_ts = timeseries(spd_mph * 1.609, t);

rpmLF_ts = timeseries(rpmLF,t);
rpmRF_ts = timeseries(rpmRF,t);
rpmLR_ts = timeseries(rpmLR,t);
rpmRR_ts = timeseries(rpmRR,t);

%% 8. 额外也放普通变量进去（可选）
% assignin('base', 'data_meas', data);
% assignin('base', 't_meas', t);
% assignin('base', 'gas_meas', gas);
% assignin('base', 'brake_meas', brake);
% assignin('base', 'steer_meas', steer);
% 
% assignin('base', 'gas_ts', gas_ts);
% assignin('base', 'brake_ts', brake_ts);
% assignin('base', 'steer_ts', steer_ts);
% 
% %% 9. 打印确认
% disp('Done. Variables added to base workspace:')
% disp('  data_meas')
% disp('  t_meas')
% disp('  gas_meas')
% disp('  brake_meas')
% disp('  steer_meas')
% disp('  gas_ts')
% disp('  brake_ts')
% disp('  steer_ts')

%% 10. 推荐仿真终止时间
% simStopTime = num2str(t(end));
% assignin('base', 'simStopTime_meas', simStopTime);
% 
% disp(['Recommended Stop Time = ', simStopTime])