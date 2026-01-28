function run_simulink_job(modelPath, inputDir, ipgDir, outputDir)
% Launching sumo -> traffic layer -> load simulink -> run with IPG.
% modelPath: full path to .slx or .mdl
% inputDir:  folder with sumo cfg files
% outputDir: where to save results

    arguments
        modelPath (1,1) string
        inputDir  (1,1) string
        ipgDir    (1,1) string
        outputDir (1,1) string
    end

    if ~isfolder(outputDir)
        mkdir(outputDir);
    end

    % SUMO processes
    cd('C:\CM_Projects\Super_RS\tests\I81Small');
    %cfgFile = [direc, '\', inputDir];
    cmdSumo = ['start "" sumo-gui -c "', char(inputDir), ...
               '" --remote-port 1337 --time-to-teleport -1 --time-to-teleport.remove false --max-depart-delay -1 --step-length 0.1 --start'];
    system(cmdSumo);
    exePath = fullfile(fullfile('..', '..'), 'TrafficLayer.exe');
    configFilename = 'C:\CM_Projects\Super_RS\tests\I81Small\RS_config_release.yaml';
    cmdTL   = sprintf('start "" cmd /c "%s" -f %s', exePath, configFilename);
    system(cmdTL);
    %system('start cmd.exe /c runI81.bat');
    pause(3);

    % Load model, open IPG GUI, and load IPG testrun
    cd('C:\CM_Projects\Super_RS\CM11_proj\src_tm4sl')
    %mdl_name = 'generic_truck_RS_Control_CompensateRS.mdl';
    open_system(modelPath);
    TM_Simulink;
    cmguicmd(['LoadTestRun ', char(ipgDir)]);
    pause(5);
    
    try
        simOut = sim(modelPath);
        disp('Simulation finished successfully.');
    catch ME
        disp('Simulation error occurred:');
        disp(ME.message);
    end

    % Kill the process
    PIDs = [];
    [~, cmdout] = system('tasklist /fi "imagename eq cmd.exe"');
    lines = strsplit(cmdout, '\n');
    for i = 1:numel(lines)
        if contains(lines{i}, 'cmd.exe')
            tokens = strsplit(strtrim(lines{i}));
            pid = str2double(tokens{2});
            PIDs = [PIDs, pid];
        end
    end
    [status, cmdout] = system('tasklist /fi "imagename eq WindowsTerminal.exe"');
    lines = strsplit(cmdout, '\n');
    for i = 1:numel(lines)
        if contains(lines{i}, 'WindowsTerminal.exe')
            tokens = strsplit(strtrim(lines{i}));
            pid = str2double(tokens{2});
            PIDs = [PIDs, pid];
        end
    end
    
    for pid=PIDs
        system(['taskkill /F /PID ', num2str(pid)]);
    end
   
    % Provide variables to the model (adjust to your needs)
    assignin('base', 'INPUT_DIR',  char(inputDir));
    assignin('base', 'OUTPUT_DIR', char(outputDir));
    assignin('base', 'SIM_OUT', simOut);

    % Save something explicit
    %simOut = [];
    save(fullfile(outputDir, 'simout.mat'), 'simOut');

    % Clean up
    %bdclose(modelPath);
end
