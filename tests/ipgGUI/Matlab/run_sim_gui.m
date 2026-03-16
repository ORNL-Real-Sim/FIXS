function sim_runner_ui
% Simple MATLAB UI to pick a Simulink .slx, input/output folders, and run.
% No dependencies beyond MATLAB/Simulink.

    %------------- State -------------
    S.modelPath = "";
    S.inputDir  = "";
    s.ipgDir    = "";
    S.outputDir = "";

    %------------- UI -------------
    f = uifigure('Name','Real-Sim GUI v0.0','Position', [100 100 720 490]);

    % Labels + fields
    uilabel(f,'Position',[20 430+20 160 22],'Text','Simulink model (.slx or .mdl)');
    eModel = uieditfield(f,'text','Position',[20 405+20 550 26],'Editable','off');

    uilabel(f,'Position',[20 370+20 160 22],'Text','IPG Testrun file (.txt)');
    eTest = uieditfield(f,'text','Position',[20 345+20 550 26],'Editable','off');

    uilabel(f,'Position',[20 305+20 240 22],'Text','Input SUMO configuration file (.sumocfg)');
    eIn = uieditfield(f,'text','Position',[20 280+20 560 26],'Editable','off');

    uilabel(f,'Position',[20 240+20 160 22],'Text','Output folder');
    eOut = uieditfield(f,'text','Position',[20 215+20 560 26],'Editable','off');

    % Browse buttons
    bModel = uibutton(f,'Text','Browse simulink model','Position',[575 405+20 135 26],...
        'ButtonPushedFcn', @(~,~) onPickModel());
    bTest = uibutton(f,'Text','Browse IPG Testrun','Position',[575 345+20 135 26],...
        'ButtonPushedFcn', @(~,~) onPickTestRun());
    bIn    = uibutton(f,'Text','Browse...','Position',[590 280+20 110 26],...
        'ButtonPushedFcn', @(~,~) onPickFileIn());
    bOut   = uibutton(f,'Text','Browse...','Position',[590 215+20 110 26],...
        'ButtonPushedFcn', @(~,~) onPickOut());

    % Action buttons
    bValidate = uibutton(f,'Text','Validate','Position',[20 175+20 100 28],...
        'ButtonPushedFcn', @(~,~) onValidate());
    bRun      = uibutton(f,'Text','Run','Position',[130 175+20 100 28],...
        'ButtonPushedFcn', @(~,~) onRun());
    bQuit     = uibutton(f,'Text','Quit','Position',[240 175+20 100 28],...
        'ButtonPushedFcn', @(~,~) close(f));

    % Log area
    tx = uitextarea(f,'Position',[20 20 680 150],'Editable','off');

    %------------- Callbacks -------------
    function onPickModel()
        [file, path] = uigetfile({'*.slx'; '*.mdl'}, 'Select Simulink model');
        if isequal(file,0), return; end
        S.modelPath = string(fullfile(path,file));
        eModel.Value = S.modelPath;
        log("Selected model: " + S.modelPath);
    end

%     function onPickIn()
%         d = uigetdir(pwd,'Select input folder');
%         if isequal(d,0), return; end
%         S.inputDir = string(d);
%         eIn.Value = S.inputDir;
%         log("Selected input: " + S.inputDir);
%     end

    function onPickFileIn()
        [file, path] = uigetfile('*.sumocfg', 'Select sumo config');
        if isequal(file,0), return; end
        S.inputDir = string(fullfile(path,file));
        %strlength(S.inputDir)
        %isfolder(S.inputDir)
        eIn.Value = S.inputDir;
        log("Selected input: " + S.inputDir);
    end
    
    function onPickTestRun()
        [file, path] = uigetfile('*.*', 'Select IPG testrun');
        if isequal(file,0), return; end
        S.ipgDir = string(fullfile(path,file));
        %strlength(S.inputDir)
        %isfolder(S.inputDir)
        eTest.Value = S.ipgDir;
        log("Selected testrun: " + S.ipgDir);
    end

    function onPickOut()
        d = uigetdir(pwd,'Select output folder');
        if isequal(d,0), return; end
        S.outputDir = string(d);
        eOut.Value = S.outputDir;
        log("Selected output: " + S.outputDir);
    end

    function onValidate()
        errs = validate(S);
        if isempty(errs)
            log("✅ Paths look good.");
        else
            for i = 1:numel(errs)
                log("❌ " + errs{i});
            end
        end
    end

    function onRun()
        errs = validate(S);
        if ~isempty(errs)
            for i = 1:numel(errs)
                log("❌ " + errs{i});
            end
            return;
        end

        set([bRun bValidate bModel bTest bIn bOut bQuit],'Enable','off');
        cleanup = onCleanup(@() set([bRun bValidate bModel bTest bIn bOut bQuit],'Enable','on'));

        try
            log("Starting simulation…");
            drawnow;

            % Call your runner
            run_simulink_job(S.modelPath, S.inputDir, s.ipgDir, S.outputDir);

            log("✅ Done. \n Results saved to: " + S.outputDir);
        catch ME
            log("❌ Error: \n" + string(ME.message));
            if ~isempty(ME.stack)
                log("   at " + string(ME.stack(1).name) + ":" + string(ME.stack(1).line));
            end
        end
    end

    %------------- Helpers -------------
    function errs = validate(state)
        errs = {};
        if strlength(state.modelPath)==0 || ~isfile(state.modelPath)
            errs{end+1} = "Model path does not exist.";
        elseif ~endsWith(lower(state.modelPath), [".slx", ".mdl"])
            errs{end+1} = "Model must be a .slx or .mdl file.";
        end

        if strlength(state.inputDir)==0 %|| ~isfolder(state.inputDir)
            errs{end+1} = "Input folder is invalid.";
        end

        if strlength(state.outputDir)==0
            errs{end+1} = "Output folder is empty.";
        else
            if ~isfolder(state.outputDir)
                % Try to create it
                try
                    mkdir(state.outputDir);
                catch e
                    errs{end+1} = "Cannot create output folder: " + e.message;
                end
            end
        end
    end

    function log(msg)
        tx.Value = [tx.Value; string(msg)];
        drawnow;
    end
end
