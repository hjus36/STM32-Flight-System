function summary = analyze_roll_pid_models()
% ANALYZE_ROLL_PID_MODELS
% STM32 Flight System: compare P, PI, 1DOF PID and 2DOF PID models.
% Tested target: MATLAB / Simulink R2026b (run locally by the user).
%
% Usage:
%   1. Save the four .slx files and put this .m file in the same folder.
%   2. In MATLAB, navigate to this folder and run:
%        summary = analyze_roll_pid_models;
%
% The script creates TEMPORARY copies of the .slx files for signal logging.
% The source .slx files are NOT saved over or modified on disk.
% This script requires Simulink and Control System Toolbox (stepinfo).

rootDir = fullfile(fileparts(mfilename('fullpath')), '..', 'simulink');
if isempty(rootDir)
    error('Save analyze_roll_pid_models.m to the model folder before running.');
end
cd(rootDir);

modelFiles = { ...
    'Roll_Single_P.slx', ...
    'Roll_Single_PI.slx', ...
    'Roll_Single_PID_Full.slx', ...
    'Roll_Single_PID_2DOF.slx'};
labels = {'P', 'PI', 'PID (1DOF)', 'PID (2DOF)'};
shortNames = {'P', 'PI', 'PID_1DOF', 'PID_2DOF'};

outDir = fullfile(rootDir, '..', '..', '.local', 'matlab-analysis');
if ~isfolder(outDir), mkdir(outDir); end
workDir = fullfile(outDir, '_temporary_models');
if ~isfolder(workDir), mkdir(workDir); end

for i = 1:numel(modelFiles)
    if ~isfile(fullfile(rootDir, modelFiles{i}))
        error('Required model not found: %s', fullfile(rootDir, modelFiles{i}));
    end
end

% Use exactly the same time grid for every model.
Ts = 0.01;
tStep = 1.0;
stopTime = 10.0;
t = (0:Ts:stopTime)';
n = numel(modelFiles);
allY = nan(numel(t), n);
allU = nan(numel(t), n);
allRef = nan(numel(t), n);

rise_s = nan(n,1);
settle_s = nan(n,1);
end_deg = nan(n,1);
endErr_deg = nan(n,1);
overshoot_pct = nan(n,1);
maxU = nan(n,1);
minU = nan(n,1);
maxAbsU = nan(n,1);
initialPeakU = nan(n,1);
IAE_deg_s = nan(n,1);
controllerP = strings(n,1);
controllerI = strings(n,1);
controllerD = strings(n,1);
controllerN = strings(n,1);
controllerB = strings(n,1);
controllerC = strings(n,1);
plantNumerator = strings(n,1);
plantDenominator = strings(n,1);

fprintf('Analyzing %d saved Simulink models...\n', n);
for i = 1:n
    % Copy source model to an isolated, disposable simulation model.
    tmpName = ['RollAnalysis_' shortNames{i}];
    tmpFile = fullfile(workDir, [tmpName '.slx']);
    if bdIsLoaded(tmpName), close_system(tmpName, 0); end
    copyfile(fullfile(rootDir, modelFiles{i}), tmpFile, 'f');

    try
        load_system(tmpFile);
        modelBlocks = find_system(tmpName, 'SearchDepth', 1, 'Type', 'Block');
        stepBlock = '';
        plantBlock = '';
        for j = 1:numel(modelBlocks)
            b = modelBlocks{j};
            blockType = get_param(b, 'BlockType');
            blockName = get_param(b, 'Name');
            if strcmp(blockType, 'Step')
                stepBlock = b;
            end
            if strcmp(blockType, 'DiscreteTransferFcn') || ...
                    strcmp(blockName, 'Discrete Transfer Fcn')
                plantBlock = b;
            end
        end
        if isempty(stepBlock) || isempty(plantBlock)
            error(['Could not locate Step and Discrete Transfer Fcn blocks. ' ...
                'Keep the original top-level block types unchanged.']);
        end

        % The controller is the source block feeding the plant input.
        plantPorts = get_param(plantBlock, 'PortHandles');
        plantInputLine = get_param(plantPorts.Inport(1), 'Line');
        pidOutPort = get_param(plantInputLine, 'SrcPortHandle');
        pidBlock = get_param(pidOutPort, 'Parent');
        stepPorts = get_param(stepBlock, 'PortHandles');

        % Log source/reference, plant output and controller output.
        enableLog(stepPorts.Outport(1), 'target_roll');
        enableLog(plantPorts.Outport(1), 'actual_roll');
        enableLog(pidOutPort, 'control_output');

        set_param(tmpName, 'SignalLogging', 'on', ...
            'SignalLoggingName', 'logsout', ...
            'ReturnWorkspaceOutputs', 'on', ...
            'StartTime', '0', 'StopTime', num2str(stopTime));
        fprintf('  [%d/%d] %s ... ', i, n, labels{i});
        simOut = sim(tmpName, 'ReturnWorkspaceOutputs', 'on');
        ds = simOut.logsout;

        refTs = ds.getElement('target_roll').Values;
        actualTs = ds.getElement('actual_roll').Values;
        controlTs = ds.getElement('control_output').Values;

        % All models use a 10-ms sample time. Align for comparison.
        r = sampleAt(refTs, t);
        y = sampleAt(actualTs, t);
        u = sampleAt(controlTs, t);
        allRef(:,i) = r;
        allY(:,i) = y;
        allU(:,i) = u;

        afterStep = t >= tStep - (Ts * 0.1);
        early = t >= tStep - (Ts * 0.1) & t <= tStep + 0.20 + (Ts * 0.1);
        referenceDeg = r * 180/pi;
        actualDeg = y * 180/pi;
        targetDeg = referenceDeg(end);
        end_deg(i) = actualDeg(end);
        endErr_deg(i) = targetDeg - actualDeg(end);
        maxU(i) = max(u(afterStep));
        minU(i) = min(u(afterStep));
        maxAbsU(i) = max(abs(u(afterStep)));
        initialPeakU(i) = max(abs(u(early)));
        overshoot_pct(i) = max(0, 100 * (max(actualDeg(afterStep)) - targetDeg) / abs(targetDeg));
        IAE_deg_s(i) = trapz(t(afterStep), abs(referenceDeg(afterStep) - actualDeg(afterStep)));

        % Measure times relative to the commanded 5-degree target, NOT to
        % each response's last value (important for the P-only controller).
        postT = t(afterStep) - tStep;
        postY = actualDeg(afterStep);
        responseInfo = stepinfo(postY, postT, targetDeg, 0, ...
            'SettlingTimeThreshold', 0.02, 'RiseTimeLimits', [0.1 0.9]);
        rise_s(i) = responseInfo.RiseTime;
        settle_s(i) = responseInfo.SettlingTime;

        plantNumerator(i) = getParamSafe(plantBlock, 'Numerator');
        plantDenominator(i) = getParamSafe(plantBlock, 'Denominator');
        controllerP(i) = getParamSafe(pidBlock, 'P');
        controllerI(i) = getParamSafe(pidBlock, 'I');
        controllerD(i) = getParamSafe(pidBlock, 'D');
        controllerN(i) = getParamSafe(pidBlock, 'N');
        controllerB(i) = getParamSafe(pidBlock, 'b');
        controllerC(i) = getParamSafe(pidBlock, 'c');

        perModel = table(t, referenceDeg, actualDeg, u, ...
            'VariableNames', {'time_s','target_deg','actual_deg','control_output'});
        writetable(perModel, fullfile(outDir, [shortNames{i} '_signals.csv']));
        fprintf('OK\n');
    catch ME
        if bdIsLoaded(tmpName), close_system(tmpName, 0); end
        error('Analysis of %s failed:\n%s', modelFiles{i}, getReport(ME, 'extended'));
    end
    % Do not save the temporary models, and never modify source files.
    if bdIsLoaded(tmpName), close_system(tmpName, 0); end
    delete(tmpFile);
end

if numel(unique(plantNumerator)) ~= 1 || numel(unique(plantDenominator)) ~= 1
    warning('Plant numerator/denominator settings differ between models. Check the comparison.');
end
if any(any(abs(allRef - allRef(:,1)) > 1e-10))
    warning('Reference signals differ between models. Check Step parameters.');
end

summary = table(string(labels(:)), controllerP, controllerI, controllerD, ...
    controllerN, controllerB, controllerC, rise_s, settle_s, ...
    end_deg, endErr_deg, overshoot_pct, maxU, minU, maxAbsU, ...
    initialPeakU, IAE_deg_s, ...
    'VariableNames', {'Controller','Kp','Ki','Kd','N','b','c', ...
    'RiseTime_s','SettlingTimeToTarget_s','RollAt10s_deg', ...
    'ErrorAt10s_deg','Overshoot_pct','MaxControl','MinControl', ...
    'MaxAbsControl','PeakAbsControlFirst0p2s','IAE_deg_s'});
writetable(summary, fullfile(outDir, 'roll_pid_comparison.csv'));

% Charts (all numerical results come from the four real simulations).
fig = figure('Visible','off','Color','w','Position',[100 100 1050 600]);
plot(t, allRef(:,1)*180/pi, 'k--', 'LineWidth', 1.7); hold on;
for i = 1:n
    plot(t, allY(:,i)*180/pi, 'LineWidth', 1.4);
end
hold off; grid on; xlabel('Time (s)'); ylabel('Roll angle (deg)');
title('Roll step response: P / PI / PID / 2DOF PID');
legend([{'Target'}, labels], 'Location','southeast');
exportgraphics(fig, fullfile(outDir, '01_step_response_comparison.png'), 'Resolution', 200);
close(fig);

fig = figure('Visible','off','Color','w','Position',[100 100 1050 600]);
hold on;
for i = 1:n
    plot(t, allU(:,i), 'LineWidth', 1.4);
end
hold off; grid on; xlabel('Time (s)'); ylabel('Controller output (arbitrary units)');
title('Control output comparison'); legend(labels, 'Location','northeast');
exportgraphics(fig, fullfile(outDir, '02_control_output_comparison.png'), 'Resolution', 200);
close(fig);

fig = figure('Visible','off','Color','w','Position',[100 100 1050 600]);
hold on;
for i = 1:n
    plot(t, allU(:,i), 'LineWidth', 1.4);
end
hold off; grid on; xlim([0.95 1.25]);
xlabel('Time (s)'); ylabel('Controller output (arbitrary units)');
title('Initial control response around t=1.0 s');
legend(labels, 'Location','best');
exportgraphics(fig, fullfile(outDir, '03_derivative_kick_zoom.png'), 'Resolution', 200);
close(fig);

% The resulting .mat preserves the numerical simulation arrays as well.
save(fullfile(outDir, 'roll_pid_simulation_data.mat'), ...
    't', 'allRef', 'allY', 'allU', 'labels', 'summary');

peakOld = maxAbsU(3);
peakNew = maxAbsU(4);
reductionPct = 100 * (peakOld - peakNew) / peakOld;
reportFile = fullfile(outDir, 'roll_pid_validation_report.md');
fid = fopen(reportFile, 'w', 'n', 'UTF-8');
if fid < 0, error('Could not create report: %s', reportFile); end
fprintf(fid, '# Roll Single PID - Simulation Validation Report\n\n');
fprintf(fid, '- Generated: %s\n', char(datetime('now','Format','yyyy-MM-dd HH:mm:ss')));
fprintf(fid, '- Environment: MATLAB/Simulink R2026b (local simulation)\n');
fprintf(fid, '- Models: P, PI, 1DOF PID, 2DOF PID\n');
fprintf(fid, '- Plant coefficients (model 1): numerator `%s`, denominator `%s`\n', ...
    plantNumerator(1), plantDenominator(1));
if numel(unique(plantNumerator)) ~= 1 || numel(unique(plantDenominator)) ~= 1
    fprintf(fid, '- WARNING: Plant coefficient settings differ between models. Comparison may not be fair.\n');
end
fprintf(fid, '- Input: 0 -> 5 degrees at t=1 s; Ts=0.01 s; stop=10 s\n');
fprintf(fid, '- Test scope: simplified plant only. This is NOT measured drone hardware performance.\n\n');
fprintf(fid, '## Results\n\n');
fprintf(fid, '| Controller | Roll at 10 s (deg) | Error at 10 s (deg) | Rise to 90%% (s) | Settle to target +/-2%% (s) | Max abs control | IAE (deg s) |\n');
fprintf(fid, '|---|---:|---:|---:|---:|---:|---:|\n');
for i = 1:n
    fprintf(fid, '| %s | %.5f | %.5f | %.4f | %.4f | %.5f | %.5f |\n', ...
        labels{i}, end_deg(i), endErr_deg(i), rise_s(i), settle_s(i), ...
        maxAbsU(i), IAE_deg_s(i));
end
fprintf(fid, '\n- Derivative-kick peak comparison (maximum absolute control after t=1 s): 1DOF PID = %.5f, 2DOF PID = %.5f.\n', peakOld, peakNew);
fprintf(fid, '- Peak reduction = %.2f%%. This compares magnitude only, not flight stability.\n', reductionPct);
fprintf(fid, '- NaN for rise or settling time means the target criterion was not reached within the recorded 10 s window.\n');
fprintf(fid, '- Rise: 10%% to 90%% of commanded target. Settling: remains inside +/-2%% of the commanded 5-degree target.\n');
fprintf(fid, '- Error at 10 s is NOT the theoretical infinite-time steady-state error.\n');
fprintf(fid, '- Controller outputs are abstract model units; they are NOT thrust, torque, or ESC PWM commands.\n\n');
fprintf(fid, '## Original model files (read-only sources)\n\n');
for i = 1:n
    fprintf(fid, '- `%s`\n', modelFiles{i});
end
fprintf(fid, '\n## Saved evidence\n\n');
fprintf(fid, '- `roll_pid_comparison.csv` and four `*_signals.csv`\n');
fprintf(fid, '- `roll_pid_simulation_data.mat`\n');
fprintf(fid, '- `01_step_response_comparison.png`\n');
fprintf(fid, '- `02_control_output_comparison.png`\n');
fprintf(fid, '- `03_derivative_kick_zoom.png`\n\n');
fprintf(fid, '## Remaining validation\n\n');
fprintf(fid, '- No real drone dynamics, inertia, motor thrust, saturation, sensor noise or delays have been validated.\n');
fprintf(fid, '- No STM32 firmware for these controllers has been built or run by this experiment.\n');
fprintf(fid, '- For a fair 1DOF/2DOF comparison, verify identical P/I/D/N, plant and reference values; only the 2DOF setpoint derivative weight c should differ.\n');
fclose(fid);

fprintf('\nComparison complete. Results: %s\n', outDir);
disp(summary);
fprintf('Derivative-kick max-abs-output reduction: %.2f%%\n', reductionPct);
end

function enableLog(portHandle, signalName)
set_param(portHandle, 'DataLogging', 'on', ...
    'DataLoggingNameMode', 'Custom', 'DataLoggingName', signalName);
end

function y = sampleAt(ts, tGrid)
t = double(ts.Time(:));
yRaw = double(ts.Data(:));
[tUnique, idx] = unique(t, 'last');
yUnique = yRaw(idx);
if numel(tUnique) < 2
    error('Not enough logged samples for a meaningful simulation comparison.');
end
y = interp1(tUnique, yUnique, tGrid, 'previous', 'extrap');
end

function val = getParamSafe(block, key)
try
    val = string(get_param(block, key));
catch
    val = "-";
end
end
