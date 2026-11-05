function summary = analyze_pitch_double_pid()
% ANALYZE_PITCH_DOUBLE_PID
% Analyze the logged signals in Pitch_Double_PID.slx (MATLAB R2026b).
% This function does not modify the Simulink model and does not flash hardware.
%
% Required logged signals:
%   Target_Pitch_Angle, Actual_Pitch_Angle, Target_Pitch_Rate, Actual_Pitch_Rate, Pitch_Control_Torque
%
% Outputs under ./analysis/:
%   pitch_double_pid_signals.csv
%   pitch_double_pid_summary.csv
%   pitch_double_pid_simulation_data.mat
%   pitch_double_pid_validation_report.md
%   figures_clean/01_pitch_angle_response_clean.png
%   figures_clean/02_pitch_rate_response_clean.png
%   figures_clean/03_control_torque_clean.png
%   figures_clean/04_pitch_angle_error_clean.png
%
% Usage:
%   summary = analyze_pitch_double_pid;
%
% Optional: if ../roll_double_pid_sim/analysis/roll_double_pid_signals.csv
% exists, also compare the two cascades without modifying the Roll files.

    modelName = 'Pitch_Double_PID';
    rootDir = fullfile(fileparts(mfilename('fullpath')), '..', 'simulink');
    if isempty(rootDir)
        rootDir = pwd;
    end
    modelFile = fullfile(rootDir, [modelName '.slx']);
    assert(isfile(modelFile), ...
        'Cannot find %s. Place this .m file next to Pitch_Double_PID.slx.', modelFile);

    outputDir = fullfile(rootDir, '..', '..', '.local', 'matlab-analysis');
    figureDir = fullfile(outputDir, 'figures_clean');
    if ~isfolder(outputDir), mkdir(outputDir); end
    if ~isfolder(figureDir), mkdir(figureDir); end

    oldDir = pwd;
    restoreDir = onCleanup(@() cd(oldDir)); %#ok<NASGU>
    cd(rootDir);
    load_system(modelFile);

    fprintf('Running model: %s\n', modelName);
    simOut = sim(modelName);
    logs = simOut.logsout;

    required = {'Target_Pitch_Angle', 'Actual_Pitch_Angle', 'Target_Pitch_Rate', ...
        'Actual_Pitch_Rate', 'Pitch_Control_Torque'};
    names = cellstr(string(logs.getElementNames()));
    missing = required(~ismember(required, names));
    if ~isempty(missing)
        error('Missing logged signals: %s', strjoin(missing, ', '));
    end

    % Use the actual-angle timestamps as the common 10 ms analysis grid.
    angleTS = logs.get('Actual_Pitch_Angle').Values;
    t = unique(double(angleTS.Time(:)));
    assert(numel(t) >= 3, 'Actual_Pitch_Angle contains too few time samples.');

    targetAngle = readOnGrid(logs, 'Target_Pitch_Angle', t);
    actualAngle = readOnGrid(logs, 'Actual_Pitch_Angle', t);
    targetRate  = readOnGrid(logs, 'Target_Pitch_Rate', t);
    actualRate  = readOnGrid(logs, 'Actual_Pitch_Rate', t);
    torque      = readOnGrid(logs, 'Pitch_Control_Torque', t);

    % The model uses radians and rad/s; keep original units in saved CSV.
    angleError = targetAngle - actualAngle;
    rad2degFactor = 180 / pi;
    targetAngleDeg = targetAngle * rad2degFactor;
    actualAngleDeg = actualAngle * rad2degFactor;
    angleErrorDeg  = angleError * rad2degFactor;
    targetRateDegS = targetRate * rad2degFactor;
    actualRateDegS = actualRate * rad2degFactor;

    initialTarget = targetAngle(1);
    finalTarget = targetAngle(end);
    stepSize = finalTarget - initialTarget;
    assert(abs(stepSize) > 1e-12, 'Target_Pitch_Angle has no detectable step.');

    stepIndex = find(abs(targetAngle - initialTarget) >= 0.5 * abs(stepSize), 1, 'first');
    assert(~isempty(stepIndex), 'Cannot determine target step time.');
    tStep = t(stepIndex);
    postStep = (t >= tStep);
    tPost = t(postStep);
    yPost = actualAngle(postStep);
    torquePost = torque(postStep);
    ratePost = actualRate(postStep);

    % Measured, sampled-data thresholds; temporal resolution is approximately Ts.
    rise10 = thresholdTime(tPost, yPost, initialTarget, stepSize, 0.10);
    rise90 = thresholdTime(tPost, yPost, initialTarget, stepSize, 0.90);
    if isnan(rise10) || isnan(rise90)
        riseTime = NaN;
    else
        riseTime = rise90 - rise10;
    end

    % Settling time: first recorded time from which all remaining samples
    % stay within +/-2% of the commanded STEP AMPLITUDE (not the final output).
    bandRad = 0.02 * abs(stepSize);
    outside = find(abs(yPost - finalTarget) > bandRad);
    if isempty(outside)
        settlingTime = 0;
    elseif outside(end) == numel(tPost)
        settlingTime = NaN;
    else
        settlingTime = tPost(outside(end) + 1) - tStep;
    end

    if stepSize >= 0
        overshootPct = max(0, (max(yPost) - finalTarget) / abs(stepSize) * 100);
    else
        overshootPct = max(0, (finalTarget - min(yPost)) / abs(stepSize) * 100);
    end

    [peakAbsRate, rateIndex] = max(abs(ratePost));
    peakSignedRate = ratePost(rateIndex);
    [peakAbsTorque, torqueIndex] = max(abs(torquePost));
    peakSignedTorque = torquePost(torqueIndex);
    finalErrorDeg = abs(finalTarget - actualAngle(end)) * rad2degFactor;
    iaeDegS = trapz(tPost, abs(targetAngle(postStep) - yPost)) * rad2degFactor;
    torqueSquaredIntegral = trapz(tPost, torquePost .^ 2);
    sampleTimeMedian = median(diff(t));

    summary = table(tStep, finalTarget * rad2degFactor, ...
        actualAngleDeg(end), finalErrorDeg, riseTime, settlingTime, ...
        overshootPct, peakAbsRate, peakSignedRate, ...
        peakAbsTorque, peakSignedTorque, iaeDegS, torqueSquaredIntegral, ...
        sampleTimeMedian, ...
        'VariableNames', {'StepTime_s', 'TargetAngle_deg', ...
        'AngleAtStop_deg', 'FinalAngleError_deg', 'RiseTime10to90_s', ...
        'SettlingTime2pct_s', 'Overshoot_pct', 'PeakAbsRate_rad_s', ...
        'PeakSignedRate_rad_s', 'PeakAbsTorque_Nm', ...
        'PeakSignedTorque_Nm', 'IAE_deg_s', ...
        'IntegralTorqueSquared_Nm2_s', 'MedianSampleTime_s'});

    signalTable = table(t, targetAngle, actualAngle, angleError, ...
        targetRate, actualRate, torque, targetAngleDeg, actualAngleDeg, ...
        angleErrorDeg, targetRateDegS, actualRateDegS, ...
        'VariableNames', {'Time_s', 'Target_Pitch_Angle_rad', 'Actual_Pitch_Angle_rad', ...
        'Angle_Error_rad', 'Target_Pitch_Rate_rad_s', 'Actual_Pitch_Rate_rad_s', ...
        'Pitch_Control_Torque_Nm', 'Target_Pitch_Angle_deg', 'Actual_Pitch_Angle_deg', ...
        'Angle_Error_deg', 'Target_Pitch_Rate_deg_s', 'Actual_Pitch_Rate_deg_s'});

    writetable(signalTable, fullfile(outputDir, 'pitch_double_pid_signals.csv'));
    writetable(summary, fullfile(outputDir, 'pitch_double_pid_summary.csv'));
    save(fullfile(outputDir, 'pitch_double_pid_simulation_data.mat'), ...
        't', 'targetAngle', 'actualAngle', 'targetRate', 'actualRate', ...
        'torque', 'angleError', 'summary', 'signalTable');

    saveLineFigure(figureDir, '01_pitch_angle_response_clean.png', ...
        t, targetAngleDeg, actualAngleDeg, tStep, ...
        'Pitch double-loop | angle response', 'Pitch angle (deg)', ...
        {'Target angle', 'Actual angle'});
    saveLineFigure(figureDir, '02_pitch_rate_response_clean.png', ...
        t, targetRateDegS, actualRateDegS, tStep, ...
        'Pitch double-loop | angular rate response', 'Pitch rate (deg/s)', ...
        {'Target rate', 'Actual rate'});
    saveLineFigure(figureDir, '03_control_torque_clean.png', ...
        t, torque, [], tStep, ...
        'Pitch double-loop | control torque', 'Control torque (N*m)', ...
        {'PI output'});
    saveLineFigure(figureDir, '04_pitch_angle_error_clean.png', ...
        t, angleErrorDeg, [], tStep, ...
        'Pitch double-loop | tracking error', 'Target - actual (deg)', ...
        {'Angle error'});

    % Optional Roll/Pitch equivalence check with existing Roll data.
    % Both learning models should agree when controller/plant parameters,
    % initial conditions, and discrete sample times are identical.
    rollFile = fullfile(rootDir, '..', '..', '.local', 'matlab-analysis', 'roll_double_pid_signals.csv');
    rollComparisonRan = false;
    rollAngleMaxDifference = NaN;
    rollRateMaxDifference = NaN;
    rollTorqueMaxDifference = NaN;
    if isfile(rollFile)
        rollTable = readtable(rollFile);
        requiredRollCols = {'Time_s', 'Actual_Angle_rad', ...
            'Actual_Rate_rad_s', 'Control_Torque_Nm'};
        if all(ismember(requiredRollCols, rollTable.Properties.VariableNames))
            tRoll = double(rollTable.Time_s(:));
            [tRoll, ixRoll] = unique(tRoll, 'last');
            if t(1) >= tRoll(1) - 1e-9 && t(end) <= tRoll(end) + 1e-9
                rollAngleRaw = double(rollTable.Actual_Angle_rad(ixRoll));
                rollRateRaw = double(rollTable.Actual_Rate_rad_s(ixRoll));
                rollTorqueRaw = double(rollTable.Control_Torque_Nm(ixRoll));
                if numel(tRoll) == numel(t) && max(abs(tRoll - t)) < 1e-8
                    % Avoid ZOH one-sample offsets due to tiny CSV time rounding.
                    rollAngle = rollAngleRaw;
                    rollRate = rollRateRaw;
                    rollTorque = rollTorqueRaw;
                else
                    rollAngle = interp1(tRoll, rollAngleRaw, t, 'previous', 'extrap');
                    rollRate = interp1(tRoll, rollRateRaw, t, 'previous', 'extrap');
                    rollTorque = interp1(tRoll, rollTorqueRaw, t, 'previous', 'extrap');
                end
                rollAngleMaxDifference = max(abs(actualAngle - rollAngle));
                rollRateMaxDifference = max(abs(actualRate - rollRate));
                rollTorqueMaxDifference = max(abs(torque - rollTorque));
                comparedSignals = {'Actual angle'; 'Actual rate'; 'Control torque'};
                maxAbsDiff = [rollAngleMaxDifference; rollRateMaxDifference; ...
                    rollTorqueMaxDifference];
                units = {'rad'; 'rad/s'; 'N*m'};
                comparison = table(comparedSignals, maxAbsDiff, units, ...
                    'VariableNames', {'Signal', 'MaxAbsDifference', 'Unit'});
                writetable(comparison, ...
                    fullfile(outputDir, 'roll_vs_pitch_comparison.csv'));
                saveLineFigure(figureDir, '05_roll_vs_pitch_angle_clean.png', ...
                    t, actualAngleDeg, rollAngle * rad2degFactor, tStep, ...
                    'Roll vs Pitch | angle overlay', 'Angle (deg)', ...
                    {'Pitch actual', 'Roll actual'});
                rollComparisonRan = true;
                fprintf('Roll/Pitch max abs difference: angle=%.9g rad, ', ...
                    rollAngleMaxDifference);
                fprintf('rate=%.9g rad/s, torque=%.9g N*m\n', ...
                    rollRateMaxDifference, rollTorqueMaxDifference);
            else
                fprintf('Roll CSV time coverage differs; skipping cross-comparison.\n');
            end
        else
            fprintf('Roll CSV lacks required columns; skipping cross-comparison.\n');
        end
    else
        fprintf('Roll analysis CSV not found; Pitch analysis proceeds independently.\n');
    end

    reportFile = fullfile(outputDir, 'pitch_double_pid_validation_report.md');
    fid = fopen(reportFile, 'w', 'n', 'UTF-8');
    assert(fid > 0, 'Cannot write %s', reportFile);
    fprintf(fid, '# Pitch Double PID - Simulation Validation Report\n\n');
    fprintf(fid, '- Model: `%s.slx`\n', modelName);
    fprintf(fid, '- Date: %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
    fprintf(fid, '- Simulation window: %.3f to %.3f s\n', t(1), t(end));
    fprintf(fid, '- Measured median sample interval: %.6f s\n', sampleTimeMedian);
    fprintf(fid, '- Step at t = %.4f s, target = %.6f deg\n\n', ...
        tStep, finalTarget * rad2degFactor);
    fprintf(fid, '## Controller and plant assumptions\n\n');
    fprintf(fid, '- Outer loop: angle P = 2 (configured learning model)\n');
    fprintf(fid, '- Inner loop: rate PI, Kp = 0.05, Ki = 0.2 (configured learning model)\n');
    fprintf(fid, '- Assumed plant: Jy = 0.002 kg*m^2, By = 0.02 N*m*s/rad\n');
    fprintf(fid, '- The script extracts simulation outputs; these nominal parameter settings are not independently read from Simulink block dialogs.\n\n');
    fprintf(fid, '## Measured results\n\n');
    fprintf(fid, '| Metric | Value |\n|---|---:|\n');
    fprintf(fid, '| Angle at stop | %.8f deg |\n', actualAngleDeg(end));
    fprintf(fid, '| Error at stop | %.8f deg |\n', finalErrorDeg);
    fprintf(fid, '| Rise time (10%% to 90%% of commanded step) | %s s |\n', fmtNum(riseTime));
    fprintf(fid, '| Settling time (target +/-2%% of step amplitude) | %s s |\n', fmtNum(settlingTime));
    fprintf(fid, '| Overshoot over target | %.6f %% |\n', overshootPct);
    fprintf(fid, '| Peak absolute actual angular rate | %.8f rad/s |\n', peakAbsRate);
    fprintf(fid, '| Peak absolute PI control torque | %.8f N*m |\n', peakAbsTorque);
    fprintf(fid, '| IAE from step to stop | %.8f deg*s |\n', iaeDegS);
    fprintf(fid, '| Integral of squared torque from step to stop | %.8g (N*m)^2*s |\n\n', torqueSquaredIntegral);
    fprintf(fid, '## Metric definitions\n\n');
    fprintf(fid, '- Rise time: elapsed time between first sampled crossings of 10%% and 90%% of the commanded angle step.\n');
    fprintf(fid, '- Settling time: elapsed time since command step after which all remaining recorded samples stay within +/-2%% of the commanded step amplitude.\n');
    fprintf(fid, '- `NaN` means the criterion was not reached before the simulation stopped.\n');
    fprintf(fid, '- Error at stop is not necessarily infinite-time steady-state error.\n');
    fprintf(fid, '- IAE = time integral of the absolute target-angle tracking error from step to stop.\n\n');
    fprintf(fid, '## Files\n\n');
    fprintf(fid, '- `pitch_double_pid_signals.csv` - time-aligned logged signals\n');
    fprintf(fid, '- `pitch_double_pid_summary.csv` - measured metrics\n');
    fprintf(fid, '- `pitch_double_pid_simulation_data.mat` - MATLAB analysis data\n');
    fprintf(fid, '- `figures_clean/` - four exported plots\n\n');
    if rollComparisonRan
        fprintf(fid, '## Roll vs Pitch comparison (optional)\n\n');
        fprintf(fid, '- Reference: `../roll_double_pid_sim/analysis/roll_double_pid_signals.csv`\n');
        fprintf(fid, '- Max absolute actual-angle difference: %.10g rad\n', rollAngleMaxDifference);
        fprintf(fid, '- Max absolute actual-rate difference: %.10g rad/s\n', rollRateMaxDifference);
        fprintf(fid, '- Max absolute control-torque difference: %.10g N*m\n', rollTorqueMaxDifference);
        fprintf(fid, '- Differences here reflect sampled simulation signals; identical parameters should yield equivalent response.\n');
        fprintf(fid, '- See `roll_vs_pitch_comparison.csv` and `figures_clean/05_roll_vs_pitch_angle_clean.png`.\n\n');
    else
        fprintf(fid, '## Roll vs Pitch comparison\n\n');
        fprintf(fid, '- Optional comparison not performed (Roll CSV missing or incompatible).\n\n');
    end
    fprintf(fid, '## Validation boundary\n\n');
    fprintf(fid, '- These results validate the simplified simulated pitch cascade under the specified step test.\n');
    fprintf(fid, '- Drone inertia, motor/ESC dynamics, sensors, saturation, disturbance rejection, MCU timing, and real flight behavior are NOT validated.\n');
    fprintf(fid, '- No STM32 code generation, arm build, or deployment is performed by this analysis script.\n');
    fclose(fid);

    fprintf('\nPitch double PID analysis complete.\n');
    disp(summary);
    fprintf('Saved results to: %s\n', outputDir);
end

function values = readOnGrid(logs, signalName, t)
    signal = logs.get(signalName);
    assert(~isempty(signal), 'Signal %s is not logged.', signalName);
    ts = signal.Values;
    assert(isa(ts, 'timeseries'), '%s must be scalar timeseries data.', signalName);
    time = double(ts.Time(:));
    valuesRaw = double(ts.Data(:));
    assert(numel(time) == numel(valuesRaw), ...
        '%s: expected one scalar value per time sample.', signalName);
    assert(all(isfinite(time)) && all(isfinite(valuesRaw)), ...
        '%s has NaN/Inf values.', signalName);
    [time, lastIndices] = unique(time, 'last');
    valuesRaw = valuesRaw(lastIndices);
    assert(numel(time) >= 2, '%s has too few data samples.', signalName);

    if numel(time) == numel(t) && max(abs(time - t)) <= 1e-8
        values = valuesRaw;
    else
        % A zero-order hold is appropriate for this discrete-time model.
        values = interp1(time, valuesRaw, t, 'previous', 'extrap');
    end
    values = values(:);
end

function crossingTime = thresholdTime(t, y, baseline, stepSize, fraction)
    normalizedProgress = (y - baseline) * sign(stepSize);
    ind = find(normalizedProgress >= fraction * abs(stepSize), 1, 'first');
    if isempty(ind)
        crossingTime = NaN;
    else
        crossingTime = t(ind);
    end
end

function saveLineFigure(outDir, fileName, t, first, second, tStep, figTitle, yLabel, seriesLabels)
    f = figure('Visible', 'off', 'Color', 'w', ...
        'Position', [100, 100, 1100, 580]);
    ax = axes('Parent', f);
    plot(ax, t, first, 'LineWidth', 1.9, 'Color', [0.05, 0.42, 0.73]);
    hold(ax, 'on');
    if ~isempty(second)
        plot(ax, t, second, 'LineWidth', 1.9, 'Color', [0.9, 0.35, 0.16]);
    end
    xline(ax, tStep, ':', 'Step', 'Color', [0.35, 0.35, 0.35], ...
        'LabelVerticalAlignment', 'bottom', 'HandleVisibility', 'off');
    grid(ax, 'on');
    set(ax, 'Color', 'w', 'XColor', [0.16, 0.19, 0.24], ...
        'YColor', [0.16, 0.19, 0.24], 'GridAlpha', 0.16, ...
        'FontName', 'Arial', 'FontSize', 11);
    title(ax, figTitle, 'FontSize', 15, 'FontWeight', 'bold');
    xlabel(ax, 'Time (s)');
    ylabel(ax, yLabel);
    legend(ax, seriesLabels, 'Location', 'best');
    xlim(ax, [t(1), t(end)]);
    exportgraphics(f, fullfile(outDir, fileName), 'Resolution', 200);
    close(f);
end

function s = fmtNum(value)
    if isnan(value)
        s = 'NaN (not reached in simulation window)';
    else
        s = sprintf('%.6f', value);
    end
end
