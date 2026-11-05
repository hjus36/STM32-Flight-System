function summary = analyze_roll_pitch_double_pid()
% ANALYZE_ROLL_PITCH_DOUBLE_PID
% Quantitative verification of the independent Roll/Pitch cascade PID model.
%
% Model: Roll_Pitch_Double_PID.slx
% Required: 10 logged signals (target/actual angle, target/actual rate,
%           and control torque for each axis).
%
% How to run (place this file beside the .slx model):
%   summary = analyze_roll_pitch_double_pid;
%
% The script DOES NOT modify step input values or controller parameters.
% Results are saved in ./analysis/.
%
% Notes:
% - Angle signals are expected in radians; angular rate in rad/s;
%   control torque in N*m.
% - Rise time and settling time are measured AFTER the input step.
% - Negative step commands (e.g., Pitch -3 deg) are handled by using
%   normalized signed responses.
% - This verifies the existing idealized, uncoupled axis models, not
%   flight stability of a real drone.

    model = 'Roll_Pitch_Double_PID';
    projectDir = fullfile(fileparts(mfilename('fullpath')), '..', 'simulink');
    modelPath = fullfile(projectDir, [model '.slx']);
    analysisDir = fullfile(projectDir, '..', '..', '.local', 'matlab-analysis');

    if ~isfile(modelPath)
        error('Model not found: %s', modelPath);
    end
    if ~isfolder(analysisDir)
        mkdir(analysisDir);
    end

    if ~bdIsLoaded(model)
        load_system(modelPath);
    end

    fprintf('Running model: %s\n', model);
    simOut = sim(model);
    logs = simOut.logsout;

    requiredNames = [ ...
        "Target_Roll_Angle", "Actual_Roll_Angle", ...
        "Target_Roll_Rate",  "Actual_Roll_Rate", ...
        "Roll_Control_Torque", ...
        "Target_Pitch_Angle", "Actual_Pitch_Angle", ...
        "Target_Pitch_Rate",  "Actual_Pitch_Rate", ...
        "Pitch_Control_Torque"];
    availableNames = string(logs.getElementNames());
    missingNames = setdiff(requiredNames, availableNames);
    if ~isempty(missingNames)
        error('Missing logged signals: %s', strjoin(missingNames, ', '));
    end

    % Use actual Roll angle timestamps as the shared comparison timebase.
    [t, rollAngle] = readLoggedSeries(logs, 'Actual_Roll_Angle');
    [~, rollTarget] = readLoggedSeries(logs, 'Target_Roll_Angle', t, 'previous');
    [~, rollTargetRate] = readLoggedSeries(logs, 'Target_Roll_Rate', t, 'linear');
    [~, rollRate] = readLoggedSeries(logs, 'Actual_Roll_Rate', t, 'linear');
    [~, rollTorque] = readLoggedSeries(logs, 'Roll_Control_Torque', t, 'linear');

    [~, pitchTarget] = readLoggedSeries(logs, 'Target_Pitch_Angle', t, 'previous');
    [~, pitchAngle] = readLoggedSeries(logs, 'Actual_Pitch_Angle', t, 'linear');
    [~, pitchTargetRate] = readLoggedSeries(logs, 'Target_Pitch_Rate', t, 'linear');
    [~, pitchRate] = readLoggedSeries(logs, 'Actual_Pitch_Rate', t, 'linear');
    [~, pitchTorque] = readLoggedSeries(logs, 'Pitch_Control_Torque', t, 'linear');

    [rollStats, rollInfo] = evaluateAxis('Roll', t, rollTarget, rollAngle, rollRate, rollTorque);
    [pitchStats, pitchInfo] = evaluateAxis('Pitch', t, pitchTarget, pitchAngle, pitchRate, pitchTorque);
    summary = [rollStats; pitchStats];

    % For linear, equally parameterized axis models, dividing the output
    % by signed step amplitude makes +5 deg and -3 deg directly comparable.
    rollAngleNorm   = (rollAngle - rollInfo.initialAngle) / rollInfo.stepRad;
    pitchAngleNorm  = (pitchAngle - pitchInfo.initialAngle) / pitchInfo.stepRad;
    rollRateNorm    = rollRate / rollInfo.stepRad;
    pitchRateNorm   = pitchRate / pitchInfo.stepRad;
    rollTorqueNorm  = rollTorque / rollInfo.stepRad;
    pitchTorqueNorm = pitchTorque / pitchInfo.stepRad;

    bothAfterStep = (t >= max(rollInfo.stepTime, pitchInfo.stepTime));
    comparison = table( ...
        max(abs(rollAngleNorm(bothAfterStep) - pitchAngleNorm(bothAfterStep))), ...
        max(abs(rollRateNorm(bothAfterStep) - pitchRateNorm(bothAfterStep))), ...
        max(abs(rollTorqueNorm(bothAfterStep) - pitchTorqueNorm(bothAfterStep))), ...
        'VariableNames', {'MaxAbsNormalizedAngleDiff', ...
                          'MaxAbsNormalizedRateDiff_per_s', ...
                          'MaxAbsNormalizedTorqueDiff_Nm_per_rad'});

    % Export all ten original signals, preserving units in the column names.
    signalTable = table( ...
        t, rollTarget, rollAngle, rollTargetRate, rollRate, rollTorque, ...
        pitchTarget, pitchAngle, pitchTargetRate, pitchRate, pitchTorque, ...
        'VariableNames', { ...
            'Time_s', 'Target_Roll_Angle_rad', 'Actual_Roll_Angle_rad', ...
            'Target_Roll_Rate_rad_s', 'Actual_Roll_Rate_rad_s', 'Roll_Control_Torque_Nm', ...
            'Target_Pitch_Angle_rad', 'Actual_Pitch_Angle_rad', ...
            'Target_Pitch_Rate_rad_s', 'Actual_Pitch_Rate_rad_s', 'Pitch_Control_Torque_Nm'});

    normalizedTable = table(t, rollAngleNorm, pitchAngleNorm, ...
        rollRateNorm, pitchRateNorm, rollTorqueNorm, pitchTorqueNorm, ...
        'VariableNames', {'Time_s','Roll_Angle_Normalized','Pitch_Angle_Normalized', ...
         'Roll_Rate_per_s','Pitch_Rate_per_s', ...
         'Roll_Torque_Nm_per_rad','Pitch_Torque_Nm_per_rad'});

    writetable(summary, fullfile(analysisDir, 'roll_pitch_summary.csv'));
    writetable(signalTable, fullfile(analysisDir, 'roll_pitch_timeseries.csv'));
    writetable(normalizedTable, fullfile(analysisDir, 'roll_pitch_normalized.csv'));
    writetable(comparison, fullfile(analysisDir, 'roll_pitch_normalized_comparison.csv'));

    settings = struct();
    settings.Model = model;
    settings.RollStepAngle_deg = rad2deg(rollInfo.stepRad);
    settings.PitchStepAngle_deg = rad2deg(pitchInfo.stepRad);
    settings.RollStepTime_s = rollInfo.stepTime;
    settings.PitchStepTime_s = pitchInfo.stepTime;
    settings.AnalysisTime = char(datetime('now'));
    save(fullfile(analysisDir, 'roll_pitch_analysis.mat'), ...
        'summary', 'comparison', 'signalTable', 'normalizedTable', 'settings');

    % Plot 1: attitude angles (degrees).
    f = newAnalysisFigure();
    plot(t, rad2deg(rollTarget), '--', 'LineWidth', 1.6); hold on;
    plot(t, rad2deg(rollAngle), 'LineWidth', 2.2);
    plot(t, rad2deg(pitchTarget), '--', 'LineWidth', 1.6);
    plot(t, rad2deg(pitchAngle), 'LineWidth', 2.2);
    formatAxes('Roll/Pitch | Attitude angle tracking', 'Angle (deg)');
    legend('Roll target','Roll actual','Pitch target','Pitch actual','Location','best');
    saveAnalysisFigure(f, analysisDir, '01_roll_pitch_angle_responses.png');

    % Plot 2: angular rates (rad/s).
    f = newAnalysisFigure();
    plot(t, rollTargetRate, '--', 'LineWidth', 1.6); hold on;
    plot(t, rollRate, 'LineWidth', 2.2);
    plot(t, pitchTargetRate, '--', 'LineWidth', 1.6);
    plot(t, pitchRate, 'LineWidth', 2.2);
    formatAxes('Roll/Pitch | Angular rate responses', 'Angular rate (rad/s)');
    legend('Roll target','Roll actual','Pitch target','Pitch actual','Location','best');
    saveAnalysisFigure(f, analysisDir, '02_roll_pitch_rate_responses.png');

    % Plot 3: signed control torque (N*m).
    f = newAnalysisFigure();
    plot(t, rollTorque, 'LineWidth', 2.2); hold on;
    plot(t, pitchTorque, 'LineWidth', 2.2);
    yline(0, ':', 'LineWidth', 1);
    formatAxes('Roll/Pitch | Signed control torque', 'Control torque (N*m)');
    legend('Roll control torque','Pitch control torque','Zero torque','Location','best');
    saveAnalysisFigure(f, analysisDir, '03_roll_pitch_control_torque.png');

    % Plot 4: normalized actual attitude response.
    f = newAnalysisFigure();
    plot(t, rollAngleNorm, 'LineWidth', 2.2); hold on;
    plot(t, pitchAngleNorm, '--', 'LineWidth', 2.2);
    yline(1, ':', 'LineWidth', 1.2);
    formatAxes('Roll/Pitch | Normalized response overlay', 'Actual angle / signed step (unitless)');
    legend('Roll normalized','Pitch normalized','Normalized target','Location','best');
    saveAnalysisFigure(f, analysisDir, '04_roll_pitch_normalized_overlay.png');

    % Plot 5: signed tracking error (degrees).
    f = newAnalysisFigure();
    plot(t, rad2deg(rollTarget - rollAngle), 'LineWidth', 2.2); hold on;
    plot(t, rad2deg(pitchTarget - pitchAngle), 'LineWidth', 2.2);
    yline(0, ':', 'LineWidth', 1);
    formatAxes('Roll/Pitch | Attitude tracking errors', 'Target - actual (deg)');
    legend('Roll error','Pitch error','Zero error','Location','best');
    saveAnalysisFigure(f, analysisDir, '05_roll_pitch_tracking_errors.png');

    % Write a readable summary report.
    reportPath = fullfile(analysisDir, 'roll_pitch_analysis_report.txt');
    fid = fopen(reportPath, 'w');
    if fid == -1
        error('Cannot write report: %s', reportPath);
    end
    fileCleanup = onCleanup(@() fclose(fid));
    fprintf(fid, 'ROLL/PITCH DOUBLE PID - QUANTITATIVE ANALYSIS\n');
    fprintf(fid, 'Model: %s\n', model);
    fprintf(fid, 'Generated: %s\n', settings.AnalysisTime);
    fprintf(fid, 'Units: angle=deg, rate=rad/s, torque=N*m\n\n');
    for k = 1:height(summary)
        fprintf(fid, 'Axis: %s\n', char(summary.Axis(k)));
        fprintf(fid, '  Step time (s):               %.9g\n', summary.StepTime_s(k));
        fprintf(fid, '  Target angle (deg):          %.9g\n', summary.TargetAngle_deg(k));
        fprintf(fid, '  Angle at stop (deg):         %.12g\n', summary.AngleAtStop_deg(k));
        fprintf(fid, '  Final abs error (deg):       %.12g\n', summary.FinalAngleError_deg(k));
        fprintf(fid, '  Rise time 10-90%% (s):        %.9g\n', summary.RiseTime10to90_s(k));
        fprintf(fid, '  Settling time 2%% (s):        %.9g\n', summary.SettlingTime2pct_s(k));
        fprintf(fid, '  Overshoot (%%):               %.9g\n', summary.Overshoot_pct(k));
        fprintf(fid, '  Peak signed rate (rad/s):    %.9g\n', summary.PeakSignedRate_rad_s(k));
        fprintf(fid, '  Peak abs rate (rad/s):       %.9g\n', summary.PeakAbsRate_rad_s(k));
        fprintf(fid, '  Peak signed torque (N*m):    %.9g\n', summary.PeakSignedTorque_Nm(k));
        fprintf(fid, '  Peak abs torque (N*m):       %.9g\n', summary.PeakAbsTorque_Nm(k));
        fprintf(fid, '  IAE (deg*s):                 %.9g\n', summary.IAE_deg_s(k));
        fprintf(fid, '  Integral torque^2 (N*m)^2*s: %.9g\n', summary.IntegralTorqueSquared_Nm2_s(k));
        fprintf(fid, '  Median sample time (s):      %.9g\n\n', summary.MedianSampleTime_s(k));
    end
    fprintf(fid, 'NORMALIZED ROLL/PITCH COMPARISON\n');
    fprintf(fid, '  Max angle difference (unitless): %.12g\n', comparison.MaxAbsNormalizedAngleDiff);
    fprintf(fid, '  Max rate difference (1/s):      %.12g\n', comparison.MaxAbsNormalizedRateDiff_per_s);
    fprintf(fid, '  Max torque difference (N*m/rad): %.12g\n', comparison.MaxAbsNormalizedTorqueDiff_Nm_per_rad);
    fprintf(fid, '\nThese are uncoupled idealized models. Real airframe, actuator,\n');
    fprintf(fid, 'sensor and disturbance effects have not been validated.\n');
    clear fileCleanup;  % close the report file now

    fprintf('\nRoll/Pitch integrated analysis complete.\n');
    disp(summary);
    fprintf('Normalized max abs difference: angle=%.9g, rate=%.9g 1/s, torque=%.9g N*m/rad\n', ...
        comparison.MaxAbsNormalizedAngleDiff, ...
        comparison.MaxAbsNormalizedRateDiff_per_s, ...
        comparison.MaxAbsNormalizedTorqueDiff_Nm_per_rad);
    fprintf('Saved results to: %s\n', analysisDir);
end

function [t, y] = readLoggedSeries(logs, signalName, queryTime, method)
    % Read scalar timeseries and optionally resample on a common time grid.
    s = logs.getElement(signalName);
    if ~isa(s.Values, 'timeseries')
        error('Signal %s does not contain a MATLAB timeseries.', signalName);
    end
    t = double(s.Values.Time(:));
    y = squeeze(double(s.Values.Data));
    if ~isvector(y)
        error('Expected scalar signal for %s.', signalName);
    end
    y = y(:);
    if numel(t) ~= numel(y)
        error('Time/data length mismatch for %s.', signalName);
    end
    [t, order] = sort(t);
    y = y(order);
    % Handle any duplicate timestamps from simulation event boundaries.
    [t, keep] = unique(t, 'last');
    y = y(keep);

    if nargin >= 3 && ~isempty(queryTime)
        if nargin < 4
            method = 'linear';
        end
        if numel(t) == 1
            y = repmat(y(1), size(queryTime));
        else
            y = interp1(t, y, queryTime, method, 'extrap');
        end
        t = queryTime(:);
        y = y(:);
    end
end

function [stats, info] = evaluateAxis(axisName, t, target, actual, rate, torque)
    startTarget = target(1);
    endTarget = target(end);
    delta = endTarget - startTarget;
    if abs(delta) <= 1e-12
        error('%s command step is zero. Use a nonzero step target.', axisName);
    end
    stepThreshold = max(1e-10, 1e-5 * abs(delta));
    stepIndex = find(abs(target - startTarget) > stepThreshold, 1, 'first');
    if isempty(stepIndex)
        error('Could not find %s step time.', axisName);
    end
    tStep = t(stepIndex);
    beforeStep = find(t < tStep, 1, 'last');
    if isempty(beforeStep)
        initialAngle = actual(1);
    else
        initialAngle = actual(beforeStep);
    end

    after = (t >= tStep);
    tt = t(after);
    yy = actual(after);
    rr = rate(after);
    uu = torque(after);
    normalized = (yy - initialAngle) / delta;

    t10 = firstCrossing(tt, normalized, 0.10);
    t90 = firstCrossing(tt, normalized, 0.90);
    if isnan(t10) || isnan(t90)
        riseTime = NaN;
    else
        riseTime = t90 - t10;
    end

    settlingBand = 0.02 * abs(delta);
    lastOutside = find(abs(yy - endTarget) > settlingBand, 1, 'last');
    if isempty(lastOutside)
        settlingTime = 0;
    elseif lastOutside == numel(tt)
        settlingTime = NaN; % did not settle by stop time
    else
        settlingTime = tt(lastOutside + 1) - tStep;
    end

    overshoot = max(0, (max(normalized) - 1) * 100);
    [peakAbsRate, ixRate] = max(abs(rr));
    [peakAbsTorque, ixTorque] = max(abs(uu));
    peakSignedRate = rr(ixRate);
    peakSignedTorque = uu(ixTorque);
    finalAbsError = abs(rad2deg(endTarget - actual(end)));
    iae = trapz(tt, abs(rad2deg(target(after) - yy)));
    its = trapz(tt, uu.^2);

    stats = table(string(axisName), tStep, rad2deg(endTarget), ...
        rad2deg(actual(end)), finalAbsError, riseTime, settlingTime, ...
        overshoot, peakAbsRate, peakSignedRate, ...
        peakAbsTorque, peakSignedTorque, iae, its, median(diff(t)), ...
        'VariableNames', { ...
          'Axis', 'StepTime_s', 'TargetAngle_deg', 'AngleAtStop_deg', ...
          'FinalAngleError_deg', 'RiseTime10to90_s', 'SettlingTime2pct_s', ...
          'Overshoot_pct', 'PeakAbsRate_rad_s', 'PeakSignedRate_rad_s', ...
          'PeakAbsTorque_Nm', 'PeakSignedTorque_Nm', ...
          'IAE_deg_s', 'IntegralTorqueSquared_Nm2_s', 'MedianSampleTime_s'});
    info = struct('stepTime',tStep,'stepRad',delta,'initialAngle',initialAngle);
end

function crossingTime = firstCrossing(t, x, level)
    idx = find(x >= level, 1, 'first');
    if isempty(idx)
        crossingTime = NaN;
    elseif idx == 1
        crossingTime = t(1);
    else
        x0 = x(idx-1); x1 = x(idx);
        if x1 == x0
            crossingTime = t(idx);
        else
            alpha = (level - x0) / (x1 - x0);
            crossingTime = t(idx-1) + alpha * (t(idx) - t(idx-1));
        end
    end
end

function f = newAnalysisFigure()
    f = figure('Visible','off', 'Color','w', 'Position',[100 100 1440 740]);
    set(f, 'DefaultAxesFontSize', 12);
end

function formatAxes(plotTitle, yLabel)
    title(plotTitle, 'FontWeight','bold', 'Color',[0.12 0.12 0.12]);
    xlabel('Time (s)');
    ylabel(yLabel);
    grid on;
    box on;
end

function saveAnalysisFigure(f, analysisDir, fileName)
    exportgraphics(f, fullfile(analysisDir,fileName), 'Resolution', 180);
    close(f);
end
