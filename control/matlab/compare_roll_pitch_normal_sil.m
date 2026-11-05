function summary = compare_roll_pitch_normal_sil()
%COMPARE_ROLL_PITCH_NORMAL_SIL Compare Normal(single) and SIL(single) runs.
%
% Project: LKM_FC_V1 / Roll-Pitch Cascade Double PID
% Inputs (existing files in analysis/):
%   normal_run_5deg_minus3deg.mat  -> normalOut
%   sil_run_5deg_minus3deg.mat     -> silOut
%
% Does not run or modify Simulink models. Signal matching uses BOTH
% signal name and full BlockPath, avoiding duplicate-name ambiguity.
%
% Tested logic against the user's logged BlockPath inventory; not executed
% inside the user's MATLAB R2026b environment.

    projectDir = fullfile(fileparts(mfilename('fullpath')), '..', '..', '.local', 'normal-sil');
    analysisDir = fullfile(projectDir, 'analysis');
    normalFile = fullfile(analysisDir, 'normal_run_5deg_minus3deg.mat');
    silFile = fullfile(analysisDir, 'sil_run_5deg_minus3deg.mat');

    assert(isfile(normalFile), 'Missing Normal MAT file: %s', normalFile);
    assert(isfile(silFile), 'Missing SIL MAT file: %s', silFile);

    n = load(normalFile, 'normalOut');
    s = load(silFile, 'silOut');
    assert(isfield(n, 'normalOut'), 'normalOut not found in Normal MAT file.');
    assert(isfield(s, 'silOut'), 'silOut not found in SIL MAT file.');
    assert(any(strcmp(who(n.normalOut), 'logsout')), 'Normal logsout is missing.');
    assert(any(strcmp(who(s.silOut), 'logsout')), 'SIL logsout is missing.');

    normalLogs = n.normalOut.logsout;
    silLogs = s.silOut.logsout;

    % Exact source blocks established from the SIL log inventory.
    % The top-level Model block provides the controller torque outputs.
    % The separate top-level Plant blocks provide rate and angle outputs.
    names = { ...
        'Actual_Roll_Angle', ...
        'Actual_Roll_Rate', ...
        'Roll_Control_Torque', ...
        'Actual_Pitch_Angle', ...
        'Actual_Pitch_Rate', ...
        'Pitch_Control_Torque' ...
    };
    blocks = { ...
        'Roll_Pitch_SIL_Harness/Roll_Plant', ...
        'Roll_Pitch_SIL_Harness/Roll_Plant', ...
        'Roll_Pitch_SIL_Harness/Model', ...
        'Roll_Pitch_SIL_Harness/Pitch_Plant', ...
        'Roll_Pitch_SIL_Harness/Pitch_Plant', ...
        'Roll_Pitch_SIL_Harness/Model' ...
    };
    units = {'rad','rad/s','N*m','rad','rad/s','N*m'};
    fileTags = { ...
        '01_roll_angle', '02_roll_rate', '03_roll_torque', ...
        '04_pitch_angle', '05_pitch_rate', '06_pitch_torque' ...
    };

    % Conservative, explicit comparison limits for this prototype test.
    % Threshold per signal = absolute tolerance + relative tolerance *
    % maximum absolute signal magnitude across both runs.
    absTolerance = [1e-6; 1e-6; 1e-7; 1e-6; 1e-6; 1e-7];
    relativeTolerance = 1e-5;
    timeTolerance_s = 1e-10;

    nSignals = numel(names);
    samples = zeros(nSignals,1);
    maxTimeDiff_s = zeros(nSignals,1);
    maxAbsDifference = zeros(nSignals,1);
    rmsDifference = zeros(nSignals,1);
    finalDifference = zeros(nSignals,1);
    tolerances = zeros(nSignals,1);
    pass = false(nSignals,1);

    comparisons = struct();

    for k = 1:nSignals
        normalSignal = findSignalByNameAndPath(normalLogs, names{k}, blocks{k});
        silSignal = findSignalByNameAndPath(silLogs, names{k}, blocks{k});

        [tn, yn] = readScalarTimeSeries(normalSignal, names{k}, 'Normal');
        [ts, ys] = readScalarTimeSeries(silSignal, names{k}, 'SIL');

        assert(numel(tn) == numel(ts), ...
            '%s: different sample counts, Normal=%d SIL=%d. No implicit resampling performed.', ...
            names{k}, numel(tn), numel(ts));

        dt = max(abs(tn - ts));
        assert(dt <= timeTolerance_s, ...
            '%s: time mismatch %.12g s exceeds %.12g s. No implicit interpolation performed.', ...
            names{k}, dt, timeTolerance_s);

        difference = ys - yn;  % SIL minus Normal
        scale = max([abs(yn); abs(ys)]);
        tol = absTolerance(k) + relativeTolerance * scale;

        samples(k) = numel(tn);
        maxTimeDiff_s(k) = dt;
        maxAbsDifference(k) = max(abs(difference));
        rmsDifference(k) = sqrt(mean(difference .^ 2));
        finalDifference(k) = difference(end);
        tolerances(k) = tol;
        pass(k) = maxAbsDifference(k) <= tol;

        comparisons(k).name = names{k}; %#ok<AGROW>
        comparisons(k).blockPath = blocks{k};
        comparisons(k).unit = units{k};
        comparisons(k).time_s = tn;
        comparisons(k).normal = yn;
        comparisons(k).sil = ys;
        comparisons(k).difference = difference;
        comparisons(k).maxAbsDifference = maxAbsDifference(k);
        comparisons(k).tolerance = tol;
        comparisons(k).pass = pass(k);

        fig = figure('Visible','off', 'Color','w', ...
            'Position', [100 100 1180 760]);
        lay = tiledlayout(fig, 2, 1, 'TileSpacing','compact', 'Padding','compact');

        ax1 = nexttile(lay);
        plot(ax1, tn, yn, '-', 'LineWidth',1.5, 'DisplayName','Normal (single)');
        hold(ax1, 'on');
        plot(ax1, ts, ys, '--', 'LineWidth',1.4, 'DisplayName','SIL (single)');
        hold(ax1, 'off');
        ylabel(ax1, sprintf('%s (%s)', names{k}, units{k}), 'Interpreter','none');
        legend(ax1, 'Location','best');
        grid(ax1, 'on');
        title(ax1, sprintf('%s: Normal vs SIL', names{k}), ...
            'Interpreter','none', 'Color',[0.1 0.1 0.1]);

        ax2 = nexttile(lay);
        plot(ax2, tn, difference, 'LineWidth',1.3);
        hold(ax2, 'on');
        yline(ax2, tol, ':', 'Positive tolerance');
        yline(ax2, -tol, ':', 'Negative tolerance');
        hold(ax2, 'off');
        xlabel(ax2, 'Time (s)');
        ylabel(ax2, sprintf('SIL - Normal (%s)', units{k}));
        grid(ax2, 'on');
        title(ax2, sprintf('Max |diff| = %.9g, RMS = %.9g, tol = %.9g', ...
            maxAbsDifference(k), rmsDifference(k), tol), ...
            'Color',[0.1 0.1 0.1]);

        exportgraphics(fig, fullfile(analysisDir, ...
            [fileTags{k} '_normal_vs_sil.png']), 'Resolution', 200);
        close(fig);
    end

    summary = table(string(names(:)), string(units(:)), samples, ...
        maxTimeDiff_s, maxAbsDifference, rmsDifference, finalDifference, ...
        tolerances, pass, 'VariableNames', { ...
        'Signal','Unit','Samples','MaxTimeDiff_s','MaxAbsDifference', ...
        'RMSDifference','FinalDifference','Tolerance','Pass'});

    writetable(summary, fullfile(analysisDir, 'normal_sil_comparison_summary.csv'));
    save(fullfile(analysisDir, 'normal_sil_comparison.mat'), ...
        'summary','comparisons','absTolerance','relativeTolerance','timeTolerance_s');

    reportFile = fullfile(analysisDir, 'normal_sil_comparison_report.txt');
    fid = fopen(reportFile,'w');
    assert(fid > 0, 'Could not open report file: %s', reportFile);
    reportCleanup = onCleanup(@() fclose(fid));
    fprintf(fid, 'LKM_FC_V1 - Normal(single) vs SIL(single) verification\n');
    fprintf(fid, 'Harness: Roll_Pitch_SIL_Harness\n');
    fprintf(fid, 'Normal logged signals: %d\n', numElements(normalLogs));
    fprintf(fid, 'SIL logged signals: %d\n', numElements(silLogs));
    fprintf(fid, 'Time tolerance: %.12g s\n', timeTolerance_s);
    fprintf(fid, 'Tolerance rule: absTol + %.9g * peak signal magnitude\n\n', relativeTolerance);
    for k = 1:nSignals
        fprintf(fid, '%s [%s] (%s)\n', names{k}, units{k}, blocks{k});
        fprintf(fid, '  Samples: %d\n', samples(k));
        fprintf(fid, '  Max time difference: %.12g s\n', maxTimeDiff_s(k));
        fprintf(fid, '  Max abs difference: %.12g\n', maxAbsDifference(k));
        fprintf(fid, '  RMS difference: %.12g\n', rmsDifference(k));
        fprintf(fid, '  Final difference: %.12g\n', finalDifference(k));
        fprintf(fid, '  Tolerance: %.12g\n', tolerances(k));
        fprintf(fid, '  PASS: %d\n\n', pass(k));
    end
    fprintf(fid, 'OVERALL: %s (%d/%d signals)\n', ...
        verdictText(all(pass)), sum(pass), nSignals);
    clear reportCleanup

    fprintf('\nNormal/SIL comparison complete.\n');
    disp(summary);
    fprintf('OVERALL: %s (%d/%d signals passed)\n', ...
        verdictText(all(pass)), sum(pass), nSignals);
    fprintf('Saved results to: %s\n', analysisDir);
end

function signal = findSignalByNameAndPath(logs, wantedName, wantedPath)
    matches = [];
    available = strings(0,1);
    for j = 1:numElements(logs)
        entry = get(logs, j);
        pathParts = convertToCell(entry.BlockPath);
        fullPath = strjoin(pathParts, ' -> ');
        available(end+1,1) = string(entry.Name) + " | " + string(fullPath); %#ok<AGROW>
        if strcmp(char(entry.Name), wantedName) && strcmp(fullPath, wantedPath)
            matches(end+1) = j; %#ok<AGROW>
        end
    end
    if numel(matches) ~= 1
        fprintf(2, '\nAvailable logged entries:\n');
        disp(available);
        error('Expected exactly 1 logged signal matching name "%s" at path "%s"; found %d.', ...
            wantedName, wantedPath, numel(matches));
    end
    signal = get(logs, matches(1));
end

function [t, y] = readScalarTimeSeries(signal, name, modeName)
    values = signal.Values;
    assert(isa(values, 'timeseries'), ...
        '%s/%s must contain a timeseries.', modeName, name);
    t = double(values.Time(:));
    raw = values.Data;
    assert(isnumeric(raw) && isreal(raw), '%s/%s data must be real numeric.', modeName, name);
    y = double(raw(:));
    assert(numel(t) == numel(y), '%s/%s is not a scalar-per-sample signal.', modeName, name);
    assert(~isempty(t), '%s/%s has no samples.', modeName, name);
    assert(all(isfinite(t)) && all(isfinite(y)), ...
        '%s/%s contains NaN or Inf.', modeName, name);
end

function textOut = verdictText(isPass)
    if isPass
        textOut = 'PASS';
    else
        textOut = 'FAIL';
    end
end
