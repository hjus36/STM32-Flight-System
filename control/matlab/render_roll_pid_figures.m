function render_roll_pid_figures()
% RENDER_ROLL_PID_FIGURES
% STM32 Flight System - portfolio plots from already-validated simulation data.
% MATLAB R2026b. Does NOT modify .slx files or re-run simulations.
%
% Place this file in: roll_single_pid_sim/
% Run: render_roll_pid_figures
% Reads: analysis/roll_pid_simulation_data.mat
% Writes: analysis/figures_clean/*.png

rootDir = fullfile(fileparts(mfilename('fullpath')), '..', 'simulink');
dataFile = fullfile(rootDir, '..', '..', '.local', 'matlab-analysis', 'roll_pid_simulation_data.mat');
if ~isfile(dataFile)
    error('Data file not found: %s', dataFile);
end
s = load(dataFile, 't', 'allRef', 'allY', 'allU');
assert(numel(s.t) == size(s.allY,1), 'Time / angle length mismatch');
assert(isequal(size(s.allY),size(s.allU),size(s.allRef)), 'Signal array sizes differ');
assert(size(s.allY,2) == 4, 'Expected P, PI, 1DOF PID, 2DOF PID');

outputDir = fullfile(rootDir, '..', '..', '.local', 'matlab-analysis', 'figures_clean');
if ~isfolder(outputDir), mkdir(outputDir); end
names = {'P','PI','PID (1DOF)','PID (2DOF)'};
T = s.t(:);
R = s.allRef(:,1)*180/pi;
Y = s.allY*180/pi;
U = s.allU;

f = newFigure(); ax = axes(f); hold(ax,'on');
plot(ax,T,R,'k--','LineWidth',1.7,'DisplayName','Target (5 deg)');
for i=1:4
    plot(ax,T,Y(:,i),'LineWidth',2,'DisplayName',names{i});
end
finalize(ax,'Roll response comparison','Roll angle (deg)',[0 10]);
legend(ax,'Location','southeast','TextColor','k','Color','w');
exportgraphics(f,fullfile(outputDir,'01_step_response_comparison_clean.png'), ...
    'Resolution',220,'BackgroundColor','white'); close(f);

f = newFigure(); ax = axes(f); hold(ax,'on');
for i=1:4
    plot(ax,T,U(:,i),'LineWidth',2,'DisplayName',names{i});
end
finalize(ax,'Controller output comparison','Controller output (arbitrary units)',[0 10]);
legend(ax,'Location','northeast','TextColor','k','Color','w');
exportgraphics(f,fullfile(outputDir,'02_control_output_comparison_clean.png'), ...
    'Resolution',220,'BackgroundColor','white'); close(f);

f = newFigure(); ax = axes(f); hold(ax,'on');
for i=1:4
    plot(ax,T,U(:,i),'LineWidth',2,'DisplayName',names{i});
end
finalize(ax,'Derivative kick: first 250 ms after the step', ...
    'Controller output (arbitrary units)',[0.95 1.25]);
legend(ax,'Location','northeast','TextColor','k','Color','w');
exportgraphics(f,fullfile(outputDir,'03_derivative_kick_zoom_clean.png'), ...
    'Resolution',220,'BackgroundColor','white'); close(f);

peakOld = max(abs(U(T>=1,3)));
peakNew = max(abs(U(T>=1,4)));
fprintf('Saved 3 clean figures in: %s\n', outputDir);
fprintf('Peak magnitude: 1DOF=%.6f, 2DOF=%.6f, reduction=%.2f%%\n', ...
    peakOld,peakNew,100*(peakOld-peakNew)/peakOld);
end

function f = newFigure()
f = figure('Visible','off','Color','w','Position',[120 120 1180 660]);
end

function finalize(ax,plotTitle,yAxisLabel,xLimits)
ax.Color = 'w';
ax.XColor = 'k'; ax.YColor = 'k';
ax.FontSize = 12;
ax.LineWidth = 0.8;
grid(ax,'on');
ax.GridAlpha = 0.14;
ax.MinorGridAlpha = 0.08;
xlim(ax,xLimits);
xlabel(ax,'Time (s)','Color','k','FontSize',13);
ylabel(ax,yAxisLabel,'Color','k','FontSize',13);
title(ax,plotTitle,'Color','k','FontSize',16,'FontWeight','bold');
end
