function summary = run_normal_sil()
% Synthetic Normal versus SIL comparison; no board execution, no model save.
root = setup_portfolio();
out = fullfile(root,'.local','normal-sil','analysis');
if ~isfolder(out), mkdir(out); end
model = 'Roll_Pitch_SIL_Harness';
load_system(model);
cleanup = onCleanup(@() close_system(model,0));
block = [model '/Model'];
originalMode = get_param(block,'SimulationMode');
set_param(block,'SimulationMode','Normal');
normalOut = sim(model);
save(fullfile(out,'normal_run_5deg_minus3deg.mat'),'normalOut');
set_param(block,'SimulationMode','Software-in-the-loop (SIL)');
silOut = sim(model);
save(fullfile(out,'sil_run_5deg_minus3deg.mat'),'silOut');
set_param(block,'SimulationMode',originalMode);
summary = compare_roll_pitch_normal_sil();
assert(all(summary.Pass), 'Normal/SIL comparison failed');
end
