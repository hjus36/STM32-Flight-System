function root = setup_portfolio()
% Read public copies only; send generated/cache outputs to ignored .local.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root,'control','simulink'));
addpath(fullfile(root,'control','matlab'));
cache = fullfile(root,'.local','simulink-cache');
gen = fullfile(root,'.local','simulink-codegen');
if ~isfolder(cache), mkdir(cache); end
if ~isfolder(gen), mkdir(gen); end
Simulink.fileGenControl('set','CacheFolder',cache,'CodeGenFolder',gen,'createDir',true);
end
