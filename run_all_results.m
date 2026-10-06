function run_all_results(showReplay)
% Generate fresh MATLAB evidence for review before the GitHub release.
if nargin<1,showReplay=true;end
here=fileparts(mfilename('fullpath'));addpath(here,fullfile(here,'src'),fullfile(here,'tests'));
c=mm_config();if ~exist(c.output,'dir'),mkdir(c.output);end
% Preserve an earlier run instead of mixing measurements from different runs.
items=dir(c.output);names={items.name};
if any(~ismember(names,{'.','..'}))
 archive=fullfile(here,['results_previous_' datestr(now,'yyyymmdd_HHMMSS')]);
 if exist(archive,'dir'),archive=[archive '_' num2str(round(1e6*rand))];end
 [ok,msg]=movefile(c.output,archive);assert(ok,msg);mkdir(c.output);
end
diary(fullfile(c.output,'matlab_run_log.txt'));cleanup=onCleanup(@()diary('off')); %#ok<NASGU>
fprintf('Started: %s\nMATLAB: %s\nPlatform: %s\n',datestr(now,31),version,computer);
try
 run_tests;
 mission=run_project(false);assert(mission.completed,'Mission did not complete successfully.');
 test_long_route;
 test_station_clearance;
 test_slam;
 run_experiments(false);
 fprintf('ALL REQUESTED RUNS COMPLETED: %s\n',datestr(now,31));
catch err
 fprintf(2,'FAILED: %s\n%s\n',err.identifier,err.message);
 for k=1:numel(err.stack),fprintf(2,'%s line %d\n',err.stack(k).name,err.stack(k).line);end
 diary off;package_results;rethrow(err);
end
diary off;package_results;
if showReplay,replay_simulation;end
end
