function resume_results()
% Resume the controller experiments without recomputing completed mission/SLAM.
here=fileparts(mfilename('fullpath'));addpath(here,fullfile(here,'src'));
c=mm_config();assert(exist(fullfile(c.output,'mission.mat'),'file')==2, ...
 'Saved mission missing. Run run_all_results instead.');
diary(fullfile(c.output,'experiments_resume_log.txt'));cleanup=onCleanup(@()diary('off')); %#ok<NASGU>
fprintf('Resumed controller experiments: %s | MATLAB %s\n',datestr(now,31),version);
try
 run_experiments(false);
 fprintf('ALL 8 CONTROLLER EXPERIMENTS COMPLETED\n');
catch err
 fprintf(2,'FAILED: %s\n%s\n',err.identifier,err.message);
 diary off;package_results;rethrow(err);
end
diary off;package_results;replay_simulation;
end
