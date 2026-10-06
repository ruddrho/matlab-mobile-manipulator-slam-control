function results=run_experiments(quick)
% Torque-driven arm dynamics: PD+gravity versus computed-torque control.
% Matched references, seeds, disturbances and actuator limits per case.
if nargin<1,quick=false;end
here=fileparts(mfilename('fullpath'));addpath(fullfile(here,'src'));c=mm_config();
if ~exist(c.output,'dir'),mkdir(c.output);end
cases=[0 1 0;1 1 0;2 1 0;2 1.20 .003]; % payload, link mass multiplier, sensor std
if quick,cases=cases([2 4],:);end
results=struct([]);idx=0;
for j=1:size(cases,1)
 for controller=1:2
  idx=idx+1;fprintf('Experiment %d / %d\n',idx,size(cases,1)*2);
  trial=mm_trial(c,cases(j,:),controller,17+j);
  % MATLAB requires matching fields for indexed struct assignment.
  if idx==1
   results=trial;
  else
   results(idx)=trial; %#ok<AGROW>
  end
 end
end
mm_experiment_report(results,c);
end
