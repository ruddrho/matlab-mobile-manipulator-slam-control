function replay_simulation(speed,fps)
% Replay saved results without recomputing the mission. Defaults: 2x, 25 fps.
if nargin<1,speed=2;end
if nargin<2,fps=25;end
here=fileparts(mfilename('fullpath'));addpath(fullfile(here,'src'));c=mm_config();
file=fullfile(c.output,'mission.mat');
if ~exist(file,'file'),error('Run run_project(false) first.');end
S=load(file);mm_animate(S.mission,false,fps,speed);
end
