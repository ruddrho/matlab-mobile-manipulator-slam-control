function file=export_video(fps,speed)
% Export at a constant frame rate. Defaults: 30 fps, 2x simulation speed.
if nargin<1,fps=30;end
if nargin<2,speed=2;end
here=fileparts(mfilename('fullpath'));addpath(fullfile(here,'src'));c=mm_config();
file=fullfile(c.output,'mission.mat');
if ~exist(file,'file'),error('Run run_project(false) first.');end
S=load(file);file=mm_animate(S.mission,true,fps,speed);
end
