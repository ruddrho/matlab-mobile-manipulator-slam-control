function results=run_monte_carlo(N)
% Optional matched uncertainty study. Slow: two full rigid-body trials/seed.
if nargin<1,N=20;end
here=fileparts(mfilename('fullpath'));addpath(fullfile(here,'src'));c=mm_config();
if ~exist(c.output,'dir'),mkdir(c.output);end
results=zeros(N,6);
for i=1:N
 rng(1000+i);s=[2*rand .85+.30*rand .001+.004*rand];
 a=mm_trial(c,s,1,1000+i);b=mm_trial(c,s,2,1000+i);
 results(i,:)=[s a.rms b.rms a.rms-b.rms];fprintf('Matched trial %d/%d\n',i,N);
end
save(fullfile(c.output,'monte_carlo.mat'),'results','-v7');
f=fopen(fullfile(c.output,'monte_carlo.csv'),'w');cleanup=onCleanup(@()fclose(f));
fprintf(f,'payload_kg,mass_scale,noise_rad,PD_rms,CTC_rms,paired_difference\n');
for i=1:N,fprintf(f,'%.8f,%.8f,%.8f,%.8f,%.8f,%.8f\n',results(i,:));end
fprintf('Mean paired PD-CTC RMS difference = %.6f rad; SE = %.6f\n',mean(results(:,6)),std(results(:,6))/sqrt(N));
end
