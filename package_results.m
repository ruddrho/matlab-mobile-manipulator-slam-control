function output=package_results()
% Bundle current results, including any optional exported video.
here=fileparts(mfilename('fullpath'));
assert(exist(fullfile(here,'results'),'dir')==7,'Results folder is missing.');
output=fullfile(here,'MATLAB_Results_For_Review.zip');
zip(output,{'results'},here);
fprintf('Send this result package for review: %s\n',output);
end
