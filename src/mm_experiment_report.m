function mm_experiment_report(results,c)
cases=vertcat(results(1:2:end).case);
save(fullfile(c.output,'controller_experiments.mat'),'results','-v7');
f=fopen(fullfile(c.output,'controller_metrics.csv'),'w');cleanup=onCleanup(@()fclose(f));
fprintf(f,'controller,payload_kg,mass_scale,sensor_std_rad,rms_error_rad,peak_error_rad,effort_Nm2s,saturation_fraction,final_error_rad\n');
for j=1:numel(results)
 r=results(j);fprintf(f,'%s,%.3f,%.3f,%.5f,%.8f,%.8f,%.6f,%.6f,%.8f\n',r.controller,r.case,r.rms,r.peak,r.effort,r.saturation,r.final);
end
fig=figure('Name','Controller comparison','Color','w','Position',[80 80 1100 780]);
for j=1:size(cases,1)
 subplot(size(cases,1),2,2*j-1);hold on;
 for k=1:2,r=results(2*j-2+k);plot(r.t,sqrt(sum(r.error.^2,2)),'LineWidth',1.1);end
 grid on;ylabel('Error norm (rad)');xlabel('Time (s)');title(sprintf('Payload %.1f kg | mass x%.2f | noise %.3f rad',cases(j,:)));legend('PD + gravity','Computed torque');
 subplot(size(cases,1),2,2*j);hold on;
 for k=1:2,r=results(2*j-2+k);plot(r.t,max(abs(r.tau),[],2),'LineWidth',1);end
 grid on;ylabel('Peak torque (Nm)');xlabel('Time (s)');
end
print(fig,fullfile(c.output,'controller_comparison.png'),'-dpng','-r150');
end
