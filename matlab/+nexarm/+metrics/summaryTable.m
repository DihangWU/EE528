function summary=summaryTable(runs)
%SUMMARYTABLE One row per mode; all distance metrics use pen-down samples.
names=fieldnames(runs); count=numel(names);
xyz=zeros(count,1); xy=xyz; contour=xyz; joint=xyz; fallbacks=xyz; qps=xyz; solve=xyz;
for k=1:count
    result=runs.(names{k}); m=result.metrics; s=result.statistics;
    xyz(k)=m.pen_xyz_rms_mm; xy(k)=m.pen_xy_rms_mm;
    contour(k)=m.pen_xy_contour_rms_mm; joint(k)=m.joint_rms_deg;
    fallbacks(k)=s.fallbacks; qps(k)=s.active_qps; solve(k)=s.solve_p99_ms;
end
summary=table(string(names),xyz,xy,contour,joint,fallbacks,qps,solve,...
    'VariableNames',{'Mode','XYZ_RMS_mm','XY_RMS_mm','XY_contour_RMS_mm',...
    'Joint_RMS_deg','Fallbacks','ActiveJointQPs','SolveP99_ms'});
end
