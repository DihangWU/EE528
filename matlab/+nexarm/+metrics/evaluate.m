function metrics=evaluate(actual,tips,data)
%EVALUATE Pen-down tracking error and within-stroke contour distance.
ink=data.ink; reference=data.reference_tips;
difference=tips-reference; squared=sum(difference.^2,2);
metrics.pen_error_mm=sqrt(squared)*1000;
counts=cumsum(ink); cumulative=cumsum(squared.*ink);
metrics.running_rms_mm=sqrt(cumulative./max(1,counts))*1000;
metrics.pen_xyz_rms_mm=sqrt(mean(squared(ink)))*1000;
metrics.pen_xy_rms_mm=sqrt(mean(sum(difference(ink,1:2).^2,2)))*1000;
joint_error=actual(ink,1:5)-data.reference(ink,1:5);
metrics.joint_rms_deg=rad2deg(sqrt(mean(joint_error.^2,'all')));
starts=find(ink & ~[false;ink(1:end-1)]);
ends=find(ink & ~[ink(2:end);false]); distances=zeros(nnz(ink),1); cursor=0;
for stroke=1:numel(starts)
    first=starts(stroke); last=ends(stroke);
    a=reference(first-1:last-1,1:2); direction=reference(first:last,1:2)-a;
    length_squared=max(sum(direction.^2,2),1e-24);
    for k=first:last
        delta=tips(k,1:2)-a;
        fraction=min(1,max(0,sum(delta.*direction,2)./length_squared));
        cursor=cursor+1;
        distances(cursor)=min(sum((delta-fraction.*direction).^2,2));
    end
end
metrics.pen_xy_contour_rms_mm=sqrt(mean(distances))*1000;
end
