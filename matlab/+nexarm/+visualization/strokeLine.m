function line=strokeLine(tips,ink)
%STROKELINE Include each segment's start and break at every pen lift.
ink=logical(ink(:)); starts=find(ink & ~[false;ink(1:end-1)]);
ends=find(ink & ~[ink(2:end);false]); chunks=cell(numel(starts),1);
for k=1:numel(starts)
    chunks{k}=[tips(max(1,starts(k)-1):ends(k),:);nan(1,size(tips,2))];
end
if isempty(chunks), line=nan(1,size(tips,2)); else, line=vertcat(chunks{:}); end
end
