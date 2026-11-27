function [z,success,info] = solveQP(H,g,A,b,solver)
%SOLVEQP Strictly convex QP: min .5*z'*H*z+g'*z, A*z<=b.
% Dual coordinate ascent (Hildreth's method) is the toolbox-free backend.
% Success requires primal feasibility and dual complementary slackness.
if nargin<5, solver='auto'; end
solver=validatestring(solver,{'auto','dual','quadprog'});
tol=1e-10; success=false;
info=struct('backend','dual','iterations',0,'primal_residual',Inf,'complementarity',Inf);
if strcmp(solver,'quadprog') || (strcmp(solver,'auto') && exist('quadprog','file')==2)
    assert(exist('quadprog','file')==2,'nexarm:MissingQuadprog','quadprog is unavailable; use solver=''dual''.');
    options=optimoptions('quadprog','Display','off','ConstraintTolerance',1e-9,'OptimalityTolerance',1e-10);
    [z,~,flag,output]=quadprog(H,g,A,b,[],[],[],[],[],options);
    info.backend='quadprog'; info.iterations=output.iterations;
    if ~isempty(z), info.primal_residual=max([0;A*z-b]); end
    success=flag>0 && info.primal_residual<=tol;
    return
end
% Normalize rows so the stopping thresholds have consistent angle units.
scale=sqrt(sum(A.^2,2)); zero=scale<1e-14;
z=-(H\g);
if any(b(zero)<-tol), return; end
A=A(~zero,:)./scale(~zero); b=b(~zero)./scale(~zero);
if isempty(b), success=true; info.primal_residual=0; info.complementarity=0; return; end
P=H\A.'; denominator=sum(A.*P.',2);
lambda=zeros(size(b));
for sweep=1:4000
    before=z;
    for i=1:numel(b)
        updated=max(0,lambda(i)+(A(i,:)*z-b(i))/denominator(i));
        change=updated-lambda(i);
        if change~=0, z=z-P(:,i)*change; lambda(i)=updated; end
    end
    residual=A*z-b;
    info.iterations=sweep;
    info.primal_residual=max([0;residual]);
    info.complementarity=max(abs(lambda.*residual));
    if info.primal_residual<=tol && info.complementarity<=tol && norm(z-before,Inf)<1e-12
        success=true; return
    end
    % An infeasible polyhedron has an unbounded dual; do not claim success.
    if any(~isfinite(z)) || max(lambda)>1e12, return; end
end
end
