function [overloaded,Node_From,Node_To] = FindOverLoadBranch(results_pf)
overload_frac = 1;
[F_BUS, T_BUS, BR_R, BR_X, BR_B, RATE_A, RATE_B, RATE_C, ...
    TAP, SHIFT, BR_STATUS, PF, QF, PT, QT, MU_SF, MU_ST, ...
    ANGMIN, ANGMAX, MU_ANGMIN, MU_ANGMAX] = idx_brch;
%%
Sf = sqrt(results_pf.branch(:,PF).^2 + results_pf.branch(:,QF).^2);
St = sqrt(results_pf.branch(:,PT).^2 + results_pf.branch(:,QT).^2);
%maximum apparent flow on branch in per-unit
S  = max(Sf,St) / results_pf.baseMVA;
% line ratings in per-unit
r  = results_pf.branch(:,RATE_A) / results_pf.baseMVA;

R = results_pf.branch(:, BR_R); % branch resistance
X = results_pf.branch(:, BR_X); % branch reactance
%% find overloaded branches
% indices of overloaded branches
overloaded = find(S./r > overload_frac);

% sort overloades from LARGEST to SMALLEST
[~,tmp] = sort(S(overloaded)./r(overloaded), 'descend');
overloaded = overloaded(tmp);
Node_From = results_pf.branch(overloaded,1);
Node_To = results_pf.branch(overloaded,2);
end