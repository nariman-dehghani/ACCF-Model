clear
clc
warning off
%% Power System Info %%
F_BUS = 1; % "from" bus number
T_BUS = 2; % "To" bus number
BR_STATUS = 11; % initial branch status, 1 = in-service, 0 = out-of-service
PD = 3; %real power demand (MW)
QD = 4; %Reactive power demand (MVA)
mpc = loadcase('case30'); %https://matpower.org/docs/ref/matpower5.0/caseformat.html
mpopt = mpoption('pf.alg', 'NR', 'pf.nr.max_it', 10,'verbose',0,'out.all',0);
%% Faulty Branches %%
nl = length(mpc.branch); % Number of lines in the system
q = 2^-3; % Probability of failure of lines

Uniform_Rand = rand(nl,1);
Logic = Uniform_Rand <= ones(nl,1).*q ;
k = find(Logic == 1); % k indicates index of faulty lines
k = k';
From_nodes_k = mpc.branch(k,F_BUS);
To_nodes_k = mpc.branch(k,T_BUS);

%% ACCF Model %%
tic
if isempty(k)
    P_shed = 0;
    P_served = sum(mpc.bus(:,PD));
else
    
    BRANCH_FAILURES = [];
    FROM_NODES = [];
    TO_NODES = [];
    
    Is_Converged = 0;
    BRANCH_FAILURES = k;
    FROM_NODES = From_nodes_k;
    TO_NODES = To_nodes_k;
    Failure_Branch = [];
    Node_From = [];
    Node_To = [];
    
    II = 0;
    while ~all(Is_Converged)
        mpc1 = mpc;
        BRANCH_FAILURES = [BRANCH_FAILURES,Failure_Branch'];
        FROM_NODES = [FROM_NODES;Node_From];
        TO_NODES = [TO_NODES;Node_To];
        
        mpc1.branch(BRANCH_FAILURES,BR_STATUS) = 0;
        [P_shed,P_served,Is_Converged,Branch_temp,MPC] = CascadingFramework(mpc1,mpopt);
        if isempty(Branch_temp)
            Failure_Branch = [];
            Node_From = [];
            Node_To = [];
        else
            Failure_Branch = find(mpc.branch(:,F_BUS) == Branch_temp(2) & mpc.branch(:,T_BUS) == Branch_temp(3));
            Node_From = Branch_temp(2);
            Node_To = Branch_temp(3);
        end
        II = II + 1;
        if II > nl
            disp('Iterations exceeded nl!')
            break
        end
    end
end
toc
%% Load Shedding Ratio (Real Power) %%
total_P_shed = sum(P_shed);
P_loss =  total_P_shed / sum( mpc.bus(:,PD) )

