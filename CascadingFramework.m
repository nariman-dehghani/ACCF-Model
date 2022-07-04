function [P_Shed,P_Served,Is_Converged,Failure_Branch,MPC] = CascadingFramework(mpc,mpopt)
define_constants;
[Islands,Isolated] = find_islands(mpc); %find islands and isolated nodes
mpc_array = extract_islands(mpc);
Failure_Branch = [];
Is_Converged = zeros(length(Islands),1);

for i = 1:length(Islands)
    mpc_new = mpc_array{i};
    ng = size(mpc_new.gen,1);
    
    if isempty(mpc_new.gen)
        P_Shed(i) = sum(mpc_new.bus(:,PD)); % amount of real power outage
        P_Served(i) = 0; % amount of real power remians in the system   
        Is_Converged(i) = 1;
        
    elseif ng == 1
        D_i = sum(mpc_new.bus(:,PD).^2); % Demand based on only real power
        S_i = sum(mpc_new.gen(:,PMAX).^2); % Capacity based on only real power
        if D_i < S_i
            P_Shed(i) = 0;
            P_Served(i) = sum(mpc_new.bus(:,PD));
        else
            Diff_P = sum(mpc_new.bus(:,PD)) - mpc_new.gen(:,PMAX);
            Diff_P(Diff_P < 0) = 0;
            P_Shed(i) = sum(Diff_P);
            P_Served(i) = sum(mpc_new.gen(:,PMAX));
        end
        
        Is_Converged(i) = 1;
        
    else
        if ~ismember(REF,mpc_new.bus(:,BUS_TYPE)) % if REF is not exist in this island do the following (Assign a slack bus if neccessary)
            %%% randomly select a gen as a reference and updated the bus types in Bus Type %%%
            temp_gen = mpc_new.gen(:,1);
            RAND_i = randi(size(mpc_new.gen,1));
            temp_gen(RAND_i) = [];
            
            IND = mpc_new.gen(RAND_i,1);
            INDEX = find(mpc_new.bus(:,1) == IND);
            
            INDEX1 = [];
            for IND1 = 1:length(temp_gen)
                INDEX1(IND1) = find(mpc_new.bus(:,BUS_I) == temp_gen(IND1));
            end
            
            mpc_new.bus(INDEX,BUS_TYPE) = REF;
            mpc_new.bus(INDEX1,BUS_TYPE) = PV;
        end
        
        results_opf = runopf(mpc_new,mpopt);
        results_pf = runpf(mpc_new,mpopt);
        [overloaded,Node_From,Node_To] = FindOverLoadBranch(results_pf);
        
        if results_opf.success == 1 % This 'if and else' tries to see whether opf converges or not.
            mpc_temp = mpc_array{i};
            
            P_Shed(i) = sum(mpc_temp.bus(:,PD)) - sum(mpc_new.bus(:,PD));
            P_Served(i) = sum(mpc_new.bus(:,PD));
            
            Is_Converged(i) = 1;
        else
            mpopt_new = mpoption(mpopt, 'opf.ac.solver', 'MIPS', 'mips.step_control', 1,'mips.sc.red_it', 100, 'mips.max_it', 500);
            results_opf = runopf(mpc_new,mpopt_new);
            
            if results_opf.success == 1
                disp('OPF converged after changing options of the solver!')
                mpc_temp = mpc_array{i};
                
                P_Shed(i) = sum(mpc_temp.bus(:,PD)) - sum(mpc_new.bus(:,PD));
                P_Served(i) = sum(mpc_new.bus(:,PD));              
                Is_Converged(i) = 1;
            end
        end
        
        if  results_opf.success == 0   % After the previous steps, if still opf does not converge, algorithm goes into this 'if statement'
            if ~isempty(overloaded)
                Failure_Branch = [overloaded(1),Node_From(1),Node_To(1)];
                
                [Shed_nodes,idx_Shed_nodes] = FindShedNodes(results_pf,mpc_new);
                
                Initial_Load_P = mpc_new.bus(idx_Shed_nodes,PD);
                Initial_Load_Q = mpc_new.bus(idx_Shed_nodes,QD);
                Initial_Load_S = sqrt(Initial_Load_P.^2 + Initial_Load_Q.^2);
                PowerFactor = Initial_Load_P./Initial_Load_S; %power factor
                PowerFactor(isnan(PowerFactor)) = 0;
                for II = 1:20
                    loadshed_factor = 0.05*II;
                    mpc_new.bus(idx_Shed_nodes,PD) = Initial_Load_P.*(1-loadshed_factor);
                    temp_QD = ((Initial_Load_P.*(1-loadshed_factor))./PowerFactor).*sqrt(1-PowerFactor.^2);
                    temp_QD(isnan(temp_QD)) = 0;
                    mpc_new.bus(idx_Shed_nodes,QD) = temp_QD;
                    
                    mpopt_new = mpoption(mpopt, 'opf.ac.solver', 'MIPS', 'mips.step_control', 1,'mips.sc.red_it', 100, 'mips.max_it', 500);
                    results_opf = runopf(mpc_new,mpopt_new);
                    results_pf = runpf(mpc_new,mpopt);
                    [overloaded,~,~] = FindOverLoadBranch(results_pf);
                    
                    if results_opf.success == 1
                        break
                    end
                    
                end
                if results_opf.success == 1 || isempty(overloaded) 
                    mpc_temp = mpc_array{i};
                    
                    P_Shed(i) = sum(mpc_temp.bus(:,PD)) - sum(mpc_new.bus(:,PD));
                    P_Served(i) = sum(mpc_new.bus(:,PD));                
                    Is_Converged(i) = 1;
                    
                else
                    results_pf = runpf(mpc_new,mpopt);
                    [overloaded,Node_From,Node_To] = FindOverLoadBranch(results_pf);
                    Failure_Branch = [overloaded(1),Node_From(1),Node_To(1)];
                    Is_Converged(i) = 0;
                    
                    P_Shed(i) = nan;
                    P_Served(i) = nan;
                end
            else
                Failure_Branch = [];
                mpc_temp = mpc_array{i};
                P_Shed(i) = sum(mpc_temp.bus(:,PD)) - sum(mpc_new.bus(:,PD));
                P_Served(i) = sum(mpc_new.bus(:,PD));            
                Is_Converged(i) = 1;
            end
        end
        
    end
    MPC{i} = mpc_new;
end

II = 0;
for i = (length(Islands)+1):(length(Islands)+length(Isolated))
    II = II + 1;
    MPC{i} = Isolated(II);
    ind_bus_isolated = Isolated(II);
    
    if ismember(mpc.bus(Isolated(II),BUS_I),mpc.gen(:,GEN_BUS))
        D_isolated = mpc.bus(ind_bus_isolated,PD).^2; % Demand based on only real power 
        ind_gen_isolated = find(mpc.gen(:,GEN_BUS) == mpc.bus(Isolated(II),BUS_I));
        S_isolated = mpc.gen(ind_gen_isolated,PMAX).^2;
        
        if D_isolated < S_isolated
            P_Shed(i) = 0;
            P_Served(i) = mpc.bus(ind_bus_isolated,PD);
        else
            P_Shed(i) = mpc.bus(ind_bus_isolated,PD) - mpc.gen(ind_gen_isolated,PMAX);
            P_Served(i) = mpc.gen(ind_gen_isolated,PMAX);
        end
        
    else
        P_Shed(i) = mpc.bus(ind_bus_isolated,PD);
        P_Served(i) = 0; % amount of real power remians in the system
    end
    
    Is_Converged(i) = 1;
end
P_Shed = P_Shed';
P_Served = P_Served';
end
