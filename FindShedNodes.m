function [Shed_nodes, idx] = FindShedNodes(results_pf,mpc_new1)
[overloaded,Node_From,Node_To] = FindOverLoadBranch(results_pf);
if ~isempty(overloaded)
    temp1_1 = find(mpc_new1.branch(:,1) == Node_From(1));
    temp1_2 = find(mpc_new1.branch(:,1) == Node_To(1));
    temp1_1 = mpc_new1.branch(temp1_1,2);
    temp1_2 = mpc_new1.branch(temp1_2,2);
    
    temp2_1 = find(mpc_new1.branch(:,2) == Node_From(1));
    temp2_2 = find(mpc_new1.branch(:,2) == Node_To(1));
    temp2_1 = mpc_new1.branch(temp2_1,1);
    temp2_2 = mpc_new1.branch(temp2_2,1);
    
    Shed_nodes = [temp1_1;temp1_2;temp2_1;temp2_2];
    Shed_nodes = unique(Shed_nodes);
    
    [tf,idx] = ismember(Shed_nodes,mpc_new1.gen(:,1));
    Shed_nodes(tf) = [];
    
    [tf,idx] = ismember(Shed_nodes,mpc_new1.bus(:,1));
else
    Shed_nodes = [];
    idx = [];
end
end
