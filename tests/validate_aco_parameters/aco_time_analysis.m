data = readtable('aco_all_data.csv');
data.escaped_percent = data.agents_escaped./data.num_agents;
% Group by param1 and param3
groupVars = {'graph_type', 'num_nodes', 'alpha', 'beta', 'num_ants', 'evaporation_rate', 'num_iterations'};
G = findgroups(data(:, groupVars));

% Compute mean elapsedTime for each group
meanTimes = splitapply(@mean, data.simulation_time, G);
meanAgents = splitapply(@mean, data.escaped_percent, G);

% Optionally, get unique combinations of the grouping parameters
groupKeys = unique(data(:, groupVars));
resultTable = [groupKeys, table(meanTimes), table(meanAgents)];
resultTable.groupCounts = splitapply(@numel, data.simulation_time, G);