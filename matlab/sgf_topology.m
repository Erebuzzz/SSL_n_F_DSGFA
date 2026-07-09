function adjacency = sgf_topology(topology, n)
%SGF_TOPOLOGY Resolve a shared-config topology into a MATLAB adjacency matrix.

if isfield(topology, 'adjacency') && ~isempty(topology.adjacency)
    adjacency = double(topology.adjacency);
    return
end

if isfield(topology, 'edges') && ~isempty(topology.edges)
    adjacency = adjacency_from_edges(n, double(topology.edges));
    return
end

name = string(topology.name);
switch name
    case "paper_fig1_reconstructed"
        if n ~= 6
            error('paper_fig1_reconstructed is only defined for n = 6.');
        end
        edges = [0 1; 1 2; 2 3; 3 4; 4 5; 5 0; 0 2; 1 4];
    case "default"
        edges = [(0:(n - 1))', [1:(n - 1), 0]'];
        if n >= 6
            edges = [edges; 0 3; 1 4; 2 5];
        end
    case "ring"
        edges = [(0:(n - 1))', [1:(n - 1), 0]'];
    case "complete"
        adjacency = ones(n, n) - eye(n);
        return
    otherwise
        error('Unknown topology: %s', name);
end

adjacency = adjacency_from_edges(n, edges);
end

function adjacency = adjacency_from_edges(n, zero_based_edges)
adjacency = zeros(n, n);
for row = 1:size(zero_based_edges, 1)
    i = zero_based_edges(row, 1) + 1;
    j = zero_based_edges(row, 2) + 1;
    adjacency(i, j) = 1;
    adjacency(j, i) = 1;
end
end
