function costs = vertexedgecosts(map)
%takes a set of nodes and maps out the paths between them

%start by mapping out where the vertices are


%then figure out what transportation methods exist between the start_node
%and end node


%then calculate the distance between each vertex


%then calculate the cost of travelling that distance


%repeat this for every edge in the network


%return a cell containing start, end, cost for each pair

end

function route = djikstra(start_node, end_node, map)
%takes a cell containing  nodes and costs and returns the shortest
%path between a start and end node in the set

%load the cell from the above function


%run djikstra's algorithm from the start node to the end node

%return the resulting shortest path as an array where each entry is the
%node traversed

end
