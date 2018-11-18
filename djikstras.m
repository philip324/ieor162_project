function visual = vertexedge(map)
%takes a set of nodes and maps out the paths between them

%start by mapping out where the vertices are


%then calculate the distance between each vertex


%then figure out what transportation methods exist between the start_node
%and end node


%then calculate the cost of traversing that edge

%do so for every edge & vertex in the network


%return a visual showing the vertices, edges, and edge lengths

end

function route = djikstra(start_node, end_node, map)
%takes a set of paths, start node, and end node, and returns the shortest
%path between them

%load the visual/data from the above map


%run djikstra's algorithm from the start node to the end node


end
