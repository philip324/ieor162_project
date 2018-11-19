function costs = edgeCosts(VDCs,location)
%takes a set of nodes and maps out the paths between them
VDC_loc_idx = @(s)find(cellfun(@(x)isequal(x,s), location(1:end,1)));

key_name = VDCs;
val_loc = cell(length(VDC),1);
for i = 1:length(VDC)
    v = VDCs{i};
    lat = location{VDC_loc_idx(v),3};
    long = location{VDC_loc_idx(v),4};
    loc = {[lat, long]};
    val_loc(i) = {loc};
end
VDC2loc = containers.Map(key_name,val_loc);

%then figure out what transportation methods exist between the start_node
%and end node
available = transAvailable(node1, node2, mode);

%then calculate the distance between each vertex


%then calculate the cost of travelling that distance


%repeat this for every edge in the network


%return a cell containing start, end, cost for each pair


end