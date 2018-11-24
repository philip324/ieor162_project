function [connection,edge_costs] = build_graph(VDCs,location,VDC_capacity)
connection = ones(numel(VDCs)) - eye(numel(VDCs));

syms x;
eqn = (200+4*x)/10 == (2000+3*x)/20;
th = solve(eqn,x);

edge_costs = zeros(numel(VDCs));
for i = 1:numel(VDCs)-1
    disp(i);
    for j = i+1:numel(VDCs)
        start_loc = get_location(VDCs{i},location);
        finish_loc = get_location(VDCs{j},location);
        rail = rail_available(VDCs{i},VDCs{j},VDC_capacity);
        dist = road_dist(start_loc,finish_loc);
        if rail && dist > th
            % rail
            edge_costs(i,j) = (dist*3 + 2000)/20;
        else
            % truck
            edge_costs(i,j) = (dist*4 + 200)/10;
        end
    end
end
edge_costs = edge_costs + rot90(fliplr(edge_costs));
end