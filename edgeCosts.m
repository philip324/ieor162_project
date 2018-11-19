function costs = edgeCosts(VDCs, location)
av_miles_hr = 30;
t_miles_hr = 30;
r_miles_hr = 10;
av_fixed_cost = 0;
t_fixed_cost = 200;
t_variable_cost = 4;
r_fixed_cost = 2000;
r_variable_cost = 3;

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



%then calculate the distance between each vertex
distSet = []
num_edge = 44*44;
key_name_dist = cell(1,num_edge);
    for k = 1:length(num_edge)
        key_name_dist{k} = k
    end

    for i = 1:length(key_name)
        for j = 1:length(key_name)
            temp = VDC2loc(VDCs{i}); 
            temp2 = VDC2loc(VDCs{j});
            distance = road_dist(temp{1}(1), temp{1}(2), temp2{1}(1), temp2{1}(2));
            distSet = [distSet, distance];
        end
    end
M = containers.Map(key_name_dist,distSet)    

%then calculate the cost of travelling that distance
%repeat this for every edge in the network

modes = nan(1,2);
modes(1) = distances(i)*t_variable_cost + t_fixed_cost; %truck cost ($)
modes(2) = distance(i)*r_variable_cost + r_fixed_cost; %rails cost ($)
minmodescost = min(modes);

%repeat this for every edge in the network


%return a cell containing start, end, cost for each pair

end
