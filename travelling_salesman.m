function tour = travelling_salesman(final_VDC,dealers,location)
% final_VDC is a string, dealers is an array
% use nearest neighbor heuristic to get an initial tour
tour = cell(1,1+length(dealers));
tour(1) = {final_VDC};
idx = 1;
while ~isempty(dealers)
    loc1 = get_location(tour{idx},location);
    
    min_dist = Inf;
    min_idx = 0;
    for i = 1:length(dealers)
        loc2 = get_location(dealers(i),location);
        dist = road_dist(loc1,loc2);
        if dist < min_dist
            min_dist = dist;
            min_idx = i;
        end
    end
    idx = idx + 1;
    tour(idx) = {dealers(min_idx)};
    dealers(min_idx) = [];
end
if length(tour) <= 3
    return;
end

% use 2-opt heuristic to improve
temp_tour = [1, cell2mat(tour(2:end)), 1];
new_tour = [0, cell2mat(tour(2:end)), 0];
while ~all(temp_tour == new_tour)
    temp_tour = new_tour;
    for s = 1:length(temp_tour)-3
        for t = s+3:length(temp_tour)
            if s == 1
                start = final_VDC;
            else
                start = temp_tour(s);
            end
            if t == length(temp_tour)
                terminal = final_VDC;
            else
                terminal = temp_tour(t);
            end
            
            loc1 = get_location(start,location);
            loc2 = get_location(temp_tour(s+1),location);
            loc3 = get_location(temp_tour(t-1),location);
            loc4 = get_location(terminal,location);
            
            if ischar(terminal)
                original_dist = road_dist(loc1,loc2);
                new_dist = road_dist(loc1,loc3);
            else
                original_dist = road_dist(loc1,loc2) + road_dist(loc3,loc4);
                new_dist = road_dist(loc1,loc3) + road_dist(loc2,loc4);
            end
            if new_dist < original_dist
                new_tour = [temp_tour(1:s),temp_tour(t-1:-1:s+1),temp_tour(t:end)];
                break;
            end
        end
        if ~all(temp_tour == new_tour)
            break;
        end
    end
end
tour(2:end) = num2cell(new_tour(2:end-1));
end