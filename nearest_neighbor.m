function [tour,total_dist] = nearest_neighbor(final_VDC,dealers,location)
tour = cell(1,1+length(dealers));
tour(1) = {final_VDC};
idx = 1;
total_dist = 0;
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
    total_dist = total_dist + min_dist;
end
end