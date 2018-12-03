function [tour,total_dist] = two_opt(final_VDC,dealers,location)
[tour,total_dist] = nearest_neighbor(final_VDC,dealers,location);
if length(tour) < 4
    return
end

% use 2-opt heuristic to improve
new_tour = [0, cell2mat(tour(2:end)), 0];
flag = 1;
while flag
    flag = 0;
    for i = 1:length(tour)
        for j = i+2:length(tour)+(i>1)-1
            if i == 1
                A = final_VDC;
            else
                A = new_tour(i);
            end
            B = new_tour(i+1); C = new_tour(j);
            if j+1 == length(new_tour)
                D = final_VDC;
            else
                D = new_tour(j+1);
            end
            
            locA = get_location(A,location);
            locB = get_location(B,location);
            locC = get_location(C,location);
            locD = get_location(D,location);
            
            if ischar(D)
                d0 = road_dist(locA,locB);
                d1 = road_dist(locA,locC);
            else
                d0 = road_dist(locA,locB) + road_dist(locC,locD);
                d1 = road_dist(locA,locC) + road_dist(locB,locD);
                
                if ischar(A)
                    temp_loc = get_location(new_tour(end-1),location);
                    rd1 = d1 - road_dist(locA,locC) + road_dist(locA,temp_loc);
                    if rd1 < d0 && rd1 < d1
                        new_tour = [new_tour(1:i),new_tour(j:-1:i+1),new_tour(j+1:end)];
                        new_tour = fliplr(new_tour);
                        flag = 1;
                        break
                    end
                end
            end
            
            if d1 < d0
                new_tour = [new_tour(1:i),new_tour(j:-1:i+1),new_tour(j+1:end)];
                flag = 1;
                break
            end
        end
        if flag
            break
        end
    end
end
tour(2:end) = num2cell(new_tour(2:end-1));
total_dist = 0;
for i = 2:length(tour)
    loc1 = get_location(tour{i-1},location);
    loc2 = get_location(tour{i},location);
    total_dist = total_dist + road_dist(loc1,loc2);
end
end