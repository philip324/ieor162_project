function tour = three_opt(final_VDC,dealers,location)
if 1+length(dealers) < 6
    tour = two_opt(final_VDC,dealers,location);
    return;
end

tour = nearest_neighbor(final_VDC,dealers,location);
new_tour = [0, cell2mat(tour(2:end)), 0];
flag = 1;
while flag
    flag = 0;
    for i = 1:length(tour)
        for j = i+2:length(tour)
            for k = j+2:length(tour)+(i>1)-1
                % define A,B,C,D,E,F
                if i == 1
                    A = final_VDC;
                else
                    A = new_tour(i);
                end
                B = new_tour(i+1); C = new_tour(j); D = new_tour(j+1); E = new_tour(k);
                if k+1 == length(new_tour)
                    F = final_VDC;
                else
                    F = new_tour(k+1);
                end
                locA = get_location(A,location);
                locB = get_location(B,location);
                locC = get_location(C,location);
                locD = get_location(D,location);
                locE = get_location(E,location);
                locF = get_location(F,location);
                
                if ischar(F)
                    % 1 original dist
                    d0 = road_dist(locA,locB) + road_dist(locC,locD);
                    % 3 two-opt dist
                    d1 = road_dist(locA,locB) + road_dist(locC,locE);
                    d2 = road_dist(locA,locE) + road_dist(locC,locD);
                    d3 = road_dist(locA,locC) + road_dist(locB,locD);
                    % 4 three-opt dist
                    d4 = road_dist(locA,locE) + road_dist(locB,locD);
                    d5 = road_dist(locA,locC) + road_dist(locB,locE);
                    d6 = road_dist(locA,locD) + road_dist(locC,locE);
                    d7 = road_dist(locA,locD) + road_dist(locB,locE);
                else
                    % 1 original dist
                    d0 = road_dist(locA,locB) + road_dist(locC,locD) + road_dist(locE,locF);
                    % 3 two-opt dist
                    d1 = road_dist(locA,locB) + road_dist(locC,locE) + road_dist(locD,locF);
                    d2 = road_dist(locA,locE) + road_dist(locC,locD) + road_dist(locB,locF);
                    d3 = road_dist(locA,locC) + road_dist(locB,locD) + road_dist(locE,locF);
                    % 4 three-opt dist
                    d4 = road_dist(locA,locE) + road_dist(locB,locD) + road_dist(locC,locF);
                    d5 = road_dist(locA,locC) + road_dist(locB,locE) + road_dist(locD,locF);
                    d6 = road_dist(locA,locD) + road_dist(locC,locE) + road_dist(locB,locF);
                    d7 = road_dist(locA,locD) + road_dist(locB,locE) + road_dist(locC,locF);
                    
                    if ischar(A)
                        temp_loc = get_location(new_tour(end-1),location);
                        rd1 = d1 - road_dist(locA,locB) + road_dist(locA,temp_loc);
                        rd2 = d2 - road_dist(locA,locE) + road_dist(locA,temp_loc);
                        rd3 = d3 - road_dist(locA,locC) + road_dist(locA,temp_loc);
                        rd4 = d4 - road_dist(locA,locE) + road_dist(locA,temp_loc);
                        rd5 = d5 - road_dist(locA,locC) + road_dist(locA,temp_loc);
                        rd6 = d6 - road_dist(locA,locD) + road_dist(locA,temp_loc);
                        rd7 = d7 - road_dist(locA,locD) + road_dist(locA,temp_loc);
                        if rd1 < d0 && rd1 < d1
                            new_tour = [new_tour(1:j),new_tour(k:-1:j+1),new_tour(k+1:end)];
                            new_tour = fliplr(new_tour);
                            flag = 1;
                            break
                        elseif rd2 < d0 && rd2 < d2
                            new_tour = [new_tour(1:i),new_tour(k:-1:i+1),new_tour(k+1:end)];
                            new_tour = fliplr(new_tour);
                            flag = 1;
                            break
                        elseif rd3 < d0 && rd3 < d3
                            new_tour = [new_tour(1:i),new_tour(j:-1:i+1),new_tour(j+1:end)];
                            new_tour = fliplr(new_tour);
                            flag = 1;
                            break
                        elseif rd4 < d0 && rd4 < d4
                            tmp = [new_tour(k:-1:j+1),new_tour(i+1:j)];
                            new_tour = [new_tour(1:i),tmp,new_tour(k+1:end)];
                            new_tour = fliplr(new_tour);
                            flag = 1;
                            break
                        elseif rd5 < d0 && rd5 < d5
                            tmp = [new_tour(j:-1:i+1),new_tour(k:-1:j+1)];
                            new_tour = [new_tour(1:i),tmp,new_tour(k+1:end)];
                            new_tour = fliplr(new_tour);
                            flag = 1;
                            break
                        elseif rd6 < d0 && rd6 < d6
                            tmp = [new_tour(j+1:k),new_tour(j:-1:i+1)];
                            new_tour = [new_tour(1:i),tmp,new_tour(k+1:end)];
                            new_tour = fliplr(new_tour);
                            flag = 1;
                            break
                        elseif rd7 < d0 && rd7 < d7
                            tmp = [new_tour(j+1:k),new_tour(i+1:j)];
                            new_tour = [new_tour(1:i),tmp,new_tour(k+1:end)];
                            new_tour = fliplr(new_tour);
                            flag = 1;
                            break
                        end
                        
                    end
                end
                
                if d1 < d0
                    new_tour = [new_tour(1:j),new_tour(k:-1:j+1),new_tour(k+1:end)];
                    flag = 1;
                    break
                elseif d2 < d0
                    new_tour = [new_tour(1:i),new_tour(k:-1:i+1),new_tour(k+1:end)];
                    flag = 1;
                    break
                elseif d3 < d0
                    new_tour = [new_tour(1:i),new_tour(j:-1:i+1),new_tour(j+1:end)];
                    flag = 1;
                    break
                elseif d4 < d0
                    tmp = [new_tour(k:-1:j+1),new_tour(i+1:j)];
                    new_tour = [new_tour(1:i),tmp,new_tour(k+1:end)];
                    flag = 1;
                    break
                elseif d5 < d0
                    tmp = [new_tour(j:-1:i+1),new_tour(k:-1:j+1)];
                    new_tour = [new_tour(1:i),tmp,new_tour(k+1:end)];
                    flag = 1;
                    break
                elseif d6 < d0
                    tmp = [new_tour(j+1:k),new_tour(j:-1:i+1)];
                    new_tour = [new_tour(1:i),tmp,new_tour(k+1:end)];
                    flag = 1;
                    break
                elseif d7 < d0
                    tmp = [new_tour(j+1:k),new_tour(i+1:j)];
                    new_tour = [new_tour(1:i),tmp,new_tour(k+1:end)];
                    flag = 1;
                    break
                end
            end
            if flag
                break
            end
        end
        if flag
            break
        end
    end
end
tour(2:end) = num2cell(new_tour(2:end-1));
end