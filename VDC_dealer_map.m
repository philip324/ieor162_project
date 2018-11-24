function [dealer2VDC,VDC2dealer] = VDC_dealer_map(location,dealers,VDCs)
dealer2VDC = containers.Map('KeyType','double','ValueType','any');
VDC2dealer = containers.Map('KeyType','char','ValueType','any');
tic
for i = 1:numel(dealers)
    d = dealers{i};
    d_loc = get_location(d,location);
    min_dist = inf;
    closest_VDC = '';
    for j = 1:numel(VDCs)
        v = VDCs{j};
        v_loc = get_location(v,location);
        dist = road_dist(d_loc(1),d_loc(2),v_loc(1),v_loc(2));
        if dist < min_dist
            min_dist = dist;
            closest_VDC = v;
        end
    end
    
    dealer2VDC(d) = closest_VDC;
    if ~isKey(VDC2dealer,closest_VDC)
        VDC2dealer(closest_VDC) = {d};
    else
        val = VDC2dealer(closest_VDC);
        val{:} = [val{:}, d];
        VDC2dealer(closest_VDC) = val;
    end
    
    if mod(i,50) == 0
        disp(['iteration ',num2str(i)]);
        disp(['Processing time: ',num2str(round(toc,2)),' sec']);
        disp(' ');
        tic
    end
end
end