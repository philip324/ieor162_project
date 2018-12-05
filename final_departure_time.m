function all_departures = final_departure_time(routing_map,timeline)
all_departures = containers.Map('KeyType','double','ValueType','any');
ks = keys(routing_map);

for i = 1:length(routing_map)
    info = routing_map(ks{i});
    depart_time = info{end-1,3};
    if ~ismember(depart_time,timeline)
        idx = find(timeline < depart_time,1,'last');
        if length(idx) == 1
            timeline = [timeline(1:idx), depart_time, timeline(idx+1:end)];
        else
            timeline = [depart_time, timeline];
        end
    end
    
    depart_vdc = info{end-1,1};
    if ~isKey(all_departures,depart_time)
        all_departures(depart_time) = {depart_vdc,1};
    else
        val = all_departures(depart_time);
        if ~ismember(depart_vdc,val(:,1))
            val(end+1,:) = {depart_vdc,1};
            all_departures(depart_time) = val;
        else
            idx = find(ismember(val(:,1),depart_vdc),1);
            val(idx,2) = {val{idx,2} + 1};
            all_departures(depart_time) = val;
        end
    end
end
end