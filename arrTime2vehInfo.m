function arrival_time_map=arrTime2vehInfo(shipment_req,plant_arrival_time)
% arrival_time_map was saved to processed_data.mat
arrival_time_map = containers.Map('KeyType','double','ValueType','any');
for i = 1:length(shipment_req)
    key = plant_arrival_time(i);
    if ~isKey(arrival_time_map, key)
        arrival_time_map(key) = shipment_req(i,1:3);
    else
        val = arrival_time_map(key);
        next_idx = size(val,1)+1;
        val(next_idx,:) = shipment_req(i,1:3);
        arrival_time_map(key) = val;
    end
end
end