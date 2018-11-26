function arrival_time_map = arrTime2vehInfo(VDCs,shipment_req,...
    plant_arrival_time,dealer2VDC,shortest_routes,trans_modes)
% arrival_time_map was saved to processed_data.mat
arrival_time_map = containers.Map('KeyType','double','ValueType','any');
for i = 1:length(shipment_req)
    key = plant_arrival_time(i);
    
    vid = shipment_req{i,1};
    arrive_time = plant_arrival_time(i);
    modes = cell(1);
    
    plant = shipment_req{i,2};
    dealer = shipment_req{i,3};
    finalVDC = dealer2VDC(dealer);
    if strcmp(plant,finalVDC)
        path = {finalVDC};
        modes = cell(0);
    else
        path = shortest_routes([plant,' ',finalVDC]);
        for j = 1:length(path)-1
            mode = trans_modes(find(ismember(VDCs,path(j)),1), ...
                               find(ismember(VDCs,path(j+1)),1));
            modes(j) = {mode};
        end
    end
    path(end+1) = {dealer};
    modes(end+1) = {'T'};
    new_val = {vid,arrive_time,path,modes};
    if ~isKey(arrival_time_map, key)
        arrival_time_map(key) = new_val;
    else
        val = arrival_time_map(key);
        next_idx = size(val,1)+1;
        val(next_idx,:) = new_val;
        arrival_time_map(key) = val;
    end
    
    if mod(i,1e5) == 0
        disp(['iteration ',num2str(i)]);
        disp(['Processing time: ',num2str(round(toc,2)),' sec']);
        disp(' ');
        tic
    end
end
end