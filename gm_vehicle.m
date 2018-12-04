%% IEOR 162, Project

%% Preparation
% Put Problem_VehicleShipmentRequirement.csv in ./final_problem/ directory
homedir = pwd;
new_dir = sprintf('%s/final_problem', homedir);
if ~exist(new_dir, 'dir')
    mkdir(new_dir);
end

%% load data
clear;
close all;
clc

tic
load arrival_time_map.mat
sorted_plant_arrival_time = sort(cell2mat(keys(arrival_time_map)));

load processed_data.mat
input_data = 'Input_Cost%2C+Location.xlsx';
dataset_1 = 'DataSet1/VehicleShipmentRequirement_DataSet1.csv';
dataset_2 = 'DataSet2/VehicleShipmentRequirement_DataSet2.csv';
final_prob = 'final_problem/Problem_VehicleShipmentRequirement.csv';

[~,~,location] = xlsread(input_data,1);
[~,~,VDC_capacity] = xlsread(input_data,2);
[~,~,VDC_cost_model] = xlsread(input_data,3);
[~,~,trans_cost_model] = xlsread(input_data,4);

location(1,:) = [];
VDC_capacity(1,:) = [];
VDC_cost_model = VDC_cost_model(1:4,:);
for i = 1:size(VDC_cost_model,1)
    for j = 1:size(VDC_cost_model,2)
        if isnan(VDC_cost_model{i,j})
            VDC_cost_model{i,j} = 0;
        end
    end
end
for i = 1:size(trans_cost_model,1)
    for j = 1:size(trans_cost_model,2)
        if isnan(trans_cost_model{i,j})
            trans_cost_model{i,j} = 0;
        end
    end
end

fid = fopen(final_prob);
% header = textscan(fid, '%q%q%q%q', 1, 'Delimiter', ',');
C = textscan(fid, '%q%q%f%q', 'Delimiter', ',', 'HeaderLines', 1);
fclose(fid);
row = numel(C{1});
col = numel(C);
shipment_req = cell(row,col);
% shipment_req(1,:) = [header{1},header{2},header{3},header{4}];
shipment_req(:,1) = C{1};
shipment_req(:,2) = C{2};
shipment_req(:,3) = num2cell(C{3});
shipment_req(:,4) = C{4};
disp(['Processing time: ',num2str(round(toc,2)),' sec']);

%% Find Shortest Paths
% Get lists of dealers and VDCs
dealers = num2cell(union(cell2mat(shipment_req(:,3)),shipment_req{1,3}));
VDCs = VDC_capacity(:,1);
plants = union(shipment_req(:,2),shipment_req{1,2});
final_VDCs = keys(VDC2dealer)';

% Run dijkstra to find the static routes
SID = zeros(1,numel(plants));
for i = 1:numel(plants)
    SID(i) = find(ismember(VDCs,plants(i)),1);
end

FID = zeros(1,numel(final_VDCs));
for i = 1:numel(final_VDCs)
    FID(i) = find(ismember(VDCs,final_VDCs(i)),1);
end

tic
shortest_routes = containers.Map('KeyType','char','ValueType','any');
for i = 1:numel(plants)
    for j = 1:numel(final_VDCs)
        if SID(i) ~= FID(j)
            [~,path] = dijkstra(connection,edge_costs,SID(i),FID(j));
            val = cell(1,numel(path));
            for k = 1:numel(path)
                v = VDCs{path(k)};
                val(k) = {v};
            end
            shortest_routes([plants{i},' ',final_VDCs{j}]) = val;
        end
    end
end
disp(['Processing time: ',num2str(round(toc,2)),' sec']);

%% Initializing variables
% 'vdc' --> double
handled_vehicles = containers.Map('KeyType','char','ValueType','double');
overflow_vehicles = containers.Map('KeyType','char','ValueType','double');
overflow_days = containers.Map('KeyType','char','ValueType','double');
for i = 1:numel(VDCs)
    handled_vehicles(VDCs{i}) = 0;
    overflow_vehicles(VDCs{i}) = 0;
    overflow_days(VDCs{i}) = 0;
end

% 'vdc' --> (timestamp, inventory)
curr_inventory = containers.Map('KeyType','char','ValueType','any');
% 'vdc' --> (vid, arrive_time, path, modes)
final_arrival = containers.Map('KeyType','char','ValueType','any');
% 'vdc1 vdc2' --> (vid, arrive_time, path, modes)
pending_vehicles = containers.Map('KeyType','char','ValueType','any');
% 'vdc1 vdc2' --> double
edge_flows = containers.Map('KeyType','char','ValueType','double');
% 'vid' --> (loc, arrive_time, depart_time, depart_mode)
routing_map = containers.Map('KeyType','char','ValueType','any');

%% Choose a subset of the dataset
% Third quarter of 2015
start_date = datenum('01-Jul-2015 00:00:00');
stop_date = datenum('30-Sep-2015 23:59:59');
start = find(sorted_plant_arrival_time >= start_date, 1, 'first');
stop = find(sorted_plant_arrival_time <= stop_date, 1, 'last');
timeline = sorted_plant_arrival_time(start:stop);

tic
% arrival_time --> (vid, arrival_time, path, curr_vdc)
all_arrivals = containers.Map('KeyType','double','ValueType','any');
for i = 1:length(timeline)
    vdc_table = cell(0,2);
    t = timeline(i);
    vehicles = arrival_time_map(t);
    for j = 1:size(vehicles,1)
        veh = vehicles(j,:);
        path = veh{3};
        for k = 1:length(path)-1
            vdc = path{k};
            handled_vehicles(vdc) = handled_vehicles(vdc) + 1;
            if k < length(path)-1
                edge = [path{k},' ',path{k+1}];
                if ~isKey(edge_flows, edge)
                    edge_flows(edge) = 1;
                else
                    edge_flows(edge) = edge_flows(edge) + 1;
                end
            end
        end
        % add new fields to vehicle
        curr_VDC = path{1};
        new_veh = {veh{1},veh{2},veh{3},curr_VDC};
        if ~ismember(curr_VDC,vdc_table(:,1))
            vdc_table(end+1,:) = {curr_VDC, new_veh};
        else
            idx = find(ismember(vdc_table(:,1),{curr_VDC}),1);
            val = vdc_table{idx,2};
            val(end+1,:) = new_veh;
            vdc_table(idx,2) = {val};
        end
    end
    all_arrivals(t) = vdc_table;
end
disp(['Processing time: ',num2str(round(toc,2)),' sec']);

%% Find final VDC arrival time for each vehicle
tic
count = 0;
logistics_cost = 0;
while ~isempty(timeline)
    % get latest arrival time
    curr_time = timeline(1);
    timeline(1) = [];
    arrive_table = all_arrivals(curr_time);
    remove(all_arrivals, curr_time);
    for i = 1:size(arrive_table,1)
        curr_VDC = arrive_table{i,1};
        arriving_vehicles = arrive_table{i,2};
        % check if any vehicles need to be added urgently
        for j = 1:length(VDCs)
            prev_VDC = VDCs{j};
            if strcmp(prev_VDC,curr_VDC)
                continue
            end
            edge = [prev_VDC,' ',curr_VDC];
            if ~isKey(pending_vehicles,edge)
                continue
            end
            idx1 = find(ismember(VDCs,prev_VDC),1);
            idx2 = find(ismember(VDCs,curr_VDC),1);
            mode = trans_modes(idx1, idx2);
            if strcmp(mode,'T')
                load_factor = 10;
            elseif strcmp(mode,'R')
                load_factor = 20;
            end
            if size(pending_vehicles(edge),1) < load_factor
                continue
            end
            temp1 = curr_inventory(prev_VDC);
            temp2 = curr_inventory(curr_VDC);
            overflow1 = max(0,temp1{2}-get_capacity(prev_VDC,VDC_capacity));
            overflow2 = max(0,temp2{2}-get_capacity(curr_VDC,VDC_capacity));
            if overflow1 < overflow2
                continue
            end
            wait_list = pending_vehicles(edge);
            remain = mod(size(wait_list,1),load_factor);
            num_shipped = size(wait_list,1) - remain;
            new_vehicles = wait_list(1:num_shipped,:);
            if remain > 0
                pending_vehicles(edge) = wait_list(num_shipped+1:end,:);
            else
                remove(pending_vehicles,edge);
            end
            arriving_vehicles(end+1:end+num_shipped,:) = new_vehicles;
            curr_inventory(prev_VDC) = {curr_time,temp1{2}-num_shipped};
        end
        
        
        depart_table = cell(0,2);
        final = 0;
        % classify vehicles by their next destinations
        for j = 1:size(arriving_vehicles,1)
            arrive_veh = arriving_vehicles(j,:);
            path = arrive_veh{3};
            curr_idx = find(ismember(path(1:end-1),{curr_VDC}),1);
            % arrive to final VDC
            if strcmp(curr_VDC, path{end-1})
                final = final + 1;
                if ~isKey(final_arrival,curr_VDC)
                    final_arrival(curr_VDC) = arrive_veh;
                else
                    val = final_arrival(curr_VDC);
                    val(end+1,:) = arrive_veh;
                    final_arrival(curr_VDC) = val;
                end
                continue
            end
            % not final VDC
            next_VDC = path{curr_idx+1};
            next_destinations = depart_table(:,1);
            if ~ismember(next_VDC,next_destinations)
                depart_table(end+1,:) = {next_VDC, arrive_veh};
            else
                idx = find(ismember(next_destinations,{next_VDC}),1);
                val = depart_table{idx,2};
                val(end+1,:) = arrive_veh;
                depart_table(idx,2) = {val};
            end
        end
        
        % check if there exists a full load along some edges
        inflows = size(arriving_vehicles,1) - final;
        outflows = 0;
        for j = 1:size(depart_table,1)
            next_VDC = depart_table{j,1};
            departing_vehicles = depart_table{j,2};
            % determine transportation mode
            idx1 = find(ismember(VDCs,curr_VDC),1);
            idx2 = find(ismember(VDCs,next_VDC),1);
            mode = trans_modes(idx1, idx2);
            if strcmp(mode,'T')
                speed = 30;
                load_factor = 10;
                fixed_cost = 200;
                variable_cost = 4;
            elseif strcmp(mode,'R')
                speed = 10;
                load_factor = 20;
                fixed_cost = 2000;
                variable_cost = 3;
            end
            
            edge = [curr_VDC,' ',next_VDC];
            if ~isKey(pending_vehicles,edge)
                wait_list = departing_vehicles;
            else
                wait_list = pending_vehicles(edge);
                wait_list(end+1:end+size(departing_vehicles,1),:) = departing_vehicles;
            end
            wl_len = size(wait_list,1);
            edge_flows(edge) = edge_flows(edge) - size(departing_vehicles,1);
            full_load = (wl_len >= load_factor || edge_flows(edge) == 0);
            % not a full load
            if ~full_load
                pending_vehicles(edge) = wait_list;
                continue
            end
            
            % check if next destination will be overflowed
            if isKey(curr_inventory,curr_VDC) && isKey(curr_inventory,next_VDC)
                outflow = wl_len - mod(wl_len,load_factor) * (edge_flows(edge) > 0);
                temp1 = curr_inventory(curr_VDC);
                temp2 = curr_inventory(next_VDC);
                overflow1 = max(0,temp1{2}+inflows-get_capacity(curr_VDC,VDC_capacity));
                overflow2 = max(0,temp2{2}+outflow-get_capacity(next_VDC,VDC_capacity));
                if overflow1 < overflow2
                    pending_vehicles(edge) = wait_list;
                    if edge_flows(edge) == 0
                        remove(edge_flows,edge);
                    end
                    continue
                end
            end
            
            
            % full load
            if isKey(pending_vehicles,edge)
                remove(pending_vehicles,edge);
            end
            if edge_flows(edge) == 0
                remove(edge_flows,edge);
                outflow = wl_len;
                departing_vehicles = wait_list;
            else
                remain = mod(wl_len,load_factor);
                outflow = wl_len - remain;
                departing_vehicles = wait_list(1:outflow,:);
                if remain > 0
                    pending_vehicles(edge) = wait_list(outflow+1:end,:);
                end
            end
            outflows = outflows + outflow;
            
            % send departing vehicles to the next destination
            curr_loc = get_location(curr_VDC,location);
            next_loc = get_location(next_VDC,location);
            dist = road_dist(curr_loc,next_loc);
            next_time = curr_time + (dist/speed)/24;
            
            % VIN level routing details
            for k = 1:outflow
                depart_veh = departing_vehicles(k,:);
                vid = depart_veh{1};
                details = {curr_VDC, depart_veh{2}, curr_time, mode};
                if ~isKey(routing_map,vid)
                    routing_map(vid) = details;
                else
                    val = routing_map(vid);
                    val(end+1,:) = details;
                    routing_map(vid) = val;
                end
            end
            
            departing_vehicles(:,2) = num2cell(repmat(next_time,outflow,1));
            departing_vehicles(:,4) = cellstr(repmat(next_VDC,outflow,1));
            if ~isKey(all_arrivals, next_time)
                all_arrivals(next_time) = {next_VDC,departing_vehicles};
            else
                val = all_arrivals(next_time);
                val(end+1,:) = {next_VDC,departing_vehicles};
                all_arrivals(next_time) = val;
            end
            
            % add new arrival time to timeline
            if ~ismember(next_time,timeline)
                idx = find(timeline < next_time,1,'last');
                if length(idx) == 1
                    timeline = [timeline(1:idx), next_time, timeline(idx+1:end)];
                else
                    timeline = [next_time, timeline];
                end
            end
            
            % calculate the transportation cost
            trans_cost = (dist*variable_cost+fixed_cost)*ceil(outflow/load_factor);
            logistics_cost = logistics_cost + trans_cost;
        end
        
        % overflow information
        netflows = inflows - outflows;
        if ~isKey(curr_inventory,curr_VDC)
            prev_inventory = 0;
        else
            past = curr_inventory(curr_VDC);
            prev_inventory = past{2};
        end
        prev_overflow = prev_inventory - get_capacity(curr_VDC,VDC_capacity);
        curr_overflow = prev_inventory + netflows - get_capacity(curr_VDC,VDC_capacity);
        if ~isKey(overflow_vehicles,curr_VDC)
            overflow_vehicles(curr_VDC) = max(0,curr_overflow);
        else
            overflow_vehicles(curr_VDC) = overflow_vehicles(curr_VDC) + max(0,curr_overflow);
        end
        curr_inventory(curr_VDC) = {curr_time, prev_inventory+netflows};
        if prev_overflow > 0
            prev_time = past{1};
            duration = floor(curr_time-prev_time);
            overflow_days(curr_VDC) = overflow_days(curr_VDC) + duration*prev_overflow;
        end
    end
    
    count = count + 1;
    if mod(count,5e3) == 0
        disp(['iteration ',num2str(count)]);
        disp(['Processing time: ',num2str(round(toc,2)),' sec']);
        disp(' ');
        tic
    end
end
toc

save('intermediate_third_quarter_2015.mat','handled_vehicles','overflow_vehicles',...
     'overflow_days','final_arrival','routing_map','logistics_cost');

%% Last leg: distribute from final VDC to dealers
speed = 30;
load_factor = 10;
fixed_cost = 200;
variable_cost = 4;

look_ahead_time = 2;   % 2 days
tic
ks = keys(final_arrival);
for i = 1:length(ks)
    final_v = ks{i};
    vehicles = final_arrival(final_v);
    count = 0;
    disp([num2str(i),' VDC: ',final_v]);
    
    while ~isempty(vehicles)
        dealer_vehicle_map = containers.Map('KeyType','double','ValueType','any');
        path = vehicles{1,3};
        center_dealer = path{end};
        % find horizon
        horizon = 1;
        while horizon < size(vehicles,1) && vehicles{horizon,2} - vehicles{1,2} < look_ahead_time
            horizon = horizon + 1;
        end
        
        for j = 1:horizon
            veh = vehicles(j,:);
            path = veh{3};
            d = path{end};
            if ~isKey(dealer_vehicle_map,d)
                dealer_vehicle_map(d) = veh;
            else
                val = dealer_vehicle_map(d);
                val(end+1,:) = veh;
                dealer_vehicle_map(d) = val;
            end
        end
        delivery_map = forward_sweep(final_v,center_dealer,dealer_vehicle_map,location);
        selected_dealers = cell2mat(keys(delivery_map));
        [tour,total_dist] = three_opt(final_v,selected_dealers,location);
        
        % deliver vehicles
        shipped_vehicles = cell(0,size(vehicles,2));
        for j = 1:length(selected_dealers)
            shipped_veh = delivery_map(selected_dealers(j));
            shipped_vehicles(end+1:end+size(shipped_veh,1),:) = shipped_veh;
        end
        
        % deliver timetable
        curr_time = shipped_vehicles{end,2};
        delivered_time = containers.Map('KeyType','double','ValueType','double');
        for j = 2:length(tour)
            loc1 = get_location(tour{j-1},location);
            loc2 = get_location(tour{j},location);
            dist = road_dist(loc1,loc2);
            delivered_time(tour{j}) = curr_time + (dist/speed)/24;
            curr_time = delivered_time(tour{j});
        end
        
        for j = 1:size(shipped_vehicles,1)
            shipped_veh = shipped_vehicles(j,:);
            
            vid = shipped_veh{1};
            path = shipped_veh{3};
            d = path{end};
            details = {d, delivered_time(d), delivered_time(d), 'T'};
            if ~isKey(routing_map,vid)
                routing_map(vid) = details;
            else
                val = routing_map(vid);
                val(end+1,:) = details;
                routing_map(vid) = val;
            end
            
            idx = find(ismember(vehicles(:,1),shipped_veh(1)),1);
            vehicles(idx,:) = [];
        end
        
        % calculate transporation cost
        outflow = size(shipped_vehicles,1);
        trans_cost = (total_dist*variable_cost+fixed_cost) * ceil(outflow/load_factor);
        logistics_cost = logistics_cost + trans_cost;
        
        count = count + 1;
        if mod(count,50) == 0
            disp(['iteration ',num2str(count)]);
            disp(['Processing time: ',num2str(round(toc,2)),' sec']);
            disp(['Length of vehicles: ',num2str(size(vehicles,1))]);
            disp(' ');
            tic
        end
    end
end
toc

save('final_third_quarter_2015.mat','handled_vehicles','overflow_vehicles',...
     'overflow_days','final_arrival','routing_map','logistics_cost');

%% Calculate costs
% Calculate late penalty cost (10 dollars per day)
ks = keys(routing_map);
lead_time = containers.Map('KeyType','char','ValueType','double');
for i = 1:length(routing_map)
    info = routing_map(ks{i});
    duration = info{end,2} - info{1,2};
    lead_time(ks{i}) = duration;
end
late_cost = 10 * sum(cell2mat(values(lead_time)));

% Calculate VDC cost
annual_cost_vdc = 730/4;
handling_cost_vdc = 50;
overflow_shuttle_cost = 30;
overflow_variable_cost = 4;

vdc_cost = 0;
for i = 1:length(VDCs)
    v = VDCs{i};
    cost = annual_cost_vdc*get_capacity(v,VDC_capacity) + handling_cost_vdc*handled_vehicles(v)...
        + overflow_shuttle_cost*overflow_vehicles(v) + overflow_variable_cost*overflow_days(v);
    vdc_cost = vdc_cost + cost;
end

% Total cost
total_cost = logistics_cost + late_cost + vdc_cost;



%% Visualization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Below are codes for visualization                  %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Visualize a milkrun tour
dealer_vehicle_map = containers.Map('KeyType','double','ValueType','any');
look_ahead_time = 2;   % 2 days

k = keys(final_arrival);
final_v = k{6};     % choose an arbitrary final vdc to visualize results
final_v_loc = get_location(final_v,location);
vehicles = final_arrival(final_v);
path = vehicles{1,3};
center_dealer = path{end};
center_loc = get_location(center_dealer,location);

horizon = min(50,size(vehicles,1));
for i = 1:horizon
    veh = vehicles(i,:);
    path = veh{3};
    d = path{end};
    if ~isKey(dealer_vehicle_map,d)
        dealer_vehicle_map(d) = veh;
    else
        val = dealer_vehicle_map(d);
        val(end+1,:) = veh;
        dealer_vehicle_map(d) = val;
    end
end
delivery_map = forward_sweep(final_v,center_dealer,dealer_vehicle_map,location);
selected_dealers = cell2mat(keys(delivery_map));
[tour,total_dist] = three_opt(final_v,selected_dealers,location);

figure();
hold on;
grid on;
lats = zeros(1,length(tour));
longs = zeros(1,length(tour));
for i = 1:length(tour)
    loc = get_location(tour{i},location);
    lats(i) = loc(1);
    longs(i) = loc(2);
end
p1 = plot(mod(longs+360,360)-180,lats,'-r*');

deals = cell2mat(keys(dealer_vehicle_map));
lats = zeros(1,length(deals));
longs = zeros(1,length(deals));
for i = 1:length(deals)
    loc = get_location(deals(i),location);
    lats(i) = loc(1);
    longs(i) = loc(2);
end
p2 = plot(mod(longs+360,360)-180,lats,'bd','MarkerSize',8);
p3 = plot(mod(center_loc(2)+360,360)-180,center_loc(1),'ks','MarkerSize',15);
p4 = plot(mod(final_v_loc(2)+360,360)-180,final_v_loc(1),'ko','MarkerSize',10);
legend([p1,p2,p3,p4],{'selected','dealers','center dealer','final VDC'});
axis equal;

%% Visualize distribution of car production
first_date = floor(min(plant_arrival_time));
last_date = first_date + 365*2;
duration = last_date-first_date+1;
vehicle_distribution = zeros(1,duration);

for i = 1:length(shipment_req)
    idx = floor(plant_arrival_time(i))-first_date+1;
    if idx <= duration
        vehicle_distribution(idx) = vehicle_distribution(idx) + 1;
    end
end

figure();
hold on;
plot(first_date:last_date, vehicle_distribution);
plot([first_date+365,first_date+365],[0 7000],'LineWidth',1.5);
grid on;
datetick('x');
title('Vehicle production distribution (2015-2017)');

%% Visualize VDC dealer pairs
figure();
hold on;
grid on;
k = keys(VDC2dealer);
colors = hsv(length(k));
tic
for i = 1:length(k)
    v = k{i};
    v_loc = get_location(v,location);
    ds = VDC2dealer(v);
    ds = ds{:};
    for j = 1:length(ds)
        d = ds(j);
        d_loc = get_location(d,location);
        plot(mod(d_loc(2)+360,360)-180,d_loc(1),'.','color',colors(i,:));
    end
    plot(mod(v_loc(2)+360,360)-180,v_loc(1),'*k');
end
xlim([-60 60]);
axis equal;
xlabel('longitude (offset = 180 degree)');
ylabel('latitude');
toc

%% Visualize shortest path from plants to final VDCs
dealer_loc = zeros(numel(dealers),2);
for i = 1:numel(dealers)
    d_loc = get_location(dealers{i},location);
    dealer_loc(i,:) = d_loc;
end
VDC_loc = zeros(numel(VDCs),2);
for i = 1:numel(VDCs)
    v_loc = get_location(VDCs{i},location);
    VDC_loc(i,:) = v_loc;
end

tic
for i = 1:numel(plants)
    figure();
    hold on;
    grid on;
    plot(mod(dealer_loc(:,2)+360,360)-180,dealer_loc(:,1),'b.');
    plot(mod(VDC_loc(:,2)+360,360)-180,VDC_loc(:,1),'r*');
    
    p = plants{i};
    for j = 1:numel(final_VDCs)
        key = [p,' ',final_VDCs{j}];
        if ~isKey(shortest_routes,key)
            continue;
        end
        path = shortest_routes(key);
        locs = zeros(numel(path),2);
        for k = 1:numel(path)
            locs(k,:) = get_location(path{k},location);
        end
        plot(mod(locs(:,2)+360,360)-180,locs(:,1),'-kd','LineWidth',1.5);
    end
    xlim([-60 60]);
    axis equal;
    xlabel('longitude (offset = 180 degree)');
    ylabel('latitude');
    title(['Plant ',plants{i}]);
    legend('dealer','VDC');
end
toc

%% Attribution
% Name: Guangzhao Yang
%       Sumaanyu Maheshwari
%       Wesley Graham
%       Aya Hamoodi
