%% IEOR 162, Project

%% 
homedir = pwd;
new_dir = sprintf('%s/final_problem', homedir);
if ~exist(new_dir, 'dir')
    mkdir(new_dir);
end

%% load data
% Evaluate cell element using C{i,j}, view cell element using C(i,j).
clear;
close all;
clc

tic
load arrival_time_map.mat
sorted_plant_arrival_time = sort(cell2mat(keys(arrival_time_map)));

load processed_data.mat
input_data = 'Input_Cost%2C+Location.xlsx';
dataset_1 = 'dataset_1/VehicleShipmentRequirement_DataSet1';
dataset_2 = 'dataset_2/VehicleShipmentRequirement_DataSet2';
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

%% Shortest Paths
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











%% Initializing variables
% 'vdc' --> double
handled_vehicles = containers.Map('KeyType','char','ValueType','double');
overflow_vehicles = containers.Map('KeyType','char','ValueType','double');
overflow_days = containers.Map('KeyType','char','ValueType','double');
curr_capacity = containers.Map('KeyType','char','ValueType','double');
for i = 1:numel(VDCs)
    handled_vehicles(VDCs{i}) = 0;
    overflow_vehicles(VDCs{i}) = 0;
    overflow_days(VDCs{i}) = 0;
    curr_capacity(VDCs{i}) = 0;
end

% 'vdc' --> (vid, arrive_time, path, modes)
final_arrival = containers.Map('KeyType','char','ValueType','any');
% 'vdc1 vdc2' --> (vid, arrive_time, path, modes)
pending_vehicles = containers.Map('KeyType','char','ValueType','any');
% 'vdc1 vdc2' --> double
edge_flows = containers.Map('KeyType','char','ValueType','double');
% 'vid' --> (loc, arrive_time, depart_time, depart_mode)
routing_map = containers.Map('KeyType','char','ValueType','any');

%% Choose a subset of the dataset
cutoff = 1e3;
timeline = sorted_plant_arrival_time(1:cutoff);
extra_field = 1;

tic
all_arrivals = containers.Map('KeyType','double','ValueType','any');
for i = 1:cutoff
    t = timeline(i);
    vehicles = arrival_time_map(t);
    dim = size(vehicles) + [0 extra_field];
    new_vehicles = cell(dim);
    for j = 1:size(vehicles,1)
        veh = vehicles(j,:);
        path = veh{3};
        for k = 1:length(path)-1
            vdc = path{k};
            handled_vehicles(vdc) = handled_vehicles(vdc) + 1;
            if k > 1
                edge = [path{k-1},' ',path{k}];
                if ~isKey(edge_flows, edge)
                    edge_flows(edge) = 0;
                end
                edge_flows(edge) = edge_flows(edge) + 1;
            end
        end
        % add new fields to vehicle
        curr_VDC = path{1};
        new_veh = {veh{1},veh{2},veh{3},veh{4}, curr_VDC};
        new_vehicles(j,:) = new_veh;
    end
    all_arrivals(t) = new_vehicles;
end
toc

%
total = 0;
vals = values(all_arrivals);
for i = 1:length(vals)
    vs = vals{i};
    for j = 1:size(vs,1)
        path = vs{j,3};
        total = total + length(path)-2;
    end
end

%% Find final VDC arrival time for each vehicle
tic
count = 0;
while ~isempty(timeline)
    curr_time = timeline(1);
    arriving_vehicles = all_arrivals(curr_time);
    timeline(1) = [];
%     remove(all_arrivals,curr_time);
    for i = 1:size(arriving_vehicles,1)
        % Arrive at current VDC
        arrive_veh  = arriving_vehicles(i,:);
        vid         = arrive_veh{1};
        path        = arrive_veh{3};
        modes       = arrive_veh{4};
        curr_VDC    = arrive_veh{5};
        curr_idx    = find(ismember(path(1:end-1),{curr_VDC}),1);
        
        curr_capacity(curr_VDC) = curr_capacity(curr_VDC) + 1;
%         if curr_capacity(curr_VDC) > get_capacity(curr_VDC,VDC_capacity)
%             overflow_vehicles(curr_VDC) = overflow_vehicles(curr_VDC)+1;
%         end
        
        % current VDC is the final VDC
        if strcmp(curr_VDC, path{end-1})
            if ~isKey(final_arrival,curr_VDC)
                final_arrival(curr_VDC) = arrive_veh;
            else
                val = final_arrival(curr_VDC);
                val(end+1,:) = arrive_veh;
                final_arrival(curr_VDC) = val;
            end
            continue;
        end
        
        next_VDC = path{curr_idx+1};
        mode = modes{1};
        edge = [curr_VDC,' ',next_VDC];
        edge_flows(edge) = edge_flows(edge) - 1;
        if ~isKey(pending_vehicles,edge)
            wait_list = arrive_veh;
        else
            wait_list = pending_vehicles(edge);
            wait_list(end+1,:) = arrive_veh;
        end
        wl_len = size(wait_list,1);
        full_load = ((strcmp(mode,'T') && wl_len >= 10) || ...
                     (strcmp(mode,'R') && wl_len >= 20) || ...
                     edge_flows(edge) == 0);
        % not a full load
        if ~full_load
            pending_vehicles(edge) = wait_list;
            continue;
        end
        
        % full load, send vehicles to the next destination
        if isKey(pending_vehicles,edge)
            remove(pending_vehicles,edge);
        end
        curr_loc = get_location(curr_VDC,location);
        next_loc = get_location(next_VDC,location);
        dist = road_dist(curr_loc,next_loc);
        if strcmp(mode,'T')
            duration = (dist/30)/24;    % unit: days
        elseif strcmp(mode,'R')
            duration = (dist/10)/24;    % unit: days
        end
        % round to minute
        t = datetime(curr_time+duration,'ConvertFrom','datenum');
        t = dateshift(t,'start','minute','nearest');
        next_time = datenum(t);
        
        % depart from current VDC
        curr_capacity(curr_VDC) = curr_capacity(curr_VDC) - wl_len;
        new_arrival = cell(wl_len,length(arrive_veh));
        for j = 1:wl_len
            depart_veh  = wait_list(j,:);
            vid         = depart_veh{1};
            arrive_time = depart_veh{2};
            new_arrival(j,:) = {vid,next_time,depart_veh{3},depart_veh{4},next_VDC};
            
            details = {curr_VDC,arrive_time,curr_time,mode};
            if ~isKey(routing_map,vid)
                routing_map(vid) = details;
            else
                routing_val = routing_map(vid);
                routing_val(end+1,:) = details;
            end
        end
        if ~isKey(all_arrivals, next_time)
            all_arrivals(next_time) = new_arrival;
        else
            val = all_arrivals(next_time);
            val(end+1:end+wl_len,:) = new_arrival;
            all_arrivals(next_time) = val;
        end
        
        % add new arrival time to timeline
        if isempty(timeline)
            timeline = next_time;
            continue;
        end
        if next_time < timeline(1)
            timeline = [next_time, timeline];
        elseif next_time > timeline(end)
            timeline = [timeline, next_time];
        else
            for k = 1:numel(timeline)-1
                if timeline(k) < next_time && next_time < timeline(k+1)
                    break;
                end
            end
            timeline = [timeline(1:k),next_time,timeline(k+1:end)];
        end
    end
    
    count = count + 1;
    if mod(count,cutoff/10) == 0
        disp(['iteration ',num2str(count)]);
        disp(['Processing time: ',num2str(round(toc,2)),' sec']);
        disp(' ');
        tic
    end
end

k = keys(edge_flows);
for i = 1:length(k)
    if edge_flows(k{i}) == 0
        remove(edge_flows,k{i});
    end
end
toc

% TODO: 1. overflow
%       2. remnant vehicles stuck in the network

%% Last leg: distribute from final VDC to dealers
look_ahead_time = 0.5;   % 12 hours
for i = 1:length(final_VDCs)
    final_v = final_VDCs{i};
    vehicles = final_arrival(final_v);
    path = vehicles{1,3};
    center_dealer = path{end};
    % find horizon
    horizon = 1;
    while vehicles{horizon,2} - vehicles{1,2} < look_ahead_time
        horizon = horizon + 1;
    end
    
    for j = 1:horizon
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
    route_map = forward_sweep(final_v,center_dealer,dealer_vehicle_map,location);
    % run the travelling salesman algorithm
    
end


% nStop = length(route_map);














%% 
k = keys(final_arrival);
final_v = k{5};
final_v_loc = get_location(final_v,location);
vehicles = final_arrival(final_v);
path = vehicles{1,3};
center_dealer = path{end};
center_loc = get_location(center_dealer,location);
dealer_vehicle_map = containers.Map('KeyType','double','ValueType','any');
horizon = min(size(vehicles,1), 50);
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
route_map = forward_sweep(final_v,center_dealer,dealer_vehicle_map,location);

%% plots
lats = zeros(1,length(tour));
longs = zeros(1,length(tour));
selected_dealers = cell2mat(keys(route_map));
tour = travelling_salesman(final_v,selected_dealers,location);

total_dist = 0;
for i = 1:length(tour)
    loc = get_location(tour{i},location);
    lats(i) = loc(1);
    longs(i) = loc(2);
    if i > 1
        loc2 = get_location(tour{i-1},location);
        total_dist = total_dist + road_dist(loc2,loc);
    end
end

figure();
hold on;
grid on;
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

%% Distribution of car productionplant_arrival_time
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

%% get all VDCs and dealers' locations
tic
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
toc

%% Plot shortest path from plant to other VDCs
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

%% visualize VDC dealer pairs
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

%% Attribution
% Name: Aya Hamoodi
%       Guangzhao Yang
%       Sumaanyu Maheshwari
%       Wesley Graham
