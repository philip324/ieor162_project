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
load processed_data.mat
input_data = 'Input_Cost%2C+Location.xlsx';
dataset_1 = 'dataset_1/VehicleShipmentRequirement_DataSet1';
dataset_2 = 'dataset_2/VehicleShipmentRequirement_DataSet2';
final_prob = 'final_problem/Problem_VehicleShipmentRequirement.csv';

shipment_data = final_prob;

[~,~,location] = xlsread(input_data,1);
[~,~,VDC_capacity] = xlsread(input_data,2);
[~,~,VDC_cost_model] = xlsread(input_data,3);
[~,~,trans_cost_model] = xlsread(input_data,4);
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
location(1,:) = [];
VDC_capacity(1,:) = [];

fid = fopen(shipment_data);
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

%% Get lists of dealers and VDCs
dealers = num2cell(union(cell2mat(shipment_req(:,3)),shipment_req{1,3}));
VDCs = VDC_capacity(:,1);
plants = union(shipment_req(:,2),shipment_req{1,2});
final_VDCs = keys(VDC2dealer)';

%% Run dijkstra to find the static routes
SID = zeros(1,numel(plants));
for i = 1:numel(plants)
    lst = all(ismember(VDCs,plants(i)),2);
    SID(i) = find(lst);
end

FID = zeros(1,numel(final_VDCs));
for i = 1:numel(final_VDCs)
    lst = all(ismember(VDCs,final_VDCs(i)),2);
    FID(i) = find(lst);
end

shortest_routes = containers.Map('KeyType','char','ValueType','any');
for i = 1:numel(plants)
    for j = 1:numel(final_VDCs)
        if SID(i) ~= FID(j)
            [~,path] = dijkstra(connection,edge_costs,SID(i),FID(j));
            val = cell(1);
            for k = 1:numel(path)
                v = VDCs{path(k)};
                val(k) = {v};
            end
            shortest_routes([plants{i},' ',final_VDCs{j}]) = val;
        end
    end
end

%% Initializing variables
% from VDC to number
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

% from final VDC to vehicle info
final_arrival = containers.Map('KeyType','char','ValueType','any');
for i = 1:numel(final_VDCs)
    final_arrival(final_VDCs{i}) = cell(0,4);
end

% from edges to vehicle info
pending_vehicles = containers.Map('KeyType','char','ValueType','any');
for i = 1:numel(VDCs)
    for j = 1:numel(VDCs)
        if i ~= j
            pending_vehicles([VDCs{i},' ',VDCs{j}]) = cell(0,4);
        end
    end
end

% % from each vehicle to routing detail
% routing_map = containers.Map('KeyType','char','ValueType','any');
% for i = 1:length(shipment_req)
%     % value = n by 4 cell. (loc, arrive_time, depart_time, depart_mode)
%     routing_map(shipment_req{i,1}) = cell(0,4);
% end

%%
tic
load arrival_time_map.mat
toc

%% Find final VDC arrival time for each vehicle
timeline = sort(cell2mat(keys(arrival_time_map)));
cutoff = 1e4;
timeline = timeline(1:cutoff);

tic
count = 0;
while size(timeline,2) > 0
    curr_time = timeline(1);
    arriving_vehicles = arrival_time_map(curr_time);
    timeline(1) = [];
%     remove(arrival_time_map,curr_time);
    for i = 1:size(arriving_vehicles,1)
        % Arriving to VDC
        arrive_veh  = arriving_vehicles(i,:);
        vid         = arrive_veh{1};
        path        = arrive_veh{3};
        modes       = arrive_veh{4};
        
        curr_capacity(path{1}) = curr_capacity(path{1}) + 1;
        handled_vehicles(path{1}) = handled_vehicles(path{1}) + 1;
        if length(path) == 2
            % arrived at final VDC
            val = final_arrival(path{1});
            val(end+1,:) = arrive_veh;
            final_arrival(path{1}) = val;
            continue;
        end
        
        curr_VDC = path{1};
        next_VDC = path{2};
        mode = modes{1};
        wait_list = pending_vehicles([curr_VDC,' ',next_VDC]);
        wait_list(end+1,:) = arrive_veh;
        curr_load = size(wait_list,1);
        not_full = ((strcmp(mode,'T') && curr_load < 10) || ...
                   (strcmp(mode,'R') && curr_load < 20));
        if not_full
            pending_vehicles([curr_VDC,' ',next_VDC]) = wait_list;
            % check if overflowed
            if curr_capacity(curr_VDC) > get_capacity(curr_VDC,VDC_capacity)
                overflow_vehicles(curr_VDC) = overflow_vehicles(curr_VDC) + 1;
            end
            continue;
        end
        
        % Later: subtract 10 or 20 from current capacity
        pending_vehicles([curr_VDC,' ',next_VDC]) = cell(0,4);
        curr_loc = get_location(curr_VDC,location);
        next_loc = get_location(next_VDC,location);
        dist = road_dist(curr_loc,next_loc);
        if strcmp(mode,'T')
            duration = (dist/30)/24;
        elseif strcmp(mode,'R')
            duration = (dist/10)/24;
        end
        t = datetime(curr_time+duration,'ConvertFrom','datenum');
        t = dateshift(t,'start','minute','nearest');
        next_time = datenum(t);
%         next_arrive_time = curr_time+duration;
        new_val = cell(0,4);
        for j = 1:curr_load
            depart_veh  = wait_list(j,:);
            vid         = depart_veh{1};
            arrive_time = depart_veh{2};
            path        = depart_veh{3};
            modes       = depart_veh{4};
%             routing_val = routing_map(vid);
%             routing_val(end+1,:) = {curr_VDC, arrive_time, ...
%                                     depart_time, mode};
            depart_veh(2) = {next_time};
            path(1) = [];
            depart_veh(3) = {path};
            modes(1) = [];
            depart_veh(4) = {modes};
            new_val(j,:) = depart_veh;
        end
        arrival_time_map(next_time) = new_val;
        
        if size(timeline,2) == 0
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
toc

%% Last leg: distribute from final VDC to dealers
% implement sweep algorithm here












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
% day 425 (Feb. 29th, 2016) is 0
% day 59 (Feb. 28th, 2015) is 6317
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
