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

load VDC_dealer_pairs
input_data = 'Input_Cost%2C+Location.xlsx';
dataset_1 = 'dataset_1/VehicleShipmentRequirement_DataSet1';
dataset_2 = 'dataset_2/VehicleShipmentRequirement_DataSet2';
final_prob = 'final_problem/Problem_VehicleShipmentRequirement.csv';

shipment_data = final_prob;

tic
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

fid = fopen(shipment_data);
header = textscan(fid, '%q%q%q%q', 1, 'Delimiter', ',');
C = textscan(fid, '%q%q%f%q', 'Delimiter', ',', 'HeaderLines', 1);
fclose(fid);
row = numel(C{1});
col = numel(C);
shipment_req = cell(row+1,col);
shipment_req(1,:) = [header{1},header{2},header{3},header{4}];
shipment_req(2:end,1) = C{1};
shipment_req(2:end,2) = C{2};
shipment_req(2:end,3) = num2cell(C{3});
shipment_req(2:end,4) = C{4};
disp(['Processing time: ',num2str(round(toc,2)),' sec']);

%% main script
dealers = union(cell2mat(shipment_req(2:end,3)),shipment_req{2,3});
dealers = mat2cell(dealers,ones(length(dealers),1));
VDCs = VDC_capacity(2:end,1);
plants = union(shipment_req(2:end,2),shipment_req{2,2});
final_VDCs = keys(VDC2dealer)';

% find all shortest routes from plants to final VDCs
% [connection,edge_costs] = build_graph(VDCs,location,VDC_capacity);
load graph

%%
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






%% get all VDCs and dealers' locations
dealer_locations = zeros(numel(dealers),2);
for i = 1:numel(dealers)
    d_loc = get_location(dealers{i},location);
    dealer_locations(i,:) = d_loc';
end
VDC_locations = zeros(numel(VDCs),2);
for i = 1:numel(VDCs)
    v_loc = get_location(VDCs{i},location);
    VDC_locations(i,:) = v_loc';
end

%% Plot shortest path from plant to other VDCs
tic
for i = 1:numel(plants)
    figure();
    hold on;
    grid on;
    plot(mod(dealer_locations(:,2)+360,360)-180,dealer_locations(:,1),'b.');
    plot(mod(VDC_locations(:,2)+360,360)-180,VDC_locations(:,1),'r*');
%     legend('dealer','VDC');
    axis([-100 100 0 60]);
    xlabel('longitude (offset = 180 degree)');
    ylabel('latitude');
    title(['Plant ',plants{i}]);
    
    p = plants{i};
    for j = 1:numel(final_VDCs)
        if ~isKey(shortest_routes,[p,' ',final_VDCs{j}])
            continue;
        end
        path = shortest_routes([p,' ',final_VDCs{j}]);
        locs = zeros(numel(path),2);
        for k = 1:numel(path)
            locs(k,:) = get_location(path{k},location);
        end
        plot(mod(locs(:,2)+360,360)-180,locs(:,1),'-kd','LineWidth',1.5);
    end
end
toc

%% visualize VDC dealer pairs
figure();
hold on;
grid on;
k = keys(VDC2dealer);
colors = hsv(length(k));
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
axis([-100 100 0 60]);
xlabel('longitude (offset = 180 degree)');
ylabel('latitude');

%% Attribution
% Name: Aya Hamoodi
%       Guangzhao Yang
%       Sumaanyu Maheshwari
%       Wesley Graham
