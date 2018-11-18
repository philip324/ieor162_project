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
fprintf('Processing time: %i\n',toc);

%% main script
VDC_loc_idx = @(s)find(cellfun(@(x)isequal(x,s), location(1:end,1)));
VDC_cap_idx = @(s)find(cellfun(@(x)isequal(x,s), VDC_capacity(1:end,1)));
dealer_loc_idx = @(i) i;

num_VDC = numel(VDC_capacity(2:end,1));
num_vehicle = numel(shipment_req(2:end,1));
active_dealer = union(cell2mat(shipment_req(2:end,3)),shipment_req{2,3});
num_dealer = numel(active_dealer);







%% closest_VDC = Map(dealer)
% Caution: this function takes a long time to run...
key_dealer = cell(3,1);
val_VDC = cell(3,1);
tic
for i = 1:5%num_dealer
    d = active_dealer(i);
    d_lat = location{dealer_loc_idx(d), 3};
    d_long = location{dealer_loc_idx(d), 4};
    min_dist = inf;
    closest_VDC = '';
    for j = 1:num_VDC
        v = VDC_capacity{j+1,1};
        v_lat = location{VDC_loc_idx(v), 3};
        v_long = location{VDC_loc_idx(v), 4};
        dist = road_dist(d_lat,d_long,v_lat,v_long);
        if dist < min_dist
            min_dist = dist;
            closest_VDC = v;
        end
    end
    key_dealer(i) = {d};
    val_VDC(i) = {closest_VDC};
end
dealer2VDC = containers.Map(key_dealer,val_VDC);
toc

%% plot all VDCs and dealers' location
dealer_loc = cell(length(active_dealer),3);
for i = 1:length(active_dealer)
    idx = dealer_loc_idx(active_dealer(i));
    dealer_loc(i,:) = {location{idx,1},location{idx,3},location{idx,4}};
end
lat_dealer = cell2mat(dealer_loc(:,2));
long_dealer = cell2mat(dealer_loc(:,3));

VDC_loc = cell(num_VDC,3);
for i = 1:num_VDC
    idx = VDC_loc_idx(VDC_capacity{i+1,1});
    VDC_loc(i,:) = {location{idx,1},location{idx,3},location{idx,4}};
end
lat_VDC = cell2mat(VDC_loc(:,2));
long_VDC = cell2mat(VDC_loc(:,3));

figure();
hold on;
plot(mod(long_dealer+360,360)-180, lat_dealer, 'b*');
plot(mod(long_VDC+360,360)-180, lat_VDC, 'r*');
legend('dealer','VDC');
axis([-180 180 0 60]);
xlabel('longitude (offset = 180 degree)');
ylabel('latitude');
grid on;

%% Attribution
% Name: Aya Hamoodi
%       Guangzhao Yang
%       Sumaanyu Maheshwari
%       Wesley Graham
