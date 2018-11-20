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
num_VDC = numel(VDCs);
num_dealer = numel(dealers);
num_plant = numel(plants);

%% Create 2 maps
% dealer2VDC = containers.Map('KeyType','double','ValueType','any');
% VDC2dealer = containers.Map('KeyType','char','ValueType','any');
% tic
% for i = 1:num_dealer
%     d = dealers{i};
%     d_loc = dealer_loc(d,location);
%     min_dist = inf;
%     closest_VDC = '';
%     for j = 1:num_VDC
%         v = VDCs{j};
%         v_loc = VDC_loc(v,location);
%         dist = road_dist(d_loc(1),d_loc(2),v_loc(1),v_loc(2));
%         if dist < min_dist
%             min_dist = dist;
%             closest_VDC = v;
%         end
%     end
%     
%     dealer2VDC(d) = closest_VDC;
%     if ~isKey(VDC2dealer,closest_VDC)
%         VDC2dealer(closest_VDC) = {d};
%     else
%         val = VDC2dealer(closest_VDC);
%         val{:} = [val{:}, d];
%         VDC2dealer(closest_VDC) = val;
%     end
%     
%     if mod(i,50) == 0
%         disp(['iteration ',num2str(i)]);
%         disp(['Processing time: ',num2str(round(toc,2)),' sec']);
%         disp(' ');
%         tic
%     end
% end
% save('VDC_dealer_pairs.mat','dealer2VDC','VDC2dealer');

%% plot all VDCs and dealers' location
dealer_locations = zeros(2,num_dealer);
for i = 1:num_dealer
    d_loc = dealer_loc(dealers{i},location);
    dealer_locations(:,i) = d_loc';
end
VDC_locations = zeros(2,num_VDC);
for i = 1:num_VDC
    v_loc = VDC_loc(VDCs{i},location);
    VDC_locations(:,i) = v_loc';
end

figure();
hold on;
grid on;
plot(mod(dealer_locations(2,:)+360,360)-180,dealer_locations(1,:),'b.');
plot(mod(VDC_locations(2,:)+360,360)-180,VDC_locations(1,:),'r*');
legend('dealer','VDC');
axis([-100 100 0 60]);
xlabel('longitude (offset = 180 degree)');
ylabel('latitude');

%% visualize VDC dealer pairs
figure();
hold on;
grid on;
k = keys(VDC2dealer);
colors = hsv(length(k));
for i = 1:length(k)
    v = k{i};
    v_loc = VDC_loc(v,location);
    ds = VDC2dealer(v);
    ds = ds{:};
    for j = 1:length(ds)
        d = ds(j);
        d_loc = dealer_loc(d,location);
        plot(mod(d_loc(2)+360,360)-180,d_loc(1),'.','color',colors(i,:));
    end
    plot(mod(v_loc(2)+360,360)-180,v_loc(1),'*k');
end
axis([-100 100 0 60]);
xlabel('longitude (offset = 180 degree)');
ylabel('latitude');

%% helper functions
function v_loc = VDC_loc(v,location)
VDC_loc_idx = @(s)find(cellfun(@(x)isequal(x,s), location(1:end,1)));
v_lat = location{VDC_loc_idx(v), 3};
v_long = location{VDC_loc_idx(v), 4};
v_loc = [v_lat, v_long];
end

function d_loc = dealer_loc(d,location)
dealer_loc_idx = @(i) i;
d_lat = location{dealer_loc_idx(d), 3};
d_long = location{dealer_loc_idx(d), 4};
d_loc = [d_lat, d_long];
end

function v_cap = VDC_cap(v,VDC_capacity)
VDC_cap_idx = @(s)find(cellfun(@(x)isequal(x,s), VDC_capacity(1:end,1)));
v_cap = VDC_capacity{VDC_cap_idx(v),1};
end

%% Attribution
% Name: Aya Hamoodi
%       Guangzhao Yang
%       Sumaanyu Maheshwari
%       Wesley Graham
