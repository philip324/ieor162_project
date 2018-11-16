%% IEOR 162, Project

%% Preprocess Data
clear;
close all;

% location, capacity_VDC, VDC_cost, logistics_cost, shipment_req 
% are all cell matrices.
% Notice the difference between C(i,j) and C{i,j}.
input_data = 'data/Input_Cost%2C+Location.xlsx';
[~,~,location] = xlsread(input_data,1);
[~,~,capacity_VDC] = xlsread(input_data,2);
[~,~,VDC_cost] = xlsread(input_data,3);
[~,~,logistics_cost] = xlsread(input_data,4);
VDC_cost = VDC_cost(1:4,:);

shipment_data = 'data/Problem_VehicleShipmentRequirement.csv';
fid = fopen(shipment_data);
header = textscan(fid,'%q %q %q %q',1,'Delimiter',',');
C = textscan(fid,'%q %q %d64 %q','Delimiter',',','HeaderLines',1);
fclose(fid);
row = numel(C{1});
col = numel(C);
shipment_req = cell(row+1,col);
shipment_req(1,:) = {header{1}{1},header{2}{1},header{3}{1},header{4}{1}};
shipment_req(2:end,1) = C{1};
shipment_req(2:end,2) = C{2};
shipment_req(2:end,3) = num2cell(C{3});
shipment_req(2:end,4) = C{4};

%% main script


%% Attribution
% Name: Aya Hamoodi
%       Guangzhao Yang
%       Sumaanyu Maheshwari
%       Wesley Graham