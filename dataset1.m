vdccost = 'DataSet1/Results_Template_1_VDC_DataSet1.csv';
shipdetails = 'DataSet1/Results_Template_4_ShipDetails_DataSet1.csv';
leadtime = 'DataSet1/Results_Template_6_Leadtime_DataSet1.csv';

%%
fid = fopen(vdccost);
A = textscan(fid,'%q%f%f%f%f%f%f%f%f%q%q','Delimiter',',','HeaderLines',1);
fclose(fid);
row = numel(A{1});
col = numel(A);
vdc_cost = cell(row,col);
vdc_cost(:,1) = A{1};
vdc_cost(:,2) = num2cell(A{2});
vdc_cost(:,3) = num2cell(A{3});
vdc_cost(:,4) = num2cell(A{4});
vdc_cost(:,5) = num2cell(A{5});
vdc_cost(:,6) = num2cell(A{6});
vdc_cost(:,7) = num2cell(A{7});
vdc_cost(:,8) = num2cell(A{8});
vdc_cost(:,9) = num2cell(A{9});
vdc_cost(:,10) = A{10};
vdc_cost(:,11) = A{11};

%%
fid = fopen(shipdetails);
B = textscan(fid, '%q%f%f%f', 'Delimiter', ',', 'HeaderLines', 1);
fclose(fid);
row = numel(B{1});
col = numel(B);
ship_details = cell(row,col);
ship_details(:,1) = B{1};
ship_details(:,2) = num2cell(B{2});
ship_details(:,3) = num2cell(B{3});
ship_details(:,4) = num2cell(B{4});

%%
fid = fopen(leadtime);
C = textscan(fid, '%q%f', 'Delimiter', ',', 'HeaderLines', 1);
fclose(fid);
row = numel(C{1});
col = numel(C);
lead_time = cell(row,col);
lead_time(:,1) = C{1};
lead_time(:,2) = num2cell(C{2});

%%
total_vdc_cost = sum(cell2mat(vdc_cost(:,end-2)));
logistics_cost = sum(cell2mat(ship_details(:,end)));
late_cost = 10*sum(cell2mat(lead_time(:,end)));
total_cost = logistics_cost+late_cost+total_vdc_cost;
