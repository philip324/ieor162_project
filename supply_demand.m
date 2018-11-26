function [supply,demand] = supply_demand(shipment_req)
supply = containers.Map('KeyType','char','ValueType','double');
demand = containers.Map('KeyType','double','ValueType','double');
tic
for i = 1:length(shipment_req)
    p = shipment_req{i,2};
    d = shipment_req{i,3};
    if ~isKey(supply,p)
        supply(p) = 1;
    else
        supply(p) = supply(p) + 1;
    end
    if ~isKey(demand,d)
        demand(d) = 1;
    else
        demand(d) = demand(d) + 1;
    end
    
    if mod(i,1e5) == 0
        disp(['iteration ',num2str(i)]);
        disp(['Processing time: ',num2str(round(toc,2)),' sec']);
        disp(' ');
        tic
    end
end
toc
end